#include "tile_fetch.h"

#ifdef __EMSCRIPTEN__

/* ═══════════════════════════════════════════════════════════════════════════
 * Emscripten path — browser Fetch API (inherently async)
 * ═══════════════════════════════════════════════════════════════════════════ */

#include <emscripten/fetch.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tile_verifier.h"

typedef struct {
    arpt_tile_fetch_cb cb;
    void *userdata;
} FetchRequest;

static void on_fetch_success(emscripten_fetch_t *fetch) {
    FetchRequest *req = (FetchRequest *)fetch->userData;

    int rc = arpentry_tiles_Tile_verify_as_root_with_identifier(
        fetch->data, (size_t)fetch->numBytes, "arpt");

    if (rc == 0) {
        /* Copy so caller owns the buffer (browser frees fetch->data) */
        uint8_t *copy = malloc((size_t)fetch->numBytes);
        if (copy) {
            memcpy(copy, fetch->data, (size_t)fetch->numBytes);
            req->cb(true, copy, (size_t)fetch->numBytes, req->userdata);
        } else {
            req->cb(false, NULL, 0, req->userdata);
        }
    } else {
        fprintf(stderr, "tile_fetch: FlatBuffer verification failed (rc=%d)\n", rc);
        req->cb(false, NULL, 0, req->userdata);
    }

    emscripten_fetch_close(fetch);
    free(req);
}

static void on_fetch_error(emscripten_fetch_t *fetch) {
    FetchRequest *req = (FetchRequest *)fetch->userData;
    fprintf(stderr, "tile_fetch: HTTP %d for %s\n", fetch->status, fetch->url);
    req->cb(false, NULL, 0, req->userdata);
    emscripten_fetch_close(fetch);
    free(req);
}

bool arpt_fetch_init(int max_concurrent) {
    (void)max_concurrent;
    return true;
}

int arpt_fetch_drain(void) {
    return 0;
}

void arpt_fetch_shutdown(void) {}

bool arpt_fetch_tile(const char *base_url, int level, int x, int y,
                     arpt_tile_fetch_cb cb, void *userdata) {
    if (!base_url || !cb) return false;

    char url[512];
    int n = snprintf(url, sizeof(url), "%s/%d/%d/%d.arpt", base_url, level, x, y);
    if (n < 0 || (size_t)n >= sizeof(url)) return false;

    FetchRequest *req = malloc(sizeof(*req));
    if (!req) return false;
    req->cb = cb;
    req->userdata = userdata;

    emscripten_fetch_attr_t attr;
    emscripten_fetch_attr_init(&attr);
    strcpy(attr.requestMethod, "GET");
    attr.attributes = EMSCRIPTEN_FETCH_LOAD_TO_MEMORY;
    attr.onsuccess = on_fetch_success;
    attr.onerror = on_fetch_error;
    attr.userData = req;

    emscripten_fetch(&attr, url);
    return true;
}

#else /* native: async fetch via pthread thread pool */

/* ═══════════════════════════════════════════════════════════════════════════
 * Native path — pthread thread pool
 *
 * Workers do HTTP + decode off the main thread.
 * Main thread drains results and fires callbacks (safe for GPU/cache ops).
 * ═══════════════════════════════════════════════════════════════════════════ */

#include "http.h"
#include "tile_verifier.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Job: main thread → worker */

typedef struct fetch_job {
    struct fetch_job *next;
    char url[768];
    arpt_tile_fetch_cb cb;
    void *userdata;
} fetch_job;

/* Result: worker → main thread */

typedef struct fetch_result {
    struct fetch_result *next;
    bool success;
    uint8_t *flatbuf;   /* malloc'd, owned by result until callback */
    size_t size;
    arpt_tile_fetch_cb cb;
    void *userdata;
} fetch_result;

/* Thread pool state */

static struct {
    pthread_t *threads;
    int thread_count;

    /* Job queue (main → workers) */
    fetch_job *job_head;
    fetch_job *job_tail;
    pthread_mutex_t job_mutex;
    pthread_cond_t job_cond;

    /* Result queue (workers → main) */
    fetch_result *result_head;
    fetch_result *result_tail;
    pthread_mutex_t result_mutex;

    bool shutdown;
    bool initialized;
} g_pool;

/* Queue helpers */

static void enqueue_job(fetch_job *job) {
    job->next = NULL;
    pthread_mutex_lock(&g_pool.job_mutex);
    if (g_pool.job_tail) {
        g_pool.job_tail->next = job;
    } else {
        g_pool.job_head = job;
    }
    g_pool.job_tail = job;
    pthread_cond_signal(&g_pool.job_cond);
    pthread_mutex_unlock(&g_pool.job_mutex);
}

static fetch_job *dequeue_job(void) {
    /* Called with job_mutex held */
    fetch_job *job = g_pool.job_head;
    if (job) {
        g_pool.job_head = job->next;
        if (!g_pool.job_head) g_pool.job_tail = NULL;
        job->next = NULL;
    }
    return job;
}

static void enqueue_result(fetch_result *result) {
    result->next = NULL;
    pthread_mutex_lock(&g_pool.result_mutex);
    if (g_pool.result_tail) {
        g_pool.result_tail->next = result;
    } else {
        g_pool.result_head = result;
    }
    g_pool.result_tail = result;
    pthread_mutex_unlock(&g_pool.result_mutex);
}

/* Worker thread */

static void *worker_func(void *arg) {
    (void)arg;

    for (;;) {
        /* Wait for a job */
        pthread_mutex_lock(&g_pool.job_mutex);
        while (!g_pool.job_head && !g_pool.shutdown) {
            pthread_cond_wait(&g_pool.job_cond, &g_pool.job_mutex);
        }
        if (g_pool.shutdown && !g_pool.job_head) {
            pthread_mutex_unlock(&g_pool.job_mutex);
            break;
        }
        fetch_job *job = dequeue_job();
        pthread_mutex_unlock(&g_pool.job_mutex);

        if (!job) continue;

        /* Do blocking HTTP + decode off the main thread */
        fetch_result *result = malloc(sizeof(*result));
        if (!result) { free(job); continue; }

        result->cb = job->cb;
        result->userdata = job->userdata;
        result->flatbuf = NULL;
        result->size = 0;
        result->success = false;

        http_response_t resp;
        if (!http_get(job->url, &resp)) {
            enqueue_result(result);
            free(job);
            continue;
        }

        if (resp.status != 200) {
            fprintf(stderr, "tile_fetch: HTTP %d for %s\n", resp.status, job->url);
            free(resp.body);
            enqueue_result(result);
            free(job);
            continue;
        }

        int rc = arpentry_tiles_Tile_verify_as_root_with_identifier(
            resp.body, resp.body_size, "arpt");
        if (rc == 0) {
            result->success = true;
            result->flatbuf = resp.body;
            result->size = resp.body_size;
            resp.body = NULL; /* transfer ownership */
        } else {
            fprintf(stderr, "tile_fetch: verification failed (rc=%d)\n", rc);
        }

        free(resp.body);
        enqueue_result(result);
        free(job);
    }

    return NULL;
}

/* Public API */

bool arpt_fetch_init(int max_concurrent) {
    if (g_pool.initialized) return false;
    if (max_concurrent <= 0) max_concurrent = 4;

    memset(&g_pool, 0, sizeof(g_pool));
    pthread_mutex_init(&g_pool.job_mutex, NULL);
    pthread_cond_init(&g_pool.job_cond, NULL);
    pthread_mutex_init(&g_pool.result_mutex, NULL);

    g_pool.thread_count = max_concurrent;
    g_pool.threads = calloc((size_t)max_concurrent, sizeof(pthread_t));
    if (!g_pool.threads) return false;

    for (int i = 0; i < max_concurrent; i++) {
        if (pthread_create(&g_pool.threads[i], NULL, worker_func, NULL) != 0) {
            /* Partial creation — shut down what we have */
            g_pool.shutdown = true;
            pthread_cond_broadcast(&g_pool.job_cond);
            for (int j = 0; j < i; j++) {
                pthread_join(g_pool.threads[j], NULL);
            }
            free(g_pool.threads);
            g_pool.threads = NULL;
            return false;
        }
    }

    g_pool.initialized = true;
    return true;
}

bool arpt_fetch_tile(const char *base_url, int level, int x, int y,
                     arpt_tile_fetch_cb cb, void *userdata) {
    if (!base_url || !cb) return false;

    fetch_job *job = malloc(sizeof(*job));
    if (!job) return false;

    int n = snprintf(job->url, sizeof(job->url), "%s/%d/%d/%d.arpt",
                     base_url, level, x, y);
    if (n < 0 || (size_t)n >= sizeof(job->url)) {
        free(job);
        return false;
    }
    job->cb = cb;
    job->userdata = userdata;

    enqueue_job(job);
    return true;
}

int arpt_fetch_drain(void) {
    /* Swap out the entire result list under the lock, then process outside */
    pthread_mutex_lock(&g_pool.result_mutex);
    fetch_result *list = g_pool.result_head;
    g_pool.result_head = NULL;
    g_pool.result_tail = NULL;
    pthread_mutex_unlock(&g_pool.result_mutex);

    int count = 0;
    while (list) {
        fetch_result *r = list;
        list = r->next;

        r->cb(r->success, r->flatbuf, r->size, r->userdata);
        /* Caller now owns flatbuf (must free it in the callback) */
        free(r);
        count++;
    }
    return count;
}

void arpt_fetch_shutdown(void) {
    if (!g_pool.initialized) return;

    /* Signal all workers to exit */
    pthread_mutex_lock(&g_pool.job_mutex);
    g_pool.shutdown = true;
    pthread_cond_broadcast(&g_pool.job_cond);
    pthread_mutex_unlock(&g_pool.job_mutex);

    /* Join all worker threads */
    for (int i = 0; i < g_pool.thread_count; i++) {
        pthread_join(g_pool.threads[i], NULL);
    }
    free(g_pool.threads);

    /* Free any remaining jobs */
    fetch_job *job = g_pool.job_head;
    while (job) {
        fetch_job *next = job->next;
        free(job);
        job = next;
    }

    /* Free any remaining results */
    fetch_result *result = g_pool.result_head;
    while (result) {
        fetch_result *next = result->next;
        free(result->flatbuf);
        free(result);
        result = next;
    }

    pthread_mutex_destroy(&g_pool.job_mutex);
    pthread_cond_destroy(&g_pool.job_cond);
    pthread_mutex_destroy(&g_pool.result_mutex);

    memset(&g_pool, 0, sizeof(g_pool));
}

#endif
