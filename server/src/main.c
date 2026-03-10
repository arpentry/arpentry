#include "http.h"
#include "net.h"
#include "xmalloc.h"
#include "archive.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Required by net.c (extern const int verb) */
const int verb = 0;

/* net_main callbacks */

static void on_listening(void *udata) {
    (void)udata;
}

static void on_ready(void *udata) {
    struct server_ctx *ctx = udata;
    if (ctx->archive) {
        printf("Serving tiles from archive %s (%" PRIu64 " tiles)\n",
               ctx->tile_dir, arpt_archive_reader_tile_count(ctx->archive));
    } else {
        printf("Serving generated tiles from %s\n", ctx->tile_dir);
    }
}

static void on_opened(struct net_conn *conn, void *udata) {
    (void)udata;
    http_conn *hc = http_conn_new();
    if (!hc) { net_conn_close(conn); return; }
    net_conn_setudata(conn, hc);
}

static void on_closed(struct net_conn *conn, void *udata) {
    (void)udata;
    http_conn *hc = net_conn_udata(conn);
    http_conn_free(hc);
}

static void on_data(struct net_conn *conn, const void *data, size_t nbytes,
                    void *udata) {
    struct server_ctx *ctx = udata;
    http_conn *hc = net_conn_udata(conn);
    http_conn_feed(hc, conn, ctx, data, nbytes);
}

/* Entry point */

int main(int argc, char *argv[]) {
    if (argc < 3 || argc > 5) {
        fprintf(stderr,
                "Usage: arpt_server <tile_dir> <style_file> [port] [threads]\n");
        return 1;
    }

    const char *tile_dir = argv[1];
    const char *style_file = argv[2];
    const char *port = argc >= 4 ? argv[3] : "8090";
    int nthreads = argc >= 5 ? atoi(argv[4]) : 8;
    if (nthreads < 1) nthreads = 1;

    /* If tile_dir is an .arpa file, open it as an archive */
    arpt_archive_reader *archive = NULL;
    size_t td_len = strlen(tile_dir);
    if (td_len >= 5 && strcmp(tile_dir + td_len - 5, ".arpa") == 0) {
        archive = arpt_archive_reader_open(tile_dir);
        if (!archive) {
            fprintf(stderr, "Failed to open archive: %s\n", tile_dir);
            return 1;
        }
    }

    struct server_ctx ctx = {
        .tile_dir = tile_dir,
        .style_file = style_file,
        .archive = archive,
    };

    xmalloc_init(nthreads);

    struct net_opts opts = {
        .host = "0.0.0.0",
        .port = port,
        .nthreads = nthreads,
        .maxconns = 10000,
        .queuesize = 1024,
        .backlog = 128,
        .tcpnodelay = true,
        .keepalive = false,
        .nouring = true,
        .nowarmup = true,
        .udata = &ctx,
        .listening = on_listening,
        .ready = on_ready,
        .opened = on_opened,
        .closed = on_closed,
        .data = on_data,
    };

    printf("Listening on %s:%s (%d thread%s)\n", opts.host, opts.port, nthreads,
           nthreads > 1 ? "s" : "");

    /* Blocks forever */
    net_main(&opts);

    /* Unreachable, but tidy */
    return 0;
}
