#include "http.h"
#include "demo_tile.h"
#include "net.h"
#include "xmalloc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Required by net.c (extern const int verb) */
const int verb = 0;

/* ── net_main callbacks ──────────────────────────────────────────────── */

static void on_listening(void *udata) {
    (void)udata;
}

static void on_ready(void *udata) {
    struct server_ctx *ctx = udata;
    printf("Serving tiles from %s\n", ctx->tile_dir);
}

static void on_opened(struct net_conn *conn, void *udata) {
    (void)udata;
    http_conn *hc = http_conn_new();
    net_conn_setudata(conn, hc);
}

static void on_closed(struct net_conn *conn, void *udata) {
    (void)udata;
    http_conn *hc = net_conn_udata(conn);
    http_conn_free(hc);
}

static void on_data(struct net_conn *conn, const void *data, size_t nbytes,
                    void *udata)
{
    struct server_ctx *ctx = udata;
    http_conn *hc = net_conn_udata(conn);
    http_conn_feed(hc, conn, ctx, data, nbytes);
}

/* ── Entry point ─────────────────────────────────────────────────────── */

int main(int argc, char *argv[]) {
    if (argc < 2 || argc > 4) {
        fprintf(stderr, "Usage: arpt_server <tile_dir> [port] [threads]\n");
        return 1;
    }

    const char *tile_dir = argv[1];
    const char *port = argc >= 3 ? argv[2] : "8090";
    int nthreads = argc >= 4 ? atoi(argv[3]) : 1;
    if (nthreads < 1) nthreads = 1;

    /* Build demo tile once */
    struct server_ctx ctx = {
        .tile_dir = tile_dir,
        .demo_tile = NULL,
        .demo_tile_size = 0,
    };

    if (!arpt_build_demo_tile(&ctx.demo_tile, &ctx.demo_tile_size)) {
        fprintf(stderr, "Failed to build demo tile\n");
        return 1;
    }
    printf("Built demo tile (%zu bytes compressed)\n", ctx.demo_tile_size);

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

    printf("Listening on %s:%s (%d thread%s)\n",
           opts.host, opts.port, nthreads, nthreads > 1 ? "s" : "");

    /* Blocks forever */
    net_main(&opts);

    /* Unreachable, but tidy */
    free(ctx.demo_tile);
    return 0;
}
