/* Minimal stubs so test_http can link http.c without the full net.c */
#include <stddef.h>

struct net_conn;

void net_conn_close(struct net_conn *conn) { (void)conn; }
void net_conn_out_write(struct net_conn *conn, const void *data, size_t nbytes) {
    (void)conn; (void)data; (void)nbytes;
}
