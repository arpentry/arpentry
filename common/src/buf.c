// Vendored from https://github.com/tidwall/pogocache
//
// Copyright 2025 Polypoint Labs, LLC. All rights reserved.
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.
//
// Modified: replaced xrealloc/xfree with realloc/free, removed varint
// functions and util.h dependency.

#include "buf.h"

#include <stdlib.h>
#include <string.h>

void buf_ensure(struct buf *buf, size_t len) {
    if (buf->len + len > buf->cap) {
        size_t newcap = buf->cap;
        if (newcap == 0) {
            buf->data = 0;
            newcap = 16;
        } else {
            newcap *= 2;
        }
        while (buf->len + len > newcap) {
            newcap *= 2;
        }
        void *tmp = realloc(buf->data, newcap);
        if (!tmp) return;
        buf->data = tmp;
        buf->cap = newcap;
    }
}

void buf_append(struct buf *buf, const void *data, size_t len) {
    if (len == 0) return;
    buf_ensure(buf, len);
    memcpy(buf->data + buf->len, data, len);
    buf->len += len;
}

void buf_append_byte(struct buf *buf, char byte) {
    if (buf->len < buf->cap) {
        buf->data[buf->len++] = byte;
    } else {
        buf_append(buf, &byte, 1);
    }
}

void buf_clear(struct buf *buf) {
    if (buf->cap) {
        free(buf->data);
    }
    memset(buf, 0, sizeof(*buf));
}
