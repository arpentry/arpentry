// Vendored from https://github.com/tidwall/pogocache
//
// Copyright 2025 Polypoint Labs, LLC. All rights reserved.
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.
//
// Modified: removed xmalloc/util.h deps, removed varint functions.

#ifndef BUF_H
#define BUF_H

#include <stddef.h>
#include <stdint.h>

struct buf {
    char *data;
    size_t len;
    size_t cap;
};

void buf_ensure(struct buf *buf, size_t len);
void buf_append(struct buf *buf, const void *data, size_t len);
void buf_append_byte(struct buf *buf, char byte);
void buf_clear(struct buf *buf);

#endif
