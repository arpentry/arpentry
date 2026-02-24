// https://github.com/tidwall/pogocache
//
// Copyright 2025 Polypoint Labs, LLC. All rights reserved.
// This file is part of the Pogocache project.
// Use of this source code is governed by the MIT that can be found in
// the LICENSE file.
//
// For alternative licensing options or general questions, please contact
// us at licensing@polypointlabs.com.
#ifndef TLS_H
#define TLS_H

#include <stdbool.h>
#include <unistd.h>

struct tls;

bool tls_accept(int fd, struct tls **tls);
ssize_t tls_read(struct tls *tls, int fd, void *buf, size_t count);
ssize_t tls_write(struct tls *tls, int fd, const void *buf, size_t count);
void tls_close(struct tls *tls, int fd);

#endif
