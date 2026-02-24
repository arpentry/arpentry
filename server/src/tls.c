// https://github.com/tidwall/pogocache
//
// Copyright 2025 Polypoint Labs, LLC. All rights reserved.
// This file is part of the Pogocache project.
// Use of this source code is governed by the MIT that can be found in
// the LICENSE file.
//
// For alternative licensing options or general questions, please contact
// us at licensing@polypointlabs.com.
//
// TLS wrapper. With NOOPENSSL, all functions are pass-through stubs.
#include "tls.h"
#include "openssl.h"

#include <stdlib.h>
#include <unistd.h>

#ifdef NOOPENSSL

struct tls { int dummy; };

bool tls_accept(int fd, struct tls **tls) {
    (void)fd;
    (void)tls;
    return false;
}

ssize_t tls_read(struct tls *tls, int fd, void *buf, size_t count) {
    if (tls) {
        // No TLS support compiled in
        return -1;
    }
    return read(fd, buf, count);
}

ssize_t tls_write(struct tls *tls, int fd, const void *buf, size_t count) {
    if (tls) {
        return -1;
    }
    return write(fd, buf, count);
}

void tls_close(struct tls *tls, int fd) {
    (void)tls;
    close(fd);
}

#else
// Full OpenSSL implementation would go here.
// Not needed for arpentry — always compiled with NOOPENSSL.
#error "OpenSSL TLS not implemented; compile with -DNOOPENSSL"
#endif
