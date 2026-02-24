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
// Arpentry shim: static inline wrappers around stdlib, matching the original
// xmalloc.h API so vendored pogocache sources compile without modification.
#ifndef XMALLOC_H
#define XMALLOC_H

#include <stddef.h>
#include <stdlib.h>

static inline void xmalloc_init(int nthreads) { (void)nthreads; }
static inline size_t xallocs(void) { return 0; }
static inline void *xmalloc(size_t size) { return malloc(size); }
static inline void *xrealloc(void *ptr, size_t size) { return realloc(ptr, size); }
static inline void xfree(void *ptr) { free(ptr); }
static inline void xpurge(void) {}
static inline size_t xrss(void) { return 0; }

#endif
