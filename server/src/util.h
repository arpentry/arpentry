// Arpentry shim: only mix13() is needed by net.c (line 358, connection hashmap).
// The full pogocache util.c/h is not vendored.
#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

// https://zimbry.blogspot.com/2011/09/better-bit-mixing-improving-on.html
static inline uint64_t mix13(uint64_t key) {
    key ^= (key >> 30);
    key *= UINT64_C(0xbf58476d1ce4e5b9);
    key ^= (key >> 27);
    key *= UINT64_C(0x94d049bb133111eb);
    key ^= (key >> 31);
    return key;
}

#endif
