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
// Abbreviated OpenSSL 3.5 type declarations.
// When NOOPENSSL is defined, this header provides no types (they are unused).
#ifndef OPENSSL_H
#define OPENSSL_H

#ifndef NOOPENSSL

// Minimal forward declarations for OpenSSL types used by tls.c
typedef struct ssl_st SSL;
typedef struct ssl_ctx_st SSL_CTX;
typedef struct ssl_method_st SSL_METHOD;

#endif // !NOOPENSSL

#endif // OPENSSL_H
