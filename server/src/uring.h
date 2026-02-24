// https://github.com/tidwall/pogocache
//
// Copyright 2025 Polypoint Labs, LLC. All rights reserved.
// This file is part of the Pogocache project.
// Use of this source code is governed by the MIT that can be found in
// the LICENSE file.
//
// For alternative licensing options or general questions, please contact
// us at licensing@polypointlabs.com.
#ifndef URING_H
#define URING_H

#include <stdbool.h>

#if !defined(__linux__) && !defined(NOURING)
#define NOURING
#endif

#ifndef NOURING
#include <liburing.h>
#endif

bool uring_available(void);

#endif
