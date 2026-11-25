// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Minimal test harness for the host tests (plain C, no dependencies).

#ifndef CS_TEST_H_
#define CS_TEST_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int cs_test_failures = 0;
static const char * cs_test_current = "";

#define CHECK(cond)                                                                                  \
  do {                                                                                               \
    if (!(cond)) {                                                                                   \
      fprintf(stderr, "%s:%d: [%s] CHECK failed: %s\n", __FILE__, __LINE__, cs_test_current, #cond); \
      cs_test_failures++;                                                                            \
    }                                                                                                \
  } while (0)

#define CHECK_EQ_INT(a, b)                                                                                             \
  do {                                                                                                                 \
    long long va_ = (long long)(a);                                                                                    \
    long long vb_ = (long long)(b);                                                                                    \
    if (va_ != vb_) {                                                                                                  \
      fprintf(                                                                                                         \
        stderr, "%s:%d: [%s] %s == %s failed: %lld != %lld\n", __FILE__, __LINE__, cs_test_current, #a, #b, va_, vb_); \
      cs_test_failures++;                                                                                              \
    }                                                                                                                  \
  } while (0)

#define CHECK_EQ_STR(a, b)                                                                              \
  do {                                                                                                  \
    const char * sa_ = (a);                                                                             \
    const char * sb_ = (b);                                                                             \
    if (strcmp(sa_, sb_) != 0) {                                                                        \
      fprintf(stderr, "%s:%d: [%s] \"%s\" != \"%s\"\n", __FILE__, __LINE__, cs_test_current, sa_, sb_); \
      cs_test_failures++;                                                                               \
    }                                                                                                   \
  } while (0)

#define RUN(test)            \
  do {                       \
    cs_test_current = #test; \
    test();                  \
  } while (0)

#define TEST_MAIN_END()                                          \
  do {                                                           \
    if (cs_test_failures != 0) {                                 \
      fprintf(stderr, "%d check(s) failed\n", cs_test_failures); \
      return EXIT_FAILURE;                                       \
    }                                                            \
    printf("all checks passed\n");                               \
    return EXIT_SUCCESS;                                         \
  } while (0)

#endif  // CS_TEST_H_
