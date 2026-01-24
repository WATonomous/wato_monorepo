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

#ifndef ASSERTIONS_HEADER
#define ASSERTIONS_HEADER

#include <cgreen/constraint.h>
#include <cgreen/reporter.h>
#include <stdint.h>

#include "internal/assertions_internal.h"
#include "internal/stringify_token.h"

#ifndef __cplusplus
  #include <stdbool.h>
#endif

#ifdef __cplusplus
  #include <cgreen/cpp_assertions.h>

namespace cgreen
{
extern "C"
{
#endif

/*
  Modern style asserts using constraints:

   assert_that(actual, <constraint>(expected));
   assert_that(<expression>);

*/
#define assert_that(...) assert_that_NARG(__VA_ARGS__)(__VA_ARGS__)
#define assert_that_double(actual, constraint) \
  assert_that_double_(__FILE__, __LINE__, STRINGIFY_TOKEN(actual), (double)(actual), (constraint))

#define pass_test() assert_true(true)
#define fail_test(...) assert_true_with_message(false, __VA_ARGS__)

  /* Utility: */
  int get_significant_figures(void);
  void significant_figures_for_assert_double_are(int figures);

#include <cgreen/legacy.h>

#ifdef __cplusplus
}
}
#endif

#endif
