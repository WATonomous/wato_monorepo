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

#ifndef C_ASSERTIONS_HEADER
#define C_ASSERTIONS_HEADER

#include <cgreen/constraint.h>
#include <inttypes.h>

#include "stringify_token.h"

#ifndef __cplusplus
  #define assert_that_constraint(actual, constraint) \
    assert_core_(__FILE__, __LINE__, STRINGIFY_TOKEN(actual), (intptr_t)(actual), (constraint))
#endif

#ifdef __cplusplus
extern "C"
{
#endif

  void assert_core_(const char * file, int line, const char * actual_string, intptr_t actual, Constraint * constraint);
  void assert_that_double_(
    const char * file, int line, const char * expression, double actual, Constraint * constraint);

#ifdef __cplusplus
}
#endif

#endif
