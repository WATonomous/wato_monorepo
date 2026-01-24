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

#ifndef CGREEN_CPP_ASSERTIONS_H
#define CGREEN_CPP_ASSERTIONS_H

#include "internal/stringify_token.h"

#define assert_throws(exceptionType, expr)                    \
  try {                                                       \
    expr;                                                     \
    fail_test("Expected [" STRINGIFY_TOKEN(                   \
      expr) "] "                                              \
            "to throw [" STRINGIFY_TOKEN(exceptionType) "]"); \
  } catch (const exceptionType & ex) {                        \
    pass_test();                                              \
  } catch (const exceptionType * ex) {                        \
    pass_test();                                              \
  }

#endif
