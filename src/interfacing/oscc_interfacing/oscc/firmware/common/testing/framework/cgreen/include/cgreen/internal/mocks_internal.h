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

#ifndef MOCKS_INTERNAL_HEADER
#define MOCKS_INTERNAL_HEADER

#include <cgreen/constraint.h>
#include <cgreen/internal/function_macro.h>
#include <cgreen/internal/mock_table.h>
#include <cgreen/reporter.h>
#include <cgreen/vector.h>
#include <stdint.h>

#ifdef __cplusplus
namespace cgreen
{
extern "C"
{
#endif

  void expect_(TestReporter * test_reporter, const char * function, const char * test_file, int test_line, ...);
  void always_expect_(TestReporter * test_reporter, const char * function, const char * test_file, int test_line, ...);
  void never_expect_(TestReporter * test_reporter, const char * function, const char * test_file, int test_line, ...);
  intptr_t mock_(
    TestReporter * test_reporter,
    const char * function,
    const char * mock_file,
    int mock_line,
    const char * parameters,
    ...);
  Constraint * when_(const char * parameter, Constraint * constraint);

  void clear_mocks(void);
  void tally_mocks(TestReporter * reporter);

#ifdef __cplusplus
}
}
#endif

#endif
