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

#ifndef SUITE_HEADER
#define SUITE_HEADER

#include <cgreen/internal/function_macro.h>
#include <cgreen/reporter.h>
#include <cgreen/unit.h>

#include "internal/suite_internal.h"
#ifndef __cplusplus
  #include <stdbool.h>
#endif

#ifdef __cplusplus
namespace cgreen
{
extern "C"
{
#endif

#define create_test_suite() create_named_test_suite_(__func__, __FILE__, __LINE__)
#define create_named_test_suite(name) create_named_test_suite_(name, __FILE__, __LINE__)
#define add_test(suite, test) add_test_(suite, STRINGIFY_TOKEN(test), &spec_name(default, test))
#define add_test_with_context(suite, context, test) add_test_(suite, STRINGIFY_TOKEN(test), &spec_name(context, test))
#define add_tests(suite, ...) add_tests_(suite, #__VA_ARGS__, (CgreenTest *)__VA_ARGS__)
#define add_suite(owner, suite) add_suite_(owner, STRINGIFY_TOKEN(suite), suite)

  void set_setup(TestSuite * suite, void (*set_up)(void));
  void set_teardown(TestSuite * suite, void (*tear_down)(void));
  int count_tests(TestSuite * suite);
  bool has_test(TestSuite * suite, const char * name);
  bool has_setup(TestSuite * suite);
  bool has_teardown(TestSuite * suite);
  void destroy_test_suite(TestSuite * suite);

#ifdef __cplusplus
}
}
#endif

#endif
