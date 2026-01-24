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

#ifndef RUNNER_PLATFORM_HEADER
#define RUNNER_PLATFORM_HEADER

#include <cgreen/reporter.h>
#include <cgreen/suite.h>

#ifdef __cplusplus
namespace cgreen
{
extern "C"
{
#endif

  void run_specified_test_if_child(TestSuite * suite, TestReporter * reporter);
  void run_test_in_its_own_process(TestSuite * suite, CgreenTest * test, TestReporter * reporter);
  void die(const char * message, ...);
  void run_the_test_code(TestSuite * suite, CgreenTest * spec, TestReporter * reporter);

#ifdef __cplusplus
}
}
#endif

#endif
