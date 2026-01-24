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

#ifndef REPORTER_HEADER
#define REPORTER_HEADER

#include <cgreen/breadcrumb.h>
#include <stdarg.h>
#include <stdint.h>

typedef struct TestReporter_ TestReporter;

struct TestReporter_
{
  void (*destroy)(TestReporter * reporter);
  void (*start_suite)(TestReporter * reporter, const char * name, const int count);
  void (*start_test)(TestReporter * reporter, const char * name);
  void (*show_pass)(TestReporter * reporter, const char * file, int line, const char * message, va_list arguments);
  void (*show_skip)(TestReporter * reporter, const char * file, int line);
  void (*show_fail)(TestReporter * reporter, const char * file, int line, const char * message, va_list arguments);
  void (*show_incomplete)(
    TestReporter * reporter, const char * file, int line, const char * message, va_list arguments);
  void (*assert_true)(TestReporter * reporter, const char * file, int line, int result, const char * message, ...);
  void (*finish_test)(TestReporter * reporter, const char * file, int line, const char * message);
  void (*finish_suite)(TestReporter * reporter, const char * file, int line);
  int passes;
  int failures;
  int exceptions;
  int skips;
  uint32_t duration;
  int total_passes;
  int total_failures;
  int total_exceptions;
  int total_skips;
  uint32_t total_duration;
  CgreenBreadcrumb * breadcrumb;
  int ipc;
  void * memo;
  void * options;
};

typedef void TestReportMemo;

#ifdef __cplusplus
namespace cgreen
{
extern "C"
{
#endif

  TestReporter * create_reporter(void);
  TestReporter * get_test_reporter(void);
  void set_reporter_options(TestReporter * reporter, void * options);
  void setup_reporting(TestReporter * reporter);
  void destroy_reporter(TestReporter * reporter);
  void destroy_memo(TestReportMemo * memo);
  void reporter_start_test(TestReporter * reporter, const char * name);
  void reporter_start_suite(TestReporter * reporter, const char * name, const int count);
  void reporter_finish_test(TestReporter * reporter, const char * filename, int line, const char * message);
  void reporter_finish_suite(TestReporter * reporter, const char * filename, int line);
  void add_reporter_result(TestReporter * reporter, int result);
  void send_reporter_exception_notification(TestReporter * reporter);
  void send_reporter_skipped_notification(TestReporter * reporter);
  void send_reporter_completion_notification(TestReporter * reporter);

#ifdef __cplusplus
}
}
#endif

#endif
