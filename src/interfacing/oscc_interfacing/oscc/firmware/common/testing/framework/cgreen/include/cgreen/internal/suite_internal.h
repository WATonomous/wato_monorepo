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

#ifndef SUITE_INTERNAL_HEADER
#define SUITE_INTERNAL_HEADER

#include "cgreen/unit.h"

enum
{
  test_function,
  test_suite
};

typedef struct TestSuite_ TestSuite;

typedef struct
{
  int type;
  const char * name;

  union
  {
    CgreenTest * test;
    TestSuite * suite;
  } Runnable;
} UnitTest;

struct TestSuite_
{
  const char * name;
  const char * filename;
  int line;
  UnitTest * tests;
  void (*setup)(void);
  void (*teardown)(void);
  int size;
};

#ifdef __cplusplus
namespace cgreen
{
extern "C"
{
#endif

  void do_nothing(void);

  TestSuite * create_named_test_suite_(const char * name, const char * filename, int line);
  void add_test_(TestSuite * suite, const char * name, CgreenTest * test);
  void add_tests_(TestSuite * suite, const char * names, ...);
  void add_suite_(TestSuite * owner, const char * name, TestSuite * suite);

#ifdef __cplusplus
}
}
#endif

#endif
