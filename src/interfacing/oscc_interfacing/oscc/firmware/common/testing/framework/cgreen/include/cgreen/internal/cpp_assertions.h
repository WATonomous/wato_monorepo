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

#ifndef INTERNAL_CPP_ASSERTIONS_HEADER
#define INTERNAL_CPP_ASSERTIONS_HEADER

#include <cgreen/constraint.h>
#include <stdint.h>

#include <string>
#include <typeinfo>

#include "stringify_token.h"

namespace cgreen
{

#define assert_that_constraint(actual, constraint) \
  assert_that_(__FILE__, __LINE__, STRINGIFY_TOKEN(actual), actual, constraint)

void assert_that_(
  const char * file, int line, const char * actual_string, const std::string & actual, Constraint * constraint);
void assert_that_(
  const char * file, int line, const char * actual_string, const std::string * actual, Constraint * constraint);
void assert_that_(const char * file, int line, const char * actual_string, double actual, Constraint * constraint);

// this isn't declared in assertions.h because you can't have overloads for an extern "C"-declared function, so it seems
void assert_that_(const char * file, int line, const char * actual_string, intptr_t actual, Constraint * constraint);

template <typename T>
void assert_that_(const char * file, int line, const char * actual_string, T actual, Constraint * constraint)
{
  if (
    typeid(actual) == typeid(std::string &) || typeid(actual) == typeid(const std::string &) ||
    typeid(actual) == typeid(const std::string *) || typeid(actual) == typeid(std::string *))
  {
    assert_that_(file, line, actual_string, reinterpret_cast<const std::string *>(actual), constraint);

  } else if (typeid(actual) == typeid(std::string)) {
    assert_that_(file, line, actual_string, (const std::string *)&actual, constraint);

  } else {
    // TODO: update actual_string with output from operator<< of (T)actual
    assert_that_(file, line, actual_string, (intptr_t)actual, constraint);
  }
}
}  // namespace cgreen

#endif
