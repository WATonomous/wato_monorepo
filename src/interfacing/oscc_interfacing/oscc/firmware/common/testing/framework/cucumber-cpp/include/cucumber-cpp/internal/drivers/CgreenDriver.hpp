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

#ifndef CUKE_CGREENDRIVER_HPP_
#define CUKE_CGREENDRIVER_HPP_

#include <cgreen/cgreen.h>

#include "../step/StepManager.hpp"

using namespace cgreen;

namespace cucumber
{
namespace internal
{

class CukeCgreenInterceptor;

typedef int TextPrinter(const char * format, ...);

typedef struct
{
  TextPrinter * printer;
  int depth;
} TextMemo;

class CgreenStep : public BasicStep
{
protected:
  const InvokeResult invokeStepBody();
  static bool initialized;

private:
  void initCgreenTest();
  void setReporterPrinter(TestReporter *, TextPrinter *);
};

#define STEP_INHERITANCE(step_name) ::cucumber::internal::CgreenStep

}  // namespace internal
}  // namespace cucumber

#endif  // CUKE_CGREENDRIVER_HPP_
