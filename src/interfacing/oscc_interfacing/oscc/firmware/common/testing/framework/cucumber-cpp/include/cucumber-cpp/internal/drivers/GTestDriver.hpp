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

#ifndef CUKE_GTESTDRIVER_HPP_
#define CUKE_GTESTDRIVER_HPP_

#include <iostream>

#include "../step/StepManager.hpp"

namespace cucumber
{
namespace internal
{

class GTestStep : public BasicStep
{
protected:
  const InvokeResult invokeStepBody();

private:
  void initGTest();
  void initFlags();

protected:
  static bool initialized;
};

#define STEP_INHERITANCE(step_name) ::cucumber::internal::GTestStep

}  // namespace internal
}  // namespace cucumber

#endif /* CUKE_GTESTDRIVER_HPP_ */
