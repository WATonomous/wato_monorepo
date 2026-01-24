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

#ifndef CUKE_SCENARIO_HPP_
#define CUKE_SCENARIO_HPP_

#include "hook/Tag.hpp"

namespace cucumber
{
namespace internal
{

class Scenario
{
public:
  Scenario(const TagExpression::tag_list & tags = TagExpression::tag_list());

  const TagExpression::tag_list & getTags();

private:
  const TagExpression::tag_list tags;
};

}  // namespace internal
}  // namespace cucumber

#endif /* CUKE_SCENARIO_HPP_ */
