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

#ifndef CUKE_CUKEENGINE_IMPL_HPP_
#define CUKE_CUKEENGINE_IMPL_HPP_

#include "CukeCommands.hpp"
#include "CukeEngine.hpp"

namespace cucumber
{
namespace internal
{

/**
 * Default Engine Implementation
 *
 * Currently it is a wrapper around CukeCommands. It will have its own
 * implementation when feature #31 is complete.
 */
class CukeEngineImpl : public CukeEngine
{
private:
  CukeCommands cukeCommands;

public:
  std::vector<StepMatch> stepMatches(const std::string & name) const;
  void beginScenario(const tags_type & tags);
  void invokeStep(const std::string & id, const invoke_args_type & args, const invoke_table_type & tableArg);
  void endScenario(const tags_type & tags);
  std::string snippetText(
    const std::string & keyword, const std::string & name, const std::string & multilineArgClass) const;
};

}  // namespace internal
}  // namespace cucumber

#endif /* CUKE_CUKEENGINE_IMPL_HPP_ */
