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

#ifndef CUKE_CUKECOMMANDS_HPP_
#define CUKE_CUKECOMMANDS_HPP_

#include <map>
#include <sstream>
#include <string>

#include <boost/shared_ptr.hpp>

#include "ContextManager.hpp"
#include "hook/HookRegistrar.hpp"
#include "Scenario.hpp"
#include "step/StepManager.hpp"
#include "Table.hpp"

namespace cucumber
{
namespace internal
{

using boost::shared_ptr;

/**
 * Legacy class to be removed when feature #31 is complete, substituted by CukeEngineImpl.
 */
class CukeCommands
{
public:
  CukeCommands();
  virtual ~CukeCommands();

  void beginScenario(const TagExpression::tag_list & tags = TagExpression::tag_list());
  void endScenario();
  const std::string snippetText(const std::string stepKeyword, const std::string stepName) const;
  MatchResult stepMatches(const std::string description) const;
  InvokeResult invoke(step_id_type id, const InvokeArgs * pArgs);

protected:
  const std::string escapeRegex(const std::string regex) const;
  const std::string escapeCString(const std::string str) const;

private:
  StepManager stepManager;
  HookRegistrar hookRegistrar;
  ContextManager contextManager;
  bool hasStarted;

private:
  static shared_ptr<Scenario> currentScenario;
};

}  // namespace internal
}  // namespace cucumber

#endif /* CUKE_CUKECOMMANDS_HPP_ */
