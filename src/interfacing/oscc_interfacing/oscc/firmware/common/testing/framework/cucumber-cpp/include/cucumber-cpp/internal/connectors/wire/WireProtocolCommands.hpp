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

#ifndef CUKE_WIREPROTOCOL_COMMANDS_HPP_
#define CUKE_WIREPROTOCOL_COMMANDS_HPP_

#include <boost/shared_ptr.hpp>

#include "WireProtocol.hpp"

namespace cucumber
{
namespace internal
{

class ScenarioCommand : public WireCommand
{
protected:
  const CukeEngine::tags_type tags;

  ScenarioCommand(const CukeEngine::tags_type & tags);
};

class BeginScenarioCommand : public ScenarioCommand
{
public:
  BeginScenarioCommand(const CukeEngine::tags_type & tags);

  boost::shared_ptr<WireResponse> run(CukeEngine & engine) const;
};

class EndScenarioCommand : public ScenarioCommand
{
public:
  EndScenarioCommand(const CukeEngine::tags_type & tags);

  boost::shared_ptr<WireResponse> run(CukeEngine & engine) const;
};

class StepMatchesCommand : public WireCommand
{
private:
  const std::string stepName;

public:
  StepMatchesCommand(const std::string & stepName);

  boost::shared_ptr<WireResponse> run(CukeEngine & engine) const;
};

class InvokeCommand : public WireCommand
{
private:
  const std::string stepId;
  const CukeEngine::invoke_args_type args;
  const CukeEngine::invoke_table_type tableArg;

public:
  InvokeCommand(
    const std::string & stepId,
    const CukeEngine::invoke_args_type & args,
    const CukeEngine::invoke_table_type & tableArg);

  boost::shared_ptr<WireResponse> run(CukeEngine & engine) const;
};

class SnippetTextCommand : public WireCommand
{
private:
  std::string keyword, name, multilineArgClass;

public:
  SnippetTextCommand(const std::string & keyword, const std::string & name, const std::string & multilineArgClass);

  boost::shared_ptr<WireResponse> run(CukeEngine & engine) const;
};

class FailingCommand : public WireCommand
{
public:
  boost::shared_ptr<WireResponse> run(CukeEngine & engine) const;
};

}  // namespace internal
}  // namespace cucumber

#endif /* CUKE_WIREPROTOCOL_COMMANDS_HPP_ */
