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

#ifndef BEHAVIOUR__INTERFACES__INTERFACE_BASE_HPP_
#define BEHAVIOUR__INTERFACES__INTERFACE_BASE_HPP_

namespace behaviour::interfaces
{
/**
   * @brief Base interface for lifecycle-managed behaviour components.
   *
   * Derived classes should allocate resources in activate() and release them
   * in deactivate() to align with lifecycle transitions.
   */
class InterfaceBase
{
public:
  virtual ~InterfaceBase() = default;

  virtual void activate()
  {}

  virtual void deactivate()
  {}
};
}  // namespace behaviour::interfaces

#endif  // BEHAVIOUR__INTERFACES__INTERFACE_BASE_HPP_
