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

#ifndef BEHAVIOUR__UTILS__PORTS_HPP_
#define BEHAVIOUR__UTILS__PORTS_HPP_

#include <behaviortree_cpp/tree_node.h>

#include <memory>
#include <optional>

namespace behaviour::ports
{
/** @brief Returns input port value as std::optional; nullopt if missing/invalid. */
template <typename T>
inline std::optional<T> tryGet(const BT::TreeNode & node, const char * port_name)
{
  auto res = node.getInput<T>(port_name);
  return res ? std::optional<T>(res.value()) : std::nullopt;
}

/** @brief Returns shared_ptr input port value; nullptr if missing/invalid/null. */
template <typename T>
inline std::shared_ptr<T> tryGetPtr(const BT::TreeNode & node, const char * port_name)
{
  auto res = node.getInput<std::shared_ptr<T>>(port_name);
  return res ? res.value() : nullptr;
}

}  // namespace behaviour::ports

#endif  // BEHAVIOUR__UTILS__PORTS_HPP_
