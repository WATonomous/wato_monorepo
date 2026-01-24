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

namespace behaviour::ports
{
/**
   * @brief Get a required input port value.
   *
   * Throws BT::RuntimeError if the port is missing or invalid.
   *
   * @tparam T The type of the port value
   * @param node The BT node
   * @param port_name The name of the input port
   * @return The port value
   */
template <typename T>
inline T get(const BT::TreeNode & node, const char * port_name)
{
  auto res = node.getInput<T>(port_name);
  if (!res) {
    throw BT::RuntimeError("Missing/invalid input [", port_name, "]: ", res.error());
  }
  return res.value();
}

/**
   * @brief Get a required shared_ptr input port value.
   *
   * Throws BT::RuntimeError if the port is missing, invalid, or null.
   *
   * @tparam T The type pointed to by the shared_ptr
   * @param node The BT node
   * @param port_name The name of the input port
   * @return The non-null shared_ptr
   */
template <typename T>
inline std::shared_ptr<T> getPtr(const BT::TreeNode & node, const char * port_name)
{
  auto res = node.getInput<std::shared_ptr<T>>(port_name);
  if (!res) {
    throw BT::RuntimeError("Missing/invalid input [", port_name, "]: ", res.error());
  }
  auto ptr = res.value();
  if (!ptr) {
    throw BT::RuntimeError("Input [", port_name, "] is a null shared_ptr");
  }
  return ptr;
}

/**
   * @brief Try to get an optional input port value.
   *
   * Returns std::nullopt if the port is missing or invalid.
   *
   * @tparam T The type of the port value
   * @param node The BT node
   * @param port_name The name of the input port
   * @return The port value, or std::nullopt
   */
template <typename T>
inline std::optional<T> tryGet(const BT::TreeNode & node, const char * port_name)
{
  auto res = node.getInput<T>(port_name);
  if (!res) {
    return std::nullopt;
  }
  return res.value();
}

/**
   * @brief Try to get an optional shared_ptr input port value.
   *
   * Returns nullptr if the port is missing, invalid, or null.
   *
   * @tparam T The type pointed to by the shared_ptr
   * @param node The BT node
   * @param port_name The name of the input port
   * @return The shared_ptr (may be null)
   */
template <typename T>
inline std::shared_ptr<T> tryGetPtr(const BT::TreeNode & node, const char * port_name)
{
  auto res = node.getInput<std::shared_ptr<T>>(port_name);
  if (!res) {
    return nullptr;
  }
  return res.value();
}

}  // namespace behaviour::ports

#endif  // BEHAVIOUR__UTILS__PORTS_HPP_
