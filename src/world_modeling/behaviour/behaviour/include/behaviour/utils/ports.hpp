#ifndef BEHAVIOUR__UTILS__PORTS_HPP_
#define BEHAVIOUR__UTILS__PORTS_HPP_

#include <memory>
#include <optional>

#include <behaviortree_cpp/tree_node.h>

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