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

#pragma once

#include <memory>

#include <pluginlib/class_loader.hpp>

#include "deep_core/plugin_interfaces/backend_memory_allocator.hpp"
#include "deep_core/plugin_interfaces/deep_backend_plugin.hpp"

namespace wato::test
{

/**
 * @brief Mock backend fixture for testing
 *
 * Catch2 test fixture that provides access to the mock backend plugin
 * and its allocator for consistent testing across all packages.
 */
class MockBackendFixture
{
public:
  /**
   * @brief Constructor - initializes mock backend plugin
   */
  MockBackendFixture()
  {
    loader_ =
      std::make_unique<pluginlib::ClassLoader<wato::DeepBackendPlugin>>("deep_core", "wato::DeepBackendPlugin");
    mock_backend_ = loader_->createUniqueInstance("mock_backend");
    allocator_ = mock_backend_->get_allocator();
  }

  /**
   * @brief Destructor - ensures clean shutdown
   */
  ~MockBackendFixture()
  {
    // Reset shared pointers before destroying the loader
    allocator_.reset();
    mock_backend_.reset();
    loader_.reset();
  }

  /**
   * @brief Get the mock backend allocator
   * @return Shared pointer to the mock backend memory allocator
   */
  std::shared_ptr<wato::BackendMemoryAllocator> getAllocator()
  {
    return allocator_;
  }

  /**
   * @brief Get the mock backend plugin
   * @return Shared pointer to the mock backend plugin
   */
  std::shared_ptr<wato::DeepBackendPlugin> getBackend()
  {
    return mock_backend_;
  }

protected:
  std::unique_ptr<pluginlib::ClassLoader<wato::DeepBackendPlugin>> loader_;
  std::shared_ptr<wato::DeepBackendPlugin> mock_backend_;
  std::shared_ptr<wato::BackendMemoryAllocator> allocator_;
};

}  // namespace wato::test
