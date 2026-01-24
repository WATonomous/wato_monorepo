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

#ifndef CUKE_CONTEXTMANAGER_HPP_
#define CUKE_CONTEXTMANAGER_HPP_

#include <vector>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

namespace cucumber
{

using boost::shared_ptr;
using boost::weak_ptr;

namespace internal
{

typedef std::vector<shared_ptr<void> > contexts_type;

class ContextManager
{
public:
  void purgeContexts();
  template <class T>
  weak_ptr<T> addContext();

protected:
  static contexts_type contexts;
};

template <class T>
weak_ptr<T> ContextManager::addContext()
{
  shared_ptr<T> shared(boost::make_shared<T>());
  contexts.push_back(shared);
  return weak_ptr<T>(shared);
}

}  // namespace internal

template <class T>
class ScenarioScope
{
public:
  ScenarioScope();

  T & operator*();
  T * operator->();
  T * get();

private:
  internal::ContextManager contextManager;
  shared_ptr<T> context;
  static weak_ptr<T> contextReference;
};

template <class T>
weak_ptr<T> ScenarioScope<T>::contextReference;

template <class T>
ScenarioScope<T>::ScenarioScope()
{
  if (contextReference.expired()) {
    contextReference = contextManager.addContext<T>();
  }
  context = contextReference.lock();
}

template <class T>
T & ScenarioScope<T>::operator*()
{
  return *(context.get());
}

template <class T>
T * ScenarioScope<T>::operator->()
{
  return (context.get());
}

template <class T>
T * ScenarioScope<T>::get()
{
  return context.get();
}

}  // namespace cucumber

#endif /* CUKE_CONTEXTMANAGER_HPP_ */
