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

#ifndef BREADCRUMB_HEADER
#define BREADCRUMB_HEADER

struct CgreenBreadcrumb_
{
  const char ** trail;
  int depth;
  int space;
};

typedef struct CgreenBreadcrumb_ CgreenBreadcrumb;

#ifdef __cplusplus
namespace cgreen
{
extern "C"
{
#endif

  CgreenBreadcrumb * create_breadcrumb(void);
  void destroy_breadcrumb(CgreenBreadcrumb * breadcrumb);
  void push_breadcrumb(CgreenBreadcrumb * breadcrumb, const char * name);
  void pop_breadcrumb(CgreenBreadcrumb * breadcrumb);
  const char * get_current_from_breadcrumb(CgreenBreadcrumb * breadcrumb);
  int get_breadcrumb_depth(CgreenBreadcrumb * breadcrumb);
  void walk_breadcrumb(CgreenBreadcrumb * breadcrumb, void (*walker)(const char *, void *), void * memo);

#ifdef __cplusplus
}
}
#endif

#endif
