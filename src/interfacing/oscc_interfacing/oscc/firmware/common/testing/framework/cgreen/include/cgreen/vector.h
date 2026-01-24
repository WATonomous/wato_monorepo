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

#ifndef VECTOR_HEADER
#define VECTOR_HEADER

#ifdef __cplusplus
namespace cgreen
{
extern "C"
{
#endif

  typedef void (*GenericDestructor)(void *);
  typedef struct CgreenVector_ CgreenVector;

  CgreenVector * create_cgreen_vector(GenericDestructor destructor);
  void destroy_cgreen_vector(CgreenVector * vector);
  void cgreen_vector_add(CgreenVector * vector, void * item);
  void * cgreen_vector_remove(CgreenVector * vector, int position);
  void * cgreen_vector_get(const CgreenVector * vector, int position);
  int cgreen_vector_size(const CgreenVector * vector);

#ifdef __cplusplus
}
}
#endif

#endif
