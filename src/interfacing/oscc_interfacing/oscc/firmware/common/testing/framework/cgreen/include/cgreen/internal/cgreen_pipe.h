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

#ifndef CGREEN_PIPE_HEADER
#define CGREEN_PIPE_HEADER

#include <unistd.h>

#ifdef __cplusplus
namespace cgreen
{
extern "C"
{
#endif

  int cgreen_pipe_open(int pipes[2]);
  void cgreen_pipe_close(int p);
  ssize_t cgreen_pipe_read(int p, void * buf, size_t count);
  ssize_t cgreen_pipe_write(int p, const void * buf, size_t count);

#ifdef __cplusplus
}
}
#endif

#endif
