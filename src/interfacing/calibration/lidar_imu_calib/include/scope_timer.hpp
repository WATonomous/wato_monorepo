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

//
// Created by yunfan on 2021/3/19.
// Version: 1.0.0
//

#ifndef SCROPE_TIMER_HPP  // SRC_POLY_VISUAL_UTILS_HPP
#define SCROPE_TIMER_HPP

#include <chrono>
#include <cstdio>
#include <string>

#include "color.h"  // NOLINT(build/include_subdir)

using namespace std;  // NOLINT(build/namespaces)

class TimeConsuming
{
public:
  TimeConsuming();

  TimeConsuming(string msg, int repeat_time)
  {
    repeat_time_ = repeat_time;
    msg_ = msg;
    tc_start = std::chrono::high_resolution_clock::now();
    has_shown = false;
  }

  explicit TimeConsuming(string msg)
  {
    msg_ = msg;
    repeat_time_ = 1;
    tc_start = std::chrono::high_resolution_clock::now();
    has_shown = false;
  }

  ~TimeConsuming()
  {
    if (!has_shown && enable_) {
      tc_end = std::chrono::high_resolution_clock::now();
      double dt = std::chrono::duration_cast<std::chrono::duration<double>>(tc_end - tc_start).count();
      double t_us = static_cast<double>(dt) * 1e6 / repeat_time_;
      if (t_us < 1) {
        t_us *= 1000;
        printf("[TIMER] %s time consuming \033[32m %lf ns\033[0m\n", msg_.c_str(), t_us);
      } else if (t_us > 1e6) {
        t_us /= 1e6;
        printf("[TIMER] %s time consuming \033[32m %lf s\033[0m\n", msg_.c_str(), t_us);
      } else if (t_us > 1e3) {
        t_us /= 1e3;
        printf("[TIMER] %s time consuming \033[32m %lf ms\033[0m\n", msg_.c_str(), t_us);
      } else
        printf("[TIMER] %s time consuming \033[32m %lf us\033[0m\n", msg_.c_str(), t_us);
    }
  }

  void set_enbale(bool enable)
  {
    enable_ = enable;
  }

  void start()
  {
    tc_start = std::chrono::high_resolution_clock::now();
  }

  void stop()
  {
    if (!enable_) {
      return;
    }
    has_shown = true;
    tc_end = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::duration<double>>(tc_end - tc_start).count();
    //            ROS_WARN("%s time consuming %lf us.",msg_.c_str(),(double)(end_t - start_t).toNSec()/ 1e3);
    double t_us = static_cast<double>(dt) * 1e6 / repeat_time_;
    if (t_us < 1) {
      t_us *= 1000;
      printf(" -- [TIMER] %s time consuming \033[32m %lf ns\033[0m\n", msg_.c_str(), t_us);
    } else if (t_us > 1e6) {
      t_us /= 1e6;
      printf(" -- [TIMER] %s time consuming \033[32m %lf s\033[0m\n", msg_.c_str(), t_us);
    } else if (t_us > 1e3) {
      t_us /= 1e3;
      printf(" -- [TIMER] %s time consuming \033[32m %lf ms\033[0m\n", msg_.c_str(), t_us);
    } else
      printf(" -- [TIMER] %s time consuming \033[32m %lf us\033[0m\n", msg_.c_str(), t_us);
  }

private:
  std::chrono::high_resolution_clock::time_point tc_start, tc_end;
  string msg_;
  int repeat_time_;
  bool has_shown = false;
  bool enable_{true};
};

#endif  // SRC_POLY_VISUAL_UTILS_HPP
