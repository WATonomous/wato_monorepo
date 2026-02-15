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

#include <algorithm>
#include <cmath>

namespace pid_control_raw
{

/**
 * @brief Simple PID controller with integral clamping, output saturation,
 *        and conditional-integration anti-windup.
 */
class Pid
{
public:
  struct Gains
  {
    double p{0.0};
    double i{0.0};
    double d{0.0};
    double i_clamp_min{-1.0};
    double i_clamp_max{1.0};
    double u_clamp_min{-1.0};
    double u_clamp_max{1.0};
  };

  void set_gains(const Gains & gains) { gains_ = gains; }
  const Gains & get_gains() const { return gains_; }

  void reset()
  {
    integral_ = 0.0;
    prev_error_ = 0.0;
    first_call_ = true;
  }

  /**
   * @brief Compute PID output.
   * @param error  setpoint - measurement
   * @param dt     time step in seconds (must be > 0)
   * @return clamped command
   */
  double compute(double error, double dt)
  {
    if (dt <= 0.0) {
      return 0.0;
    }

    // Proportional
    double p_term = gains_.p * error;

    // Derivative (zero on first call to avoid spike)
    double d_term = 0.0;
    if (!first_call_) {
      d_term = gains_.d * (error - prev_error_) / dt;
    }
    prev_error_ = error;
    first_call_ = false;

    // Compute raw command before integral to decide anti-windup
    double command_no_i = p_term + d_term;

    // Conditional integration anti-windup:
    // Only integrate when output is not saturated, or when the error
    // would drive the output away from the saturation limit.
    double command_prev = command_no_i + gains_.i * integral_;
    double command_prev_sat = std::clamp(command_prev, gains_.u_clamp_min, gains_.u_clamp_max);
    bool saturated = (command_prev != command_prev_sat);

    if (!saturated || (error * command_prev <= 0.0)) {
      integral_ += error * dt;
      integral_ = std::clamp(integral_, gains_.i_clamp_min, gains_.i_clamp_max);
    }

    double i_term = gains_.i * integral_;

    // Final output with saturation
    double command = p_term + i_term + d_term;
    command = std::clamp(command, gains_.u_clamp_min, gains_.u_clamp_max);

    return command;
  }

private:
  Gains gains_;
  double integral_{0.0};
  double prev_error_{0.0};
  bool first_call_{true};
};

}  // namespace pid_control_raw
