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

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <functional>
#include <string>

namespace eidos
{

/// Index of each bias in the bias vector of a relative-pose sensor.
enum RelativePoseBias : int
{
  kTranslationScale = 0,  ///< measured translation = (1 + s) * true translation
  kYawPerMetre = 1,  ///< extra yaw (rad) per metre travelled, about the vehicle's z axis
  kRotationScale = 2,  ///< measured rotation angle = (1 + r) * true angle
};

/**
 * @brief What a relative-pose sensor with these biases reports for a true relative pose (ROS-free).
 *
 * measured = Pose3(Rz(b_yaw * |t|) * Exp((1 + b_rot) * Log(R)), (1 + b_scale) * t)
 */
inline gtsam::Pose3 biasedRelativePose(const gtsam::Pose3 & relative, const gtsam::Vector3 & bias)
{
  const gtsam::Vector3 rotation = gtsam::Rot3::Logmap(relative.rotation()) * (1.0 + bias(kRotationScale));
  const double distance = relative.translation().norm();
  return gtsam::Pose3(
    gtsam::Rot3::Yaw(bias(kYawPerMetre) * distance) * gtsam::Rot3::Expmap(rotation),
    relative.translation() * (1.0 + bias(kTranslationScale)));
}

/// @brief The true relative pose a biased sensor's measurement stands for (inverse of the above).
inline gtsam::Pose3 unbiasedRelativePose(const gtsam::Pose3 & measured, const gtsam::Vector3 & bias)
{
  const gtsam::Point3 t = measured.translation() / (1.0 + bias(kTranslationScale));
  const gtsam::Rot3 r = gtsam::Rot3::Yaw(-bias(kYawPerMetre) * t.norm()) * measured.rotation();
  return gtsam::Pose3(gtsam::Rot3::Expmap(gtsam::Rot3::Logmap(r) / (1.0 + bias(kRotationScale))), t);
}

/**
 * @brief BetweenFactor<Pose3> for a sensor whose relative poses carry slowly varying biases.
 *
 * Joins two poses and a bias vector (translation scale, yaw per metre, rotation scale): the error
 * is the measured relative pose's local coordinates of biasedRelativePose(x1^-1 x2, bias), in
 * Pose3 tangent order [rot, trans] like BetweenFactor. The bias is estimated with the poses when
 * other sensors (IMU, GPS) make it observable. Jacobians are numerical.
 */
class BiasedRelativePoseFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::Vector3>
{
public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::Vector3>;

  BiasedRelativePoseFactor(
    gtsam::Key from, gtsam::Key to, gtsam::Key bias, const gtsam::Pose3 & measured, const gtsam::SharedNoiseModel & noise)
  : Base(noise, from, to, bias)
  , measured_(measured)
  {}

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & x1,
    const gtsam::Pose3 & x2,
    const gtsam::Vector3 & bias,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none,
    boost::optional<gtsam::Matrix &> H3 = boost::none) const override
  {
    const std::function<gtsam::Vector6(const gtsam::Pose3 &, const gtsam::Pose3 &, const gtsam::Vector3 &)> error =
      [this](const gtsam::Pose3 & a, const gtsam::Pose3 & b, const gtsam::Vector3 & c) { return residual(a, b, c); };
    if (H1) *H1 = gtsam::numericalDerivative31<gtsam::Vector6, gtsam::Pose3, gtsam::Pose3, gtsam::Vector3>(error, x1, x2, bias);
    if (H2) *H2 = gtsam::numericalDerivative32<gtsam::Vector6, gtsam::Pose3, gtsam::Pose3, gtsam::Vector3>(error, x1, x2, bias);
    if (H3) *H3 = gtsam::numericalDerivative33<gtsam::Vector6, gtsam::Pose3, gtsam::Pose3, gtsam::Vector3>(error, x1, x2, bias);
    return residual(x1, x2, bias);
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new BiasedRelativePoseFactor(*this)));
  }

  const gtsam::Pose3 & measured() const
  {
    return measured_;
  }

private:
  gtsam::Vector6 residual(const gtsam::Pose3 & x1, const gtsam::Pose3 & x2, const gtsam::Vector3 & bias) const
  {
    return measured_.localCoordinates(biasedRelativePose(x1.between(x2), bias));
  }

  gtsam::Pose3 measured_;
};

}  // namespace eidos
