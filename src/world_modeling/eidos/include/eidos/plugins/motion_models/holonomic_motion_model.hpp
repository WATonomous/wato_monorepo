#pragma once

#include <vector>

#include "eidos/plugins/base_motion_model_plugin.hpp"

namespace eidos {

/**
 * @brief Simple holonomic motion model using BetweenFactor<Pose3>.
 *
 * Connects consecutive states with an identity-mean BetweenFactor whose
 * noise scales linearly with dt. Useful for testing the keygroup
 * architecture without requiring an IMU.
 */
class HolonomicMotionModel : public MotionModelPlugin {
public:
  HolonomicMotionModel() = default;
  ~HolonomicMotionModel() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;

  void generateMotionModel(
      gtsam::Key key_begin, double t_begin,
      gtsam::Key key_end, double t_end,
      gtsam::NonlinearFactorGraph& factors,
      gtsam::Values& values) override;

private:
  // Process noise diagonal [roll, pitch, yaw, x, y, z] variance per second
  std::vector<double> process_noise_diagonal_;
};

}  // namespace eidos
