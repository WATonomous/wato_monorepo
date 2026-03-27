// Plugin registration for HolonomicEKF (separate from implementation to avoid
// duplicate symbols when holonomic_ekf.cpp is compiled into eidos_transform_core).

#include <pluginlib/class_list_macros.hpp>

#include "eidos_transform/holonomic_ekf.hpp"

PLUGINLIB_EXPORT_CLASS(eidos_transform::HolonomicEKF, eidos_transform::EKFModelPlugin)
