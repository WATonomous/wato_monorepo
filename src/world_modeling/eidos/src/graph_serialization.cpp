// Boost serialization exports for all GTSAM factor/value types used in Eidos.
// This must be a .cpp file — BOOST_CLASS_EXPORT expands to code that must
// appear in exactly one translation unit.

#include <gtsam/base/serialization.h>
#include <gtsam/base/GenericValue.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>

// Value types
GTSAM_VALUE_EXPORT(gtsam::Pose3);
GTSAM_VALUE_EXPORT(gtsam::Vector3);
GTSAM_VALUE_EXPORT(gtsam::imuBias::ConstantBias);

// Factor types
BOOST_CLASS_EXPORT(gtsam::BetweenFactor<gtsam::Pose3>);
BOOST_CLASS_EXPORT(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>);
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::Pose3>);
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::Vector3>);
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>);
BOOST_CLASS_EXPORT(gtsam::GPSFactor);
BOOST_CLASS_EXPORT(gtsam::ImuFactor);
BOOST_CLASS_EXPORT(gtsam::CombinedImuFactor);

// Noise model types (needed for factor deserialization)
BOOST_CLASS_EXPORT(gtsam::noiseModel::Diagonal);
BOOST_CLASS_EXPORT(gtsam::noiseModel::Isotropic);
BOOST_CLASS_EXPORT(gtsam::noiseModel::Constrained);
BOOST_CLASS_EXPORT(gtsam::noiseModel::Unit);
