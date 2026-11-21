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

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>
#include <utility>
#include <vector>

#include <catch2/catch_all.hpp>

#include "bevfusion/lidar_sweep_timing.hpp"

using wato::perception::bevfusion::cameraAzimuthDeg;
using wato::perception::bevfusion::sweepTargets;
using wato::perception::bevfusion::wrapDeg;

namespace
{

constexpr int64_t kStamp = 1'783'475'930'067'891'456;
constexpr double kPeriodNs = 50e6;  // 1200 RPM
constexpr size_t kStep = 16;  // x, y, z, intensity as float32

// Points in firing order: azimuth goes from start_deg by sweep_deg over the scan (negative = clockwise
// seen from above, like a Velodyne). Every nan_every-th point has no return.
std::vector<uint8_t> makeSweep(size_t n, double start_deg, double sweep_deg, size_t nan_every = 0)
{
  std::vector<uint8_t> buf(n * kStep, 0);
  for (size_t i = 0; i < n; ++i) {
    const double a = (start_deg + sweep_deg * static_cast<double>(i) / static_cast<double>(n - 1)) * M_PI / 180.0;
    float p[4] = {static_cast<float>(10.0 * std::cos(a)), static_cast<float>(10.0 * std::sin(a)), 0.5f, 7.0f};
    if (nan_every && i % nan_every == 3 % nan_every) {
      p[0] = p[1] = std::numeric_limits<float>::quiet_NaN();
    }
    std::memcpy(buf.data() + i * kStep, p, sizeof(p));
  }
  return buf;
}

// Independent reference: walk the synthetic points back from the end to the latest one that points at
// the camera, and use that point's time (points are evenly spaced in time over the sweep)
int64_t expectedTarget(double start_deg, double sweep_deg, double camera_deg, size_t n)
{
  const double sweep_ns = std::fabs(sweep_deg) / 360.0 * kPeriodNs;
  const double step_deg = std::fabs(sweep_deg) / static_cast<double>(n - 1);
  for (size_t i = n; i-- > 0;) {
    const double a = start_deg + sweep_deg * static_cast<double>(i) / static_cast<double>(n - 1);
    if (std::fabs(wrapDeg(a - camera_deg)) <= step_deg) {
      return kStamp -
             static_cast<int64_t>(std::llround(static_cast<double>(n - 1 - i) / static_cast<double>(n - 1) * sweep_ns));
    }
  }
  return std::numeric_limits<int64_t>::min();
}

const std::vector<double> kCameras = {-6.0, -51.0, -96.0, -141.0, 174.0, 129.0, 84.0, 39.0};

// A Velodyne-like cloud: columns of 32 lasers fired together, each laser with its own azimuth offset
// (so single steps between sampled points go either way), lasers 30 and 31 without returns (0, 0), and
// a sector of columns without any return
constexpr size_t kLasers = 32;

std::vector<uint8_t> makeVelodyneSweep(
  size_t columns, double start_deg, double sweep_deg, double gap_from_deg, double gap_deg)
{
  std::vector<uint8_t> buf(columns * kLasers * kStep, 0);
  for (size_t c = 0; c < columns; ++c) {
    const double travelled = sweep_deg * static_cast<double>(c) / static_cast<double>(columns - 1);
    const bool in_gap = std::fabs(travelled) >= gap_from_deg && std::fabs(travelled) < gap_from_deg + gap_deg;
    for (size_t j = 0; j < kLasers; ++j) {
      float p[4] = {0.0f, 0.0f, 0.0f, 0.0f};
      if (in_gap) {
        p[0] = p[1] = std::numeric_limits<float>::quiet_NaN();
      } else if (j < 30) {
        // Up to +-4 deg, as on a VLP-32C
        const double offset = 4.0 * std::sin(2.0 * M_PI * static_cast<double>(j) / static_cast<double>(kLasers));
        const double a = (start_deg + travelled + offset) * M_PI / 180.0;
        p[0] = static_cast<float>(20.0 * std::cos(a));
        p[1] = static_cast<float>(20.0 * std::sin(a));
      }
      std::memcpy(buf.data() + (c * kLasers + j) * kStep, p, sizeof(p));
    }
  }
  return buf;
}

// Reference for makeVelodyneSweep: the latest column whose firing azimuth points at the camera
int64_t expectedColumnTarget(double start_deg, double sweep_deg, double camera_deg, size_t columns)
{
  const double sweep_ns = std::fabs(sweep_deg) / 360.0 * kPeriodNs;
  const double step_deg = std::fabs(sweep_deg) / static_cast<double>(columns - 1);
  for (size_t c = columns; c-- > 0;) {
    const double a = start_deg + sweep_deg * static_cast<double>(c) / static_cast<double>(columns - 1);
    if (std::fabs(wrapDeg(a - camera_deg)) <= step_deg) {
      return kStamp - static_cast<int64_t>(std::llround(
                        static_cast<double>(columns - 1 - c) / static_cast<double>(columns - 1) * sweep_ns));
    }
  }
  return std::numeric_limits<int64_t>::min();
}

}  // namespace

TEST_CASE("sweepTargets: clockwise Velodyne sweep gives each camera the time the beam passed it", "[sweep][fast]")
{
  const double start = 100.0;
  const double sweep = -360.0;
  const auto cloud = makeSweep(36000, start, sweep);
  const auto t = sweepTargets(cloud.data(), 36000, kStep, 0, 4, kStamp, kCameras, kPeriodNs);
  REQUIRE(t.has_value());
  REQUIRE(t->size() == kCameras.size());
  for (size_t i = 0; i < kCameras.size(); ++i) {
    const int64_t expected = expectedTarget(start, sweep, kCameras[i], 36000);
    INFO("camera " << i << " at " << kCameras[i] << " deg");
    CHECK(std::llabs((*t)[i] - expected) < 100'000);  // 0.1 ms
    CHECK((*t)[i] <= kStamp);
    CHECK((*t)[i] >= kStamp - static_cast<int64_t>(kPeriodNs));
  }
}

TEST_CASE("sweepTargets: counter-clockwise rotation and a start angle across +-180 deg", "[sweep][fast]")
{
  const double start = 170.0;
  const double sweep = 360.0;
  const auto cloud = makeSweep(20000, start, sweep);
  const auto t = sweepTargets(cloud.data(), 20000, kStep, 0, 4, kStamp, kCameras, kPeriodNs);
  REQUIRE(t.has_value());
  for (size_t i = 0; i < kCameras.size(); ++i) {
    INFO("camera " << i);
    CHECK(std::llabs((*t)[i] - expectedTarget(start, sweep, kCameras[i], 20000)) < 100'000);
  }
}

TEST_CASE("sweepTargets: a sweep slightly over one turn and points without returns", "[sweep][fast]")
{
  const double start = -20.0;
  const double sweep = -361.4;  // as measured on the drive bags
  // -20.7 deg lies in the 1.4 deg swept twice; its latest pass is at the very end of the scan
  const std::vector<double> cams = {-6.0, 39.0, -20.7, 174.0};
  const auto cloud = makeSweep(21000, start, sweep, 7);
  const auto t = sweepTargets(cloud.data(), 21000, kStep, 0, 4, kStamp, cams, kPeriodNs);
  REQUIRE(t.has_value());
  for (size_t i = 0; i < cams.size(); ++i) {
    INFO("camera at " << cams[i] << " deg");
    CHECK(std::llabs((*t)[i] - expectedTarget(start, sweep, cams[i], 21000)) < 150'000);
  }
  CHECK(kStamp - (*t)[2] < 500'000);
}

TEST_CASE(
  "sweepTargets: Velodyne-like columns with per-laser offsets, missing returns and a blind sector", "[sweep][fast]")
{
  // 912 columns of 32 lasers, as recorded from lidar_cc; no returns for 60 deg starting 100 deg into the sweep
  const size_t columns = 912;
  const double start = 35.0;
  const double sweep = -360.0;
  const auto cloud = makeVelodyneSweep(columns, start, sweep, 100.0, 60.0);
  const auto t = sweepTargets(cloud.data(), columns * kLasers, kStep, 0, 4, kStamp, kCameras, kPeriodNs);
  REQUIRE(t.has_value());
  for (size_t i = 0; i < kCameras.size(); ++i) {
    INFO("camera at " << kCameras[i] << " deg");
    CHECK(std::llabs((*t)[i] - expectedColumnTarget(start, sweep, kCameras[i], columns)) < 200'000);  // 0.2 ms
  }
}

TEST_CASE("sweepTargets: clouds that are not one ordered sweep return no result", "[sweep][fast]")
{
  SECTION("points not in firing order")
  {
    const size_t n = 20000;
    const auto ordered = makeSweep(n, 0.0, -360.0);
    for (uint32_t seed = 1; seed <= 20; ++seed) {
      std::mt19937 rng(seed);
      auto cloud = ordered;
      for (size_t i = n - 1; i > 0; --i) {
        const size_t j = rng() % (i + 1);
        for (size_t b = 0; b < kStep; ++b) {
          std::swap(cloud[i * kStep + b], cloud[j * kStep + b]);
        }
      }
      INFO("seed " << seed);
      CHECK_FALSE(sweepTargets(cloud.data(), n, kStep, 0, 4, kStamp, kCameras, kPeriodNs).has_value());
    }
  }
  SECTION("half a sweep")
  {
    const auto cloud = makeSweep(10000, 0.0, -180.0);
    CHECK_FALSE(sweepTargets(cloud.data(), 10000, kStep, 0, 4, kStamp, kCameras, kPeriodNs).has_value());
  }
  SECTION("two sweeps back to back, like a merged cloud")
  {
    const auto cloud = makeSweep(20000, 0.0, -720.0);
    CHECK_FALSE(sweepTargets(cloud.data(), 20000, kStep, 0, 4, kStamp, kCameras, kPeriodNs).has_value());
  }
  SECTION("a full sweep followed by another LiDAR's points that turn out and back, like a merged cloud")
  {
    const auto main_sweep = makeSweep(20000, 0.0, -360.0);
    const auto out_and_back_1 = makeSweep(5000, 10.0, -170.0);
    const auto out_and_back_2 = makeSweep(5000, -160.0, 170.0);
    std::vector<uint8_t> cloud = main_sweep;
    cloud.insert(cloud.end(), out_and_back_1.begin(), out_and_back_1.end());
    cloud.insert(cloud.end(), out_and_back_2.begin(), out_and_back_2.end());
    const size_t n = cloud.size() / kStep;
    CHECK_FALSE(sweepTargets(cloud.data(), n, kStep, 0, 4, kStamp, kCameras, kPeriodNs).has_value());
  }
  SECTION("too few points")
  {
    const auto cloud = makeSweep(100, 0.0, -360.0);
    CHECK_FALSE(sweepTargets(cloud.data(), 100, kStep, 0, 4, kStamp, kCameras, kPeriodNs).has_value());
  }
  SECTION("all points without returns")
  {
    const auto cloud = makeSweep(1000, 0.0, -360.0, 1);
    CHECK_FALSE(sweepTargets(cloud.data(), 1000, kStep, 0, 4, kStamp, kCameras, kPeriodNs).has_value());
  }
  SECTION("bad layout")
  {
    const auto cloud = makeSweep(1000, 0.0, -360.0);
    CHECK_FALSE(sweepTargets(cloud.data(), 1000, 4, 0, 4, kStamp, kCameras, kPeriodNs).has_value());
    CHECK_FALSE(sweepTargets(nullptr, 1000, kStep, 0, 4, kStamp, kCameras, kPeriodNs).has_value());
    CHECK_FALSE(sweepTargets(cloud.data(), 1000, kStep, 0, 4, kStamp, kCameras, 0.0).has_value());
    CHECK_FALSE(sweepTargets(cloud.data(), 1000, kStep, 0, 4, kStamp, kCameras, std::nan("")).has_value());
    CHECK_FALSE(sweepTargets(cloud.data(), 1000, kStep, 0, 4, kStamp, kCameras, INFINITY).has_value());
  }
}

TEST_CASE("sweepTargets: a camera with an unknown direction gets the scan stamp", "[sweep][fast]")
{
  const auto cloud = makeSweep(20000, 0.0, -360.0);
  const auto t = sweepTargets(cloud.data(), 20000, kStep, 0, 4, kStamp, {-90.0, std::nan("")}, kPeriodNs);
  REQUIRE(t.has_value());
  CHECK((*t)[1] == kStamp);
  CHECK(std::llabs((*t)[0] - expectedTarget(0.0, -360.0, -90.0, 20000)) < 100'000);
}

TEST_CASE("sweepTargets: each acceptance rule on its own", "[sweep][fast]")
{
  // Clouds built from sweep segments in firing order, each rejected by exactly one rule
  const auto join = [](std::initializer_list<std::vector<uint8_t>> parts) {
    std::vector<uint8_t> cloud;
    for (const auto & part : parts) {
      cloud.insert(cloud.end(), part.begin(), part.end());
    }
    return cloud;
  };
  const auto accepted = [](const std::vector<uint8_t> & cloud) {
    return sweepTargets(cloud.data(), cloud.size() / kStep, kStep, 0, 4, kStamp, kCameras, kPeriodNs).has_value();
  };
  SECTION("a 160 deg blind sector (step rule), otherwise one clockwise turn")
  {
    CHECK_FALSE(accepted(join({makeSweep(5000, 0.0, -100.0), makeSweep(5000, -260.0, -100.0)})));
    CHECK(accepted(join({makeSweep(5000, 0.0, -100.0), makeSweep(5000, -160.0, -200.0)})));  // 60 deg sector: fine
  }
  SECTION("net rotation outside 300 to 420 deg")
  {
    CHECK_FALSE(accepted(makeSweep(10000, 0.0, -250.0)));
    CHECK_FALSE(accepted(makeSweep(10000, 0.0, -540.0)));
    CHECK(accepted(makeSweep(10000, 0.0, -310.0)));
    CHECK(accepted(makeSweep(10000, 0.0, -410.0)));
  }
  SECTION("turning back 40 deg (fall-back rule) within a net turn of 400 deg")
  {
    const auto cloud =
      join({makeSweep(5000, 0.0, -200.0), makeSweep(1000, -200.0, 40.0), makeSweep(6000, -160.0, -240.0)});
    CHECK_FALSE(accepted(cloud));
    // Turning back 10 deg is per-laser noise and is accepted
    CHECK(
      accepted(join({makeSweep(5000, 0.0, -200.0), makeSweep(300, -200.0, 10.0), makeSweep(6000, -190.0, -170.0)})));
  }
}

TEST_CASE("cameraAzimuthDeg: azimuth of the optical axis in the LiDAR frame", "[sweep][fast]")
{
  for (const double deg : {0.0, 90.0, -135.0, 174.0}) {
    const double a = deg * M_PI / 180.0;
    // Optical frame z axis -> (cos a, sin a, 0) in the LiDAR frame; x and y complete a right-handed frame
    const float m[16] = {
      static_cast<float>(std::sin(a)),
      0.0f,
      static_cast<float>(std::cos(a)),
      1.0f,
      static_cast<float>(-std::cos(a)),
      0.0f,
      static_cast<float>(std::sin(a)),
      2.0f,
      0.0f,
      -1.0f,
      0.0f,
      3.0f,
      0.0f,
      0.0f,
      0.0f,
      1.0f};
    CHECK(cameraAzimuthDeg(m) == Catch::Approx(deg).margin(1e-4));
  }
}

TEST_CASE("wrapDeg: wraps into (-180, 180]", "[sweep][fast]")
{
  CHECK(wrapDeg(0.0) == Catch::Approx(0.0));
  CHECK(wrapDeg(180.0) == Catch::Approx(180.0));
  CHECK(wrapDeg(-180.0) == Catch::Approx(180.0));
  CHECK(wrapDeg(190.0) == Catch::Approx(-170.0));
  CHECK(wrapDeg(-370.0) == Catch::Approx(-10.0));
  CHECK(wrapDeg(725.0) == Catch::Approx(5.0));
}
