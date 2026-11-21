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

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include <catch2/catch_all.hpp>

#include "bevfusion/camera_matcher.hpp"

using wato::perception::bevfusion::CameraMatcher;
using wato::perception::bevfusion::CameraMatcherConfig;

namespace
{

constexpr int64_t kMs = 1'000'000;
using Matcher = CameraMatcher<int64_t, int>;
using Sets = std::vector<Matcher::MatchedSet>;

CameraMatcherConfig config(size_t cameras, int64_t max_wait_ms = 60, int64_t tolerance_ms = 450, double early = 0.5)
{
  CameraMatcherConfig c;
  c.num_cameras = cameras;
  c.max_wait_ns = max_wait_ms < 0 ? -1 : max_wait_ms * kMs;
  c.tolerance_ns = tolerance_ms * kMs;
  c.early_fraction = early;
  return c;
}

// Feeds frames of one camera at a fixed 50 ms period, from first_ms on, after 20 earlier warm-up frames so
// the matcher has learned the interval (the early rule needs 20 intervals).
constexpr int64_t kWarmUpFrames = 20;

void feedHistory(Matcher & m, size_t camera, int64_t first_ms, int64_t count, Sets & out)
{
  for (int64_t k = -kWarmUpFrames; k < count; ++k) {
    const int64_t stamp = (first_ms + 50 * k) * kMs;
    m.addFrame(camera, stamp, stamp, stamp + 30 * kMs, out);
  }
}

}  // namespace

TEST_CASE("CameraMatcher: early rule decides on the frame before the target without waiting", "[matcher][fast]")
{
  Matcher m(config(1));
  Sets out;
  feedHistory(m, 0, 900, 3, out);  // frames at 900, 950, 1000 ms
  REQUIRE(out.empty());
  // Target 1005 ms: the frame at 1000 ms is 5 ms before it, within half the 50 ms interval
  m.addScan(1005 * kMs, 7, {}, 1007 * kMs, out);
  REQUIRE(out.size() == 1);
  CHECK(out[0].frames[0].stamp_ns == 1000 * kMs);
  CHECK(out[0].decided_ns == 1007 * kMs);
  CHECK(out[0].scan == 7);
  CHECK(out[0].max_gap_ns == 5 * kMs);
}

TEST_CASE(
  "CameraMatcher: waits for the next frame when the previous one is more than half a period away", "[matcher][fast]")
{
  Matcher m(config(1));
  Sets out;
  feedHistory(m, 0, 900, 3, out);  // newest frame 1000 ms
  m.addScan(1030 * kMs, 0, {}, 1031 * kMs, out);  // 30 ms after the newest frame
  REQUIRE(out.empty());
  REQUIRE(m.pendingScans() == 1);
  m.addFrame(0, 1050 * kMs, 1050 * kMs, 1080 * kMs, out);
  REQUIRE(out.size() == 1);
  CHECK(out[0].frames[0].stamp_ns == 1050 * kMs);  // 20 ms away beats 30 ms
  CHECK(out[0].decided_ns == 1080 * kMs);
}

TEST_CASE("CameraMatcher: keeps the previous frame when it is nearer than the next", "[matcher][fast]")
{
  Matcher m(config(1));
  Sets out;
  feedHistory(m, 0, 900, 3, out);
  // 26 ms after 1000: not early (threshold 20 ms: half of 950 + 2 x 0.9 x 50 = 1040 ms, less 1000); deadline 1087 ms
  m.addScan(1026 * kMs, 0, {}, 1027 * kMs, out);
  REQUIRE(out.empty());
  m.addFrame(0, 1060 * kMs, 1060 * kMs, 1080 * kMs, out);  // next is 34 ms away, arrives before the deadline
  REQUIRE(out.size() == 1);
  CHECK(out[0].frames[0].stamp_ns == 1000 * kMs);
  CHECK(out[0].decided_ns == 1080 * kMs);  // decided by the next frame, not the deadline
}

TEST_CASE("CameraMatcher: at the wait cap the best received frame is used", "[matcher][fast]")
{
  Matcher m(config(1, 60));
  Sets out;
  feedHistory(m, 0, 900, 3, out);
  m.addScan(1040 * kMs, 0, {}, 1041 * kMs, out);
  REQUIRE(out.empty());
  const auto deadline = m.nextDeadline();
  REQUIRE(deadline.has_value());
  CHECK(*deadline == 1101 * kMs);
  m.tick(1100 * kMs, out);
  CHECK(out.empty());
  m.tick(*deadline, out);
  REQUIRE(out.size() == 1);
  CHECK(out[0].frames[0].stamp_ns == 1000 * kMs);
  CHECK(out[0].decided_ns == 1101 * kMs);
  CHECK_FALSE(m.nextDeadline().has_value());
}

TEST_CASE("CameraMatcher: a frame outside the tolerance at the cap drops the scan", "[matcher][fast]")
{
  Matcher m(config(1, 60, 30));
  Sets out;
  feedHistory(m, 0, 900, 3, out);
  m.addScan(1040 * kMs, 0, {}, 1041 * kMs, out);
  m.tick(1101 * kMs, out);
  CHECK(out.empty());
  CHECK(m.counters().dropped_out_of_tolerance == 1);
  CHECK(m.counters().dropped_no_frame == 0);
  CHECK(m.pendingScans() == 0);
}

TEST_CASE("CameraMatcher: a camera with no frame before its target at the cap drops the scan", "[matcher][fast]")
{
  Matcher m(config(2, 60));
  Sets out;
  feedHistory(m, 0, 900, 3, out);  // camera 1 never sends a frame
  m.addScan(1040 * kMs, 0, {}, 1041 * kMs, out);
  m.tick(1101 * kMs, out);
  CHECK(out.empty());
  CHECK(m.counters().dropped_no_frame == 1);
  CHECK(m.pendingScans() == 0);
}

TEST_CASE("CameraMatcher: a certain pick outside the tolerance drops the scan", "[matcher][fast]")
{
  Matcher m(config(1, 60, 50));
  Sets out;
  m.addFrame(0, 900 * kMs, 1, 930 * kMs, out);
  m.addFrame(0, 1100 * kMs, 2, 1130 * kMs, out);  // frames 200 ms apart (dropped frames)
  m.addScan(1000 * kMs, 0, {}, 1131 * kMs, out);  // both 100 ms away, beyond the 50 ms tolerance
  CHECK(out.empty());
  CHECK(m.counters().dropped_out_of_tolerance == 1);
}

TEST_CASE("CameraMatcher: each camera is matched to its own target", "[matcher][fast]")
{
  Matcher m(config(2));
  Sets out;
  feedHistory(m, 0, 800, 6, out);  // 800 ... 1050
  feedHistory(m, 1, 800, 6, out);
  // Camera 0 targets 960 ms, camera 1 targets 1010 ms (e.g. swept 50 ms apart)
  m.addScan(1010 * kMs, 0, {960 * kMs, 1010 * kMs}, 1100 * kMs, out);
  REQUIRE(out.size() == 1);
  CHECK(out[0].frames[0].stamp_ns == 950 * kMs);
  CHECK(out[0].frames[1].stamp_ns == 1000 * kMs);
  CHECK(out[0].targets_ns == std::vector<int64_t>{960 * kMs, 1010 * kMs});
}

TEST_CASE("CameraMatcher: a target list of the wrong size falls back to the scan stamp", "[matcher][fast]")
{
  Matcher m(config(2));
  Sets out;
  feedHistory(m, 0, 900, 3, out);
  feedHistory(m, 1, 900, 3, out);
  m.addScan(1000 * kMs, 0, {123}, 1031 * kMs, out);
  REQUIRE(out.size() == 1);
  CHECK(out[0].targets_ns == std::vector<int64_t>{1000 * kMs, 1000 * kMs});
}

TEST_CASE("CameraMatcher: triggered cameras with identical stamps match exactly", "[matcher][fast]")
{
  Matcher m(config(4));
  Sets out;
  for (size_t c = 0; c < 4; ++c) {
    feedHistory(m, c, 900, 3, out);
  }
  m.addScan(1000 * kMs, 0, {}, 1002 * kMs, out);
  REQUIRE(out.size() == 1);
  CHECK(out[0].max_gap_ns == 0);
  for (const auto & f : out[0].frames) {
    CHECK(f.stamp_ns == 1000 * kMs);
  }
}

namespace
{

struct Event
{
  int64_t at;
  int kind;  // 0 frame, 1 scan
  size_t camera;
  int64_t stamp;
};

// Free-running cameras with random phases, 25-45 ms delivery delay, stamp jitter of +-jitter_us and a
// dropped frame now and then (never in a camera's first 5 frames, so its frame interval is learned first)
std::vector<Event> makeEvents(uint32_t seed, size_t cams, int64_t jitter_us)
{
  std::mt19937 rng(seed);
  const auto uniform = [&rng](int64_t lo, int64_t hi) { return lo + static_cast<int64_t>(rng() % (hi - lo + 1)); };
  std::vector<Event> events;
  for (size_t c = 0; c < cams; ++c) {
    const int64_t phase = uniform(0, 49);
    for (int64_t k = 0; k < 400; ++k) {
      const int64_t jitter = jitter_us ? uniform(-jitter_us, jitter_us) * 1000 : 0;
      if (k >= 5 && uniform(0, 30) == 0) {
        continue;
      }
      const int64_t stamp = (1000 + phase + 50 * k) * kMs + jitter;
      events.push_back({stamp + uniform(25, 45) * kMs, 0, c, stamp});
    }
  }
  const int64_t lidar_phase = uniform(0, 49);
  for (int64_t k = 2; k < 395; ++k) {
    const int64_t stamp = (1000 + lidar_phase + 50 * k) * kMs;
    events.push_back({stamp + 2 * kMs, 1, 0, stamp});
  }
  std::stable_sort(events.begin(), events.end(), [](const Event & a, const Event & b) {
    return a.at != b.at ? a.at < b.at : a.kind < b.kind;
  });
  return events;
}

// Runs the events through a matcher, ticking at every deadline as a node's timer would
std::map<int64_t, Matcher::MatchedSet> runEvents(const std::vector<Event> & events, const CameraMatcherConfig & cfg)
{
  Matcher m(cfg);
  Sets all;
  for (const auto & e : events) {
    while (true) {
      const auto d = m.nextDeadline();
      if (!d || *d > e.at) {
        break;
      }
      m.tick(*d, all);
    }
    if (e.kind == 0) {
      m.addFrame(e.camera, e.stamp, e.stamp, e.at, all);
    } else {
      m.addScan(e.stamp, 0, {}, e.at, all);
    }
  }
  REQUIRE(m.counters().dropped_overflow == 0);
  std::map<int64_t, Matcher::MatchedSet> by_scan;
  for (auto & set : all) {
    by_scan.emplace(set.scan_stamp_ns, std::move(set));
  }
  return by_scan;
}

}  // namespace

TEST_CASE("CameraMatcher: the early rule changes when a choice is made, never the choice", "[matcher][fast]")
{
  constexpr size_t kCams = 6;
  // With fixed phases the same camera is the last to be decided in every scan of a seed, so some seeds
  // gain nothing; the time saved is checked over all of them
  int64_t total_saved = 0;
  // Stamp jitter of +-1 ms keeps every interval within 10% of the shortest one, so the rule must be exact
  for (const int64_t jitter_us : {0, 1000}) {
    for (uint32_t seed = 1; seed <= 20; ++seed) {
      INFO("seed " << seed << " jitter_us " << jitter_us);
      const auto events = makeEvents(seed, kCams, jitter_us);
      auto cfg = config(kCams, -1, 450, 0.0);
      cfg.max_pending_scans = 64;  // the waiting run must never drop a scan for overflow
      const auto waited = runEvents(events, cfg);
      cfg.early_fraction = 0.5;
      const auto early = runEvents(events, cfg);
      REQUIRE(waited.size() > 300);
      REQUIRE(early.size() == waited.size());
      for (const auto & [stamp, w] : waited) {
        const auto it = early.find(stamp);
        REQUIRE(it != early.end());
        for (size_t c = 0; c < kCams; ++c) {
          REQUIRE(it->second.frames[c].stamp_ns == w.frames[c].stamp_ns);
        }
        REQUIRE(it->second.decided_ns <= w.decided_ns);
        total_saved += w.decided_ns - it->second.decided_ns;
      }
    }
  }
  CHECK(total_saved > 0);
}

TEST_CASE("CameraMatcher: no early decision on a frame stamped late", "[matcher][fast]")
{
  Matcher m(config(1));
  Sets out;
  feedHistory(m, 0, 1900, 2, out);  // ... 1900, 1950
  m.addFrame(0, 2021 * kMs, 0, 2051 * kMs, out);  // stamped 21 ms late; the next frame keeps the 50 ms grid
  // 17 ms after 2021, within half of 0.9 x 50 ms; but the next frame (2050) can come only 29 ms later
  m.addScan(2038 * kMs, 0, {}, 2052 * kMs, out);
  REQUIRE(out.empty());
  m.addFrame(0, 2050 * kMs, 0, 2080 * kMs, out);
  REQUIRE(out.size() == 1);
  CHECK(out[0].frames[0].stamp_ns == 2050 * kMs);  // 12 ms away beats 17 ms
}

TEST_CASE("CameraMatcher: no early decision before 20 intervals are known", "[matcher][fast]")
{
  Matcher m(config(1));
  Sets out;
  // Two dropped frames in a row right after start: intervals of 100 and 100 ms confirm each other
  m.addFrame(0, 1000 * kMs, 0, 1030 * kMs, out);
  m.addFrame(0, 1100 * kMs, 0, 1130 * kMs, out);
  m.addFrame(0, 1200 * kMs, 0, 1230 * kMs, out);
  m.addScan(1240 * kMs, 0, {}, 1241 * kMs, out);  // 40 ms after 1200: early if 100 ms were the frame interval
  REQUIRE(out.empty());
  m.addFrame(0, 1250 * kMs, 0, 1280 * kMs, out);
  REQUIRE(out.size() == 1);
  CHECK(out[0].frames[0].stamp_ns == 1250 * kMs);
}

TEST_CASE("CameraMatcher: a reset forgets the learned frame interval", "[matcher][fast]")
{
  Matcher m(config(1));
  Sets out;
  for (int64_t k = 0; k < 25; ++k) {  // 10 Hz: 100 ms intervals
    m.addFrame(0, (100'000 + 100 * k) * kMs, 0, (100'030 + 100 * k) * kMs, out);
  }
  m.addFrame(0, 5'000 * kMs, 0, 5'030 * kMs, out);  // bag restarted
  REQUIRE(m.counters().resets == 1);
  m.addScan(5'040 * kMs, 0, {}, 5'041 * kMs, out);  // 40 ms later: early only with the old 100 ms interval
  CHECK(out.empty());
  CHECK(m.pendingScans() == 1);
}

TEST_CASE("CameraMatcher: the learned interval follows a camera that slows down", "[matcher][fast]")
{
  Matcher m(config(1, -1));
  Sets out;
  int64_t t = 0;
  for (int64_t k = 0; k < 25; ++k, t += 50) {
    m.addFrame(0, t * kMs, 0, (t + 30) * kMs, out);
  }
  for (int64_t k = 0; k < 205; ++k) {  // more than the 200-interval window at 100 ms
    t += 100;
    m.addFrame(0, t * kMs, 0, (t + 30) * kMs, out);
  }
  // 35 ms after the newest frame: within half of min(0.9 x 100, 2 x 0.9 x 100 - 100) = 80 ms
  m.addScan((t + 35) * kMs, 0, {}, (t + 36) * kMs, out);
  REQUIRE(out.size() == 1);
  CHECK(out[0].frames[0].stamp_ns == t * kMs);
}

TEST_CASE("CameraMatcher: no early decision on an interval that spans a dropped frame", "[matcher][fast]")
{
  Matcher m(config(1));
  Sets out;
  // The second frame (1050) was dropped: the only interval seen is 100 ms, which is not yet confirmed
  m.addFrame(0, 1000 * kMs, 0, 1030 * kMs, out);
  m.addFrame(0, 1100 * kMs, 0, 1130 * kMs, out);
  m.addScan(1140 * kMs, 0, {}, 1141 * kMs, out);  // 40 ms after 1100: early with a 100 ms interval
  REQUIRE(out.empty());
  m.addFrame(0, 1150 * kMs, 0, 1180 * kMs, out);
  REQUIRE(out.size() == 1);
  CHECK(out[0].frames[0].stamp_ns == 1150 * kMs);  // 10 ms away beats 40 ms
}

TEST_CASE("CameraMatcher: the early rule allows for stamp jitter", "[matcher][fast]")
{
  Matcher m(config(1));
  Sets out;
  feedHistory(m, 0, 1900, 3, out);  // 1900, 1950, 2000: shortest interval 50 ms
  // 24.5 ms after 2000: within half of 50 ms, but the next frame comes 2 ms early (48 ms interval)
  m.addScan(2024 * kMs + 500'000, 0, {}, 2026 * kMs, out);
  REQUIRE(out.empty());
  m.addFrame(0, 2048 * kMs, 0, 2078 * kMs, out);
  REQUIRE(out.size() == 1);
  CHECK(out[0].frames[0].stamp_ns == 2048 * kMs);  // 23.5 ms away beats 24.5 ms
}

TEST_CASE("CameraMatcher: the early threshold is inclusive and ties pick the earlier frame", "[matcher][fast]")
{
  SECTION("exactly at the threshold")
  {
    Matcher m(config(1));
    Sets out;
    feedHistory(m, 0, 900, 3, out);
    // Next frame no earlier than min(1000 + 45, 950 + 2 x 45) = 1040 ms: threshold half of 40 ms
    m.addScan(1020 * kMs, 0, {}, 1023 * kMs, out);
    REQUIRE(out.size() == 1);
    CHECK(out[0].frames[0].stamp_ns == 1000 * kMs);
  }
  SECTION("equal distance to both frames")
  {
    Matcher m(config(1));
    Sets out;
    feedHistory(m, 0, 900, 3, out);
    m.addScan(1025 * kMs, 0, {}, 1026 * kMs, out);
    REQUIRE(out.empty());
    m.addFrame(0, 1050 * kMs, 0, 1080 * kMs, out);
    REQUIRE(out.size() == 1);
    CHECK(out[0].frames[0].stamp_ns == 1000 * kMs);
  }
}

TEST_CASE("CameraMatcher: a camera's choice is final once made", "[matcher][fast]")
{
  CameraMatcherConfig c = config(2, -1);
  c.frames_per_camera = 2;
  Matcher m(c);
  Sets out;
  for (size_t cam = 0; cam < 2; ++cam) {
    feedHistory(m, cam, 900, 3, out);
  }
  m.addScan(1030 * kMs, 0, {}, 1031 * kMs, out);  // 30 ms after 1000: both cameras wait
  m.addFrame(0, 1050 * kMs, 0, 1080 * kMs, out);  // camera 0 decided: 1050 (20 ms) beats 1000
  // Camera 1 stalls while camera 0 keeps going; 1050 is evicted from camera 0's two kept frames
  m.addFrame(0, 1100 * kMs, 0, 1130 * kMs, out);
  m.addFrame(0, 1150 * kMs, 0, 1180 * kMs, out);
  REQUIRE(out.empty());
  m.addFrame(1, 1050 * kMs, 0, 1200 * kMs, out);
  REQUIRE(out.size() == 1);
  CHECK(out[0].frames[0].stamp_ns == 1050 * kMs);
  CHECK(out[0].frames[1].stamp_ns == 1050 * kMs);
}

TEST_CASE("CameraMatcher: picks the nearest frame per camera", "[matcher][fast]")
{
  Matcher m(config(3, -1));
  Sets out;
  const std::vector<int64_t> phases = {0, 12, 37};
  std::vector<std::vector<int64_t>> stamps(3);
  for (size_t c = 0; c < 3; ++c) {
    for (int k = 0; k < 6; ++k) {
      const int64_t s = (800 + phases[c] + 50 * k) * kMs;
      stamps[c].push_back(s);
      m.addFrame(c, s, s, s + 30 * kMs, out);
    }
  }
  const int64_t target = 1003 * kMs;
  m.addScan(target, 0, {}, 2000 * kMs, out);
  REQUIRE(out.size() == 1);
  for (size_t c = 0; c < 3; ++c) {
    const auto nearest = *std::min_element(stamps[c].begin(), stamps[c].end(), [&](int64_t a, int64_t b) {
      return std::llabs(a - target) < std::llabs(b - target);
    });
    CHECK(out[0].frames[c].stamp_ns == nearest);
  }
}

TEST_CASE("CameraMatcher: unlimited wait still has a one second safety limit", "[matcher][fast]")
{
  Matcher m(config(1, -1));
  Sets out;
  feedHistory(m, 0, 900, 3, out);
  m.addScan(1040 * kMs, 0, {}, 1041 * kMs, out);
  const auto d = m.nextDeadline();
  REQUIRE(d.has_value());
  CHECK(*d == 2041 * kMs);
  m.tick(*d, out);
  REQUIRE(out.size() == 1);
  CHECK(out[0].frames[0].stamp_ns == 1000 * kMs);
}

TEST_CASE("CameraMatcher: too many waiting scans decides the oldest at once", "[matcher][fast]")
{
  SECTION("without a usable frame it is dropped")
  {
    CameraMatcherConfig c = config(1);
    c.max_pending_scans = 2;
    Matcher m(c);
    Sets out;
    for (int k = 0; k < 3; ++k) {
      m.addScan((1000 + 50 * k) * kMs, k, {}, (1001 + 50 * k) * kMs, out);
    }
    CHECK(out.empty());
    CHECK(m.pendingScans() == 2);
    CHECK(m.counters().dropped_overflow == 1);
    CHECK(m.counters().scans == 3);
  }
  SECTION("with a usable frame it is matched, even if its deadline was missed")
  {
    CameraMatcherConfig c = config(1);
    c.max_pending_scans = 1;
    Matcher m(c);
    Sets out;
    feedHistory(m, 0, 900, 3, out);
    m.addScan(1040 * kMs, 1, {}, 1041 * kMs, out);  // deadline 1101 ms; the timer is late
    m.addScan(1090 * kMs, 2, {}, 1150 * kMs, out);
    REQUIRE(out.size() == 1);
    CHECK(out[0].scan == 1);
    CHECK(out[0].frames[0].stamp_ns == 1000 * kMs);
    CHECK(m.counters().dropped_overflow == 0);
    CHECK(m.pendingScans() == 1);
  }
}

TEST_CASE("CameraMatcher: nextDeadline is the earliest deadline of the waiting scans", "[matcher][fast]")
{
  Matcher m(config(1));
  Sets out;
  m.addScan(1000 * kMs, 0, {}, 1001 * kMs, out);
  m.addScan(1050 * kMs, 0, {}, 1051 * kMs, out);
  REQUIRE(m.nextDeadline().has_value());
  CHECK(*m.nextDeadline() == 1061 * kMs);  // also when the caller is already past it
  m.tick(1061 * kMs, out);
  REQUIRE(m.nextDeadline().has_value());
  CHECK(*m.nextDeadline() == 1111 * kMs);
}

TEST_CASE("CameraMatcher: a backward time jump clears all state", "[matcher][fast]")
{
  Matcher m(config(2));
  Sets out;
  feedHistory(m, 0, 100'000, 3, out);
  feedHistory(m, 1, 100'000, 3, out);
  m.addScan(100'200 * kMs, 0, {}, 100'201 * kMs, out);
  out.clear();
  m.addFrame(0, 5'000 * kMs, 0, 5'030 * kMs, out);  // bag restarted
  CHECK(m.counters().resets == 1);
  CHECK(m.framesHeld(0) == 1);
  CHECK(m.framesHeld(1) == 0);
  CHECK(m.pendingScans() == 0);
  m.addScan(90'000 * kMs, 0, {}, 90'001 * kMs, out);  // the scan stream does not jump again after the reset
  CHECK(m.counters().resets == 1);
}

TEST_CASE("CameraMatcher: a backward jump of the scan stream clears all state", "[matcher][fast]")
{
  CameraMatcher<int64_t, std::shared_ptr<int>> m(config(1));
  std::vector<CameraMatcher<int64_t, std::shared_ptr<int>>::MatchedSet> out;
  auto scan = std::make_shared<int>(1);
  m.addFrame(0, 1000 * kMs, 0, 1030 * kMs, out);
  m.addScan(2000 * kMs, scan, {}, 2001 * kMs, out);  // waits (no frame after its target)
  CHECK(scan.use_count() == 2);
  m.addScan(1500 * kMs, std::make_shared<int>(2), {}, 2051 * kMs, out);  // 500 ms back
  CHECK(m.counters().resets == 1);
  CHECK(m.framesHeld(0) == 0);
  CHECK(scan.use_count() == 1);
}

TEST_CASE("CameraMatcher: a small backward step is not a reset", "[matcher][fast]")
{
  Matcher m(config(1));
  Sets out;
  m.addFrame(0, 1000 * kMs, 0, 1030 * kMs, out);
  m.addFrame(0, 900 * kMs, 0, 1040 * kMs, out);  // 100 ms back: a late frame, kept in stamp order
  CHECK(m.counters().resets == 0);
  CHECK(m.framesHeld(0) == 2);
}

TEST_CASE("CameraMatcher: evicted and cleared frames release their payloads", "[matcher][fast]")
{
  CameraMatcherConfig c = config(1);
  c.frames_per_camera = 3;
  CameraMatcher<std::shared_ptr<int>, int> m(c);
  std::vector<CameraMatcher<std::shared_ptr<int>, int>::MatchedSet> out;
  std::vector<std::shared_ptr<int>> buffers;
  for (int k = 0; k < 5; ++k) {
    buffers.push_back(std::make_shared<int>(k));
    m.addFrame(0, (1000 + 50 * k) * kMs, buffers.back(), (1030 + 50 * k) * kMs, out);
  }
  CHECK(m.framesHeld(0) == 3);
  CHECK(buffers[0].use_count() == 1);
  CHECK(buffers[1].use_count() == 1);
  CHECK(buffers[4].use_count() == 2);
  m.clear();
  CHECK(buffers[4].use_count() == 1);
}

TEST_CASE("CameraMatcher: dropped and cleared scans release their payloads", "[matcher][fast]")
{
  CameraMatcherConfig c = config(1);
  c.max_pending_scans = 1;
  CameraMatcher<int64_t, std::shared_ptr<int>> m(c);
  std::vector<CameraMatcher<int64_t, std::shared_ptr<int>>::MatchedSet> out;
  auto first = std::make_shared<int>(1);
  auto second = std::make_shared<int>(2);
  m.addScan(1000 * kMs, first, {}, 1001 * kMs, out);
  m.addScan(1050 * kMs, second, {}, 1051 * kMs, out);  // overflow: the first is dropped (no frames)
  CHECK(first.use_count() == 1);
  CHECK(second.use_count() == 2);
  m.clear();
  CHECK(second.use_count() == 1);
}

TEST_CASE("CameraMatcher: a move-only scan payload is moved into the set", "[matcher][fast]")
{
  CameraMatcher<int64_t, std::unique_ptr<int>> m(config(1));
  std::vector<CameraMatcher<int64_t, std::unique_ptr<int>>::MatchedSet> out;
  for (int64_t k = -kWarmUpFrames; k < 3; ++k) {
    m.addFrame(0, (900 + 50 * k) * kMs, 0, (930 + 50 * k) * kMs, out);
  }
  m.addScan(1000 * kMs, std::make_unique<int>(5), {}, 1031 * kMs, out);
  REQUIRE(out.size() == 1);
  REQUIRE(out[0].scan);
  CHECK(*out[0].scan == 5);
}

TEST_CASE("CameraMatcher: out-of-order, duplicate and unknown-camera frames are handled", "[matcher][fast]")
{
  Matcher m(config(1, -1));
  Sets out;
  m.addFrame(0, 1000 * kMs, 1, 1030 * kMs, out);
  m.addFrame(0, 1100 * kMs, 3, 1130 * kMs, out);
  m.addFrame(0, 1050 * kMs, 2, 1140 * kMs, out);  // late arrival, inserted in stamp order
  m.addFrame(0, 1050 * kMs, 9, 1150 * kMs, out);  // duplicate stamp, ignored
  m.addFrame(5, 1050 * kMs, 9, 1150 * kMs, out);  // no such camera, ignored
  CHECK(m.framesHeld(0) == 3);
  m.addScan(1060 * kMs, 0, {}, 1200 * kMs, out);
  REQUIRE(out.size() == 1);
  CHECK(out[0].frames[0].data == 2);
}
