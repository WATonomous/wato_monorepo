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

#include <cstdint>

#include <catch2/catch_test_macros.hpp>

#include "camera_trigger/sequence_tracker.hpp"

using camera_trigger::SequenceResult;
using camera_trigger::SequenceTracker;
using camera_trigger::SequenceVerdict;

TEST_CASE("First pulse and consecutive pulses", "[sequence]")
{
  SequenceTracker tracker;
  CHECK_FALSE(tracker.hasLast());
  CHECK(tracker.update(100).verdict == SequenceVerdict::kFirst);
  CHECK(tracker.update(101).verdict == SequenceVerdict::kNext);
  CHECK(tracker.update(102).verdict == SequenceVerdict::kNext);
  CHECK(tracker.last() == 102);
}

TEST_CASE("Gaps are counted, not filled", "[sequence]")
{
  SequenceTracker tracker;
  tracker.update(10);
  const SequenceResult gap = tracker.update(14);
  CHECK(gap.verdict == SequenceVerdict::kGap);
  CHECK(gap.missing == 3);
  CHECK(gap.accepted());
  CHECK(tracker.last() == 14);
  CHECK(tracker.update(15).verdict == SequenceVerdict::kNext);
}

TEST_CASE("Duplicates are dropped", "[sequence]")
{
  SequenceTracker tracker;
  tracker.update(5);
  const SequenceResult same = tracker.update(5);
  CHECK(same.verdict == SequenceVerdict::kDuplicate);
  CHECK_FALSE(same.accepted());

  tracker.update(6);
  tracker.update(7);
  CHECK(tracker.update(6).verdict == SequenceVerdict::kDuplicate);  // older, already accepted
  CHECK(tracker.last() == 7);
}

TEST_CASE("Late (reordered) pulses are dropped and do not move the sequence back", "[sequence]")
{
  SequenceTracker tracker;
  tracker.update(20);
  CHECK(tracker.update(22).verdict == SequenceVerdict::kGap);  // 21 missing so far
  const SequenceResult late = tracker.update(21);
  CHECK(late.verdict == SequenceVerdict::kLate);
  CHECK_FALSE(late.accepted());
  CHECK(tracker.last() == 22);
  CHECK(tracker.update(23).verdict == SequenceVerdict::kNext);
}

TEST_CASE("A large backward jump means the board rebooted", "[sequence]")
{
  SequenceTracker tracker;
  tracker.update(5000);
  const SequenceResult reboot = tracker.update(0);
  CHECK(reboot.verdict == SequenceVerdict::kReboot);
  CHECK(reboot.accepted());
  CHECK(tracker.last() == 0);
  CHECK(tracker.update(1).verdict == SequenceVerdict::kNext);
}

TEST_CASE("Reboot threshold boundary", "[sequence]")
{
  SequenceTracker tracker(10);
  tracker.update(100);
  CHECK(tracker.update(90).verdict == SequenceVerdict::kLate);  // 10 behind: within threshold
  CHECK(tracker.update(89).verdict == SequenceVerdict::kReboot);  // 11 behind: reboot
}

TEST_CASE("Sequence numbers wrap at 2^32", "[sequence]")
{
  SequenceTracker tracker;
  tracker.update(0xFFFFFFFEU);
  CHECK(tracker.update(0xFFFFFFFFU).verdict == SequenceVerdict::kNext);
  CHECK(tracker.update(0).verdict == SequenceVerdict::kNext);

  SequenceTracker gap_tracker;
  gap_tracker.update(0xFFFFFFFEU);
  const SequenceResult gap = gap_tracker.update(1);
  CHECK(gap.verdict == SequenceVerdict::kGap);
  CHECK(gap.missing == 2);

  SequenceTracker late_tracker;
  late_tracker.update(2);
  CHECK(late_tracker.update(0xFFFFFFFFU).verdict == SequenceVerdict::kLate);  // 3 behind, across the wrap
}

TEST_CASE("Reset forgets history", "[sequence]")
{
  SequenceTracker tracker;
  tracker.update(42);
  tracker.reset();
  CHECK_FALSE(tracker.hasLast());
  CHECK(tracker.update(0).verdict == SequenceVerdict::kFirst);
  CHECK(tracker.update(42).verdict == SequenceVerdict::kGap);  // 42 is no longer remembered
}
