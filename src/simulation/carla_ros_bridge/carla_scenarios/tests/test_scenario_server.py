# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Tests for scenario server node."""

import contextlib
import hashlib
import json
import unittest
from pathlib import Path
from unittest.mock import patch

from carla_scenarios.scenario_server_node import load_map_provenance


class TestMapProvenance(unittest.TestCase):
    """Verify existing records without creating or changing any map files."""

    def setUp(self):
        self.map_path = Path("/maps/xodr/oval.xodr")
        self.osm_path = Path("/maps/osm/oval.osm")
        self.metadata_path = Path(str(self.map_path) + ".metadata.json")
        self.mapping_path = self.map_path.with_suffix(".mapping.json")
        self.files = {self.map_path: b"converted map", self.osm_path: b"source map"}
        self.record = {
            "osm_sha256": hashlib.sha256(self.files[self.osm_path]).hexdigest(),
            "xodr_sha256": hashlib.sha256(self.files[self.map_path]).hexdigest(),
        }
        self.files[self.mapping_path] = json.dumps(self.record).encode()

    def load(self, *, required=True):
        with contextlib.ExitStack() as stack:
            stack.enter_context(
                patch.object(
                    Path,
                    "is_file",
                    autospec=True,
                    side_effect=lambda p: p in self.files,
                )
            )
            stack.enter_context(
                patch.object(
                    Path,
                    "read_bytes",
                    autospec=True,
                    side_effect=lambda p: self.files[p],
                )
            )
            stack.enter_context(
                patch.object(
                    Path,
                    "read_text",
                    autospec=True,
                    side_effect=lambda p: self.files[p].decode(),
                )
            )
            return load_map_provenance(self.map_path, required=required)

    def test_existing_converter_record_verifies_both_maps(self):
        provenance = self.load()

        self.assertEqual(provenance["record_type"], "converter_mapping")
        self.assertEqual(provenance["source_sha256"], self.record["osm_sha256"])
        self.assertEqual(provenance["xodr_sha256"], self.record["xodr_sha256"])
        self.assertEqual(provenance["record_path"], str(self.mapping_path))
        self.assertEqual(
            provenance["record_sha256"],
            hashlib.sha256(self.files[self.mapping_path]).hexdigest(),
        )
        self.assertNotIn("converter", provenance)
        self.assertNotIn("cache_key", provenance)

    def test_wrapper_metadata_takes_precedence(self):
        metadata = {
            "source_sha256": self.record["osm_sha256"],
            "xodr_sha256": self.record["xodr_sha256"],
            "converter": "recorded revision",
        }
        self.files[self.metadata_path] = json.dumps(metadata).encode()
        self.files[self.mapping_path] = b"invalid legacy record"

        self.assertEqual(self.load(), metadata)

    def test_bad_wrapper_metadata_does_not_fall_back_to_mapping(self):
        self.files[self.metadata_path] = json.dumps({"xodr_sha256": "wrong"}).encode()

        with self.assertRaisesRegex(ValueError, "Converted map digest mismatch"):
            self.load()

    def test_changed_map_or_source_is_rejected(self):
        for path, message in (
            (self.map_path, "Converted map digest mismatch"),
            (self.osm_path, "Lanelet2/OpenDRIVE map versions differ"),
        ):
            with self.subTest(path=path):
                original = self.files[path]
                self.files[path] = b"changed content"
                with self.assertRaisesRegex(ValueError, message):
                    self.load()
                self.files[path] = original

    def test_missing_source_is_rejected(self):
        del self.files[self.osm_path]

        with self.assertRaisesRegex(ValueError, "Missing Lanelet2 source map"):
            self.load()

    def test_missing_record_is_optional_only_when_not_required(self):
        del self.files[self.mapping_path]

        self.assertIsNone(self.load(required=False))
        with self.assertRaisesRegex(ValueError, "Missing converted-map provenance"):
            self.load()

    def test_record_without_hashes_is_rejected(self):
        self.files[self.mapping_path] = b"{}"

        with self.assertRaisesRegex(ValueError, "Converted map digest mismatch"):
            self.load()
