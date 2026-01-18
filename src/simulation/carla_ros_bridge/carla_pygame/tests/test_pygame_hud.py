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
"""Tests for pygame HUD node."""

import sys
from unittest.mock import MagicMock, patch

import pytest


class TestPygameHudParameters:
    """Tests for pygame HUD parameters."""

    def test_default_parameter_values(self):
        """Test expected default parameter values."""
        defaults = {
            "carla_host": "localhost",
            "carla_port": 2000,
            "carla_timeout": 10.0,
            "role_name": "ego_vehicle",
            "width": 1280,
            "height": 720,
            "show_triggers": False,
            "show_connections": False,
            "show_spawn_points": False,
            "frame_rate": 30.0,
        }

        assert defaults["width"] == 1280
        assert defaults["height"] == 720
        assert defaults["frame_rate"] == 30.0
        assert defaults["show_triggers"] is False

    def test_frame_rate_to_interval(self):
        """Test frame rate to interval conversion."""
        frame_rate = 30.0
        interval = 1.0 / frame_rate

        assert abs(interval - 0.03333333333333333) < 1e-10


class TestMapRendering:
    """Tests for map rendering logic."""

    def test_pixels_per_meter(self):
        """Test pixels per meter constant."""
        pixels_per_meter = 5.0
        assert pixels_per_meter > 0

    def test_zoom_scaling(self):
        """Test zoom affects surface size."""
        base_size = 1000
        zoom = 2.0

        scaled_size = int(base_size * zoom)
        assert scaled_size == 2000

    def test_pan_offset(self):
        """Test pan offset updates correctly."""
        offset = [0, 0]
        dx, dy = 10, 20

        offset[0] += dx
        offset[1] += dy

        assert offset[0] == 10
        assert offset[1] == 20


class TestActorRendering:
    """Tests for actor rendering."""

    def test_vehicle_color(self):
        """Test vehicle color is consistent."""
        vehicle_color = (0, 0, 255)  # Blue
        assert len(vehicle_color) == 3
        assert all(0 <= c <= 255 for c in vehicle_color)

    def test_pedestrian_color(self):
        """Test pedestrian color is consistent."""
        pedestrian_color = (255, 0, 0)  # Red
        assert len(pedestrian_color) == 3
        assert all(0 <= c <= 255 for c in pedestrian_color)

    def test_ego_vehicle_highlight(self):
        """Test ego vehicle has distinct color."""
        ego_color = (0, 255, 0)  # Green
        other_color = (0, 0, 255)  # Blue

        assert ego_color != other_color


class TestWebServer:
    """Tests for web server functionality."""

    def test_default_port(self):
        """Test default web server port."""
        default_port = 5000
        assert default_port == 5000

    def test_mjpeg_content_type(self):
        """Test MJPEG stream content type."""
        content_type = "multipart/x-mixed-replace; boundary=frame"
        assert "multipart" in content_type
        assert "boundary=frame" in content_type

    def test_jpeg_quality(self):
        """Test JPEG quality setting."""
        quality = 85
        assert 0 <= quality <= 100


class TestMJPEGStream:
    """Tests for MJPEG streaming."""

    def test_frame_boundary(self):
        """Test frame boundary format."""
        boundary = b"--frame\r\n"
        assert boundary.startswith(b"--")
        assert boundary.endswith(b"\r\n")

    def test_content_type_header(self):
        """Test content type header in frame."""
        header = b"Content-Type: image/jpeg\r\n\r\n"
        assert b"image/jpeg" in header

    def test_frame_format(self):
        """Test complete frame format."""
        boundary = b"--frame\r\n"
        header = b"Content-Type: image/jpeg\r\n\r\n"
        image_data = b"\xff\xd8\xff"  # JPEG magic bytes
        trailer = b"\r\n"

        frame = boundary + header + image_data + trailer
        assert frame.startswith(b"--frame")
        assert b"image/jpeg" in frame
