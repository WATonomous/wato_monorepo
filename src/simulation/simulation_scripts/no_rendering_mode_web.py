#!/usr/bin/env python

# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Web-based CARLA No-Rendering Mode Visualizer

Serves an interactive map visualization through a web browser.
Controls:
    Mouse Wheel  : zoom in / zoom out
    Mouse Drag   : pan map
"""

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import carla
from carla import TrafficLightState as tls

import argparse
import logging
import datetime
import weakref
import math
import random
import hashlib
import threading
import base64
import io
import os
import glob

# Set SDL to use dummy video driver for headless operation
os.environ['SDL_VIDEODRIVER'] = 'dummy'
os.environ['SDL_AUDIODRIVER'] = 'dummy'

import pygame
from flask import Flask, render_template_string
from flask_socketio import SocketIO, emit

# ==============================================================================
# -- Constants -----------------------------------------------------------------
# ==============================================================================

# Colors
COLOR_BUTTER_0 = pygame.Color(252, 233, 79)
COLOR_BUTTER_1 = pygame.Color(237, 212, 0)
COLOR_BUTTER_2 = pygame.Color(196, 160, 0)

COLOR_ORANGE_0 = pygame.Color(252, 175, 62)
COLOR_ORANGE_1 = pygame.Color(245, 121, 0)
COLOR_ORANGE_2 = pygame.Color(209, 92, 0)

COLOR_CHOCOLATE_0 = pygame.Color(233, 185, 110)
COLOR_CHOCOLATE_1 = pygame.Color(193, 125, 17)
COLOR_CHOCOLATE_2 = pygame.Color(143, 89, 2)

COLOR_CHAMELEON_0 = pygame.Color(138, 226, 52)
COLOR_CHAMELEON_1 = pygame.Color(115, 210, 22)
COLOR_CHAMELEON_2 = pygame.Color(78, 154, 6)

COLOR_SKY_BLUE_0 = pygame.Color(114, 159, 207)
COLOR_SKY_BLUE_1 = pygame.Color(52, 101, 164)
COLOR_SKY_BLUE_2 = pygame.Color(32, 74, 135)

COLOR_PLUM_0 = pygame.Color(173, 127, 168)
COLOR_PLUM_1 = pygame.Color(117, 80, 123)
COLOR_PLUM_2 = pygame.Color(92, 53, 102)

COLOR_SCARLET_RED_0 = pygame.Color(239, 41, 41)
COLOR_SCARLET_RED_1 = pygame.Color(204, 0, 0)
COLOR_SCARLET_RED_2 = pygame.Color(164, 0, 0)

COLOR_ALUMINIUM_0 = pygame.Color(238, 238, 236)
COLOR_ALUMINIUM_1 = pygame.Color(211, 215, 207)
COLOR_ALUMINIUM_2 = pygame.Color(186, 189, 182)
COLOR_ALUMINIUM_3 = pygame.Color(136, 138, 133)
COLOR_ALUMINIUM_4 = pygame.Color(85, 87, 83)
COLOR_ALUMINIUM_4_5 = pygame.Color(66, 62, 64)
COLOR_ALUMINIUM_5 = pygame.Color(46, 52, 54)

COLOR_WHITE = pygame.Color(255, 255, 255)
COLOR_BLACK = pygame.Color(0, 0, 0)

# Module Defines
TITLE_WORLD = 'WORLD'
PIXELS_PER_METER = 12
MAP_DEFAULT_SCALE = 0.1

# ==============================================================================
# -- HTML Template -------------------------------------------------------------
# ==============================================================================

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>CARLA Map Visualizer</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            background: #1a1a1a;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
        }
        .container {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 10px;
        }
        h1 {
            color: #eee;
            font-size: 1.5em;
            font-weight: normal;
        }
        #status {
            color: #888;
            font-size: 0.9em;
        }
        #status.connected { color: #8ae234; }
        #status.disconnected { color: #ef2929; }
        #canvas-container {
            position: relative;
            border: 2px solid #333;
            border-radius: 4px;
            overflow: hidden;
        }
        #mapCanvas {
            display: block;
            cursor: grab;
        }
        #mapCanvas:active {
            cursor: grabbing;
        }
        .controls {
            color: #888;
            font-size: 0.85em;
            text-align: center;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>CARLA Map Visualizer</h1>
        <div id="status" class="disconnected">Connecting...</div>
        <div id="canvas-container">
            <canvas id="mapCanvas" width="{{ width }}" height="{{ height }}"></canvas>
        </div>
        <div class="controls">
            Scroll to zoom | Drag to pan
        </div>
    </div>

    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <script>
        const canvas = document.getElementById('mapCanvas');
        const ctx = canvas.getContext('2d');
        const status = document.getElementById('status');

        const socket = io();

        let isDragging = false;
        let lastMousePos = { x: 0, y: 0 };

        socket.on('connect', () => {
            status.textContent = 'Connected';
            status.className = 'connected';
        });

        socket.on('disconnect', () => {
            status.textContent = 'Disconnected';
            status.className = 'disconnected';
        });

        socket.on('frame', (data) => {
            const img = new Image();
            img.onload = () => {
                ctx.drawImage(img, 0, 0);
            };
            img.src = 'data:image/png;base64,' + data;
        });

        // Mouse drag for panning
        canvas.addEventListener('mousedown', (e) => {
            isDragging = true;
            lastMousePos = { x: e.clientX, y: e.clientY };
            canvas.style.cursor = 'grabbing';
        });

        canvas.addEventListener('mousemove', (e) => {
            if (isDragging) {
                const dx = e.clientX - lastMousePos.x;
                const dy = e.clientY - lastMousePos.y;
                lastMousePos = { x: e.clientX, y: e.clientY };
                socket.emit('pan', { dx: dx, dy: dy });
            }
        });

        canvas.addEventListener('mouseup', () => {
            isDragging = false;
            canvas.style.cursor = 'grab';
        });

        canvas.addEventListener('mouseleave', () => {
            isDragging = false;
            canvas.style.cursor = 'grab';
        });

        // Mouse wheel for zooming
        canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const delta = e.deltaY > 0 ? -1 : 1;
            const rect = canvas.getBoundingClientRect();
            const x = e.clientX - rect.left;
            const y = e.clientY - rect.top;
            socket.emit('zoom', { delta: delta, x: x, y: y });
        });

        // Prevent context menu on right-click
        canvas.addEventListener('contextmenu', (e) => e.preventDefault());
    </script>
</body>
</html>
"""

# ==============================================================================
# -- Util ----------------------------------------------------------------------
# ==============================================================================

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


class Util(object):

    @staticmethod
    def length(v):
        return math.sqrt(v.x**2 + v.y**2 + v.z**2)

    @staticmethod
    def get_bounding_box(actor):
        bb = actor.trigger_volume.extent
        corners = [carla.Location(x=-bb.x, y=-bb.y),
                   carla.Location(x=bb.x, y=-bb.y),
                   carla.Location(x=bb.x, y=bb.y),
                   carla.Location(x=-bb.x, y=bb.y),
                   carla.Location(x=-bb.x, y=-bb.y)]
        corners = [x + actor.trigger_volume.location for x in corners]
        t = actor.get_transform()
        t.transform(corners)
        return corners


# ==============================================================================
# -- TrafficLightSurfaces ------------------------------------------------------
# ==============================================================================

class TrafficLightSurfaces(object):
    def __init__(self):
        def make_surface(tl):
            w = 40
            surface = pygame.Surface((w, 3 * w), pygame.SRCALPHA)
            surface.fill(COLOR_ALUMINIUM_5 if tl != 'h' else COLOR_ORANGE_2)
            if tl != 'h':
                hw = int(w / 2)
                off = COLOR_ALUMINIUM_4
                red = COLOR_SCARLET_RED_0
                yellow = COLOR_BUTTER_0
                green = COLOR_CHAMELEON_0
                pygame.draw.circle(surface, red if tl == tls.Red else off, (hw, hw), int(0.4 * w))
                pygame.draw.circle(surface, yellow if tl == tls.Yellow else off, (hw, w + hw), int(0.4 * w))
                pygame.draw.circle(surface, green if tl == tls.Green else off, (hw, 2 * w + hw), int(0.4 * w))
            return pygame.transform.smoothscale(surface, (15, 45) if tl != 'h' else (19, 49))

        self._original_surfaces = {
            'h': make_surface('h'),
            tls.Red: make_surface(tls.Red),
            tls.Yellow: make_surface(tls.Yellow),
            tls.Green: make_surface(tls.Green),
            tls.Off: make_surface(tls.Off),
            tls.Unknown: make_surface(tls.Unknown)
        }
        self.surfaces = dict(self._original_surfaces)

    def rotozoom(self, angle, scale):
        for key, surface in self._original_surfaces.items():
            self.surfaces[key] = pygame.transform.rotozoom(surface, angle, scale)


# ==============================================================================
# -- MapImage ------------------------------------------------------------------
# ==============================================================================

class MapImage(object):
    def __init__(self, carla_world, carla_map, pixels_per_meter, show_triggers, show_connections, show_spawn_points):
        self._pixels_per_meter = pixels_per_meter
        self.scale = 1.0
        self.show_triggers = show_triggers
        self.show_connections = show_connections
        self.show_spawn_points = show_spawn_points

        waypoints = carla_map.generate_waypoints(2)
        margin = 50
        max_x = max(waypoints, key=lambda x: x.transform.location.x).transform.location.x + margin
        max_y = max(waypoints, key=lambda x: x.transform.location.y).transform.location.y + margin
        min_x = min(waypoints, key=lambda x: x.transform.location.x).transform.location.x - margin
        min_y = min(waypoints, key=lambda x: x.transform.location.y).transform.location.y - margin

        self.width = max(max_x - min_x, max_y - min_y)
        self._world_offset = (min_x, min_y)

        width_in_pixels = (1 << 14) - 1
        surface_pixel_per_meter = int(width_in_pixels / self.width)
        if surface_pixel_per_meter > PIXELS_PER_METER:
            surface_pixel_per_meter = PIXELS_PER_METER

        self._pixels_per_meter = surface_pixel_per_meter
        width_in_pixels = int(self._pixels_per_meter * self.width)

        self.big_map_surface = pygame.Surface((width_in_pixels, width_in_pixels)).convert()

        opendrive_content = carla_map.to_opendrive()
        hash_func = hashlib.sha1()
        hash_func.update(opendrive_content.encode("UTF-8"))
        opendrive_hash = str(hash_func.hexdigest())

        filename = carla_map.name.split('/')[-1] + "_" + opendrive_hash + ".tga"
        dirname = os.path.join("cache", "no_rendering_mode")
        full_path = str(os.path.join(dirname, filename))

        if os.path.isfile(full_path):
            self.big_map_surface = pygame.image.load(full_path)
        else:
            self.draw_road_map(
                self.big_map_surface,
                carla_world,
                carla_map,
                self.world_to_pixel,
                self.world_to_pixel_width)

            if not os.path.exists(dirname):
                os.makedirs(dirname)

            list_filenames = glob.glob(os.path.join(dirname, carla_map.name) + "*")
            for town_filename in list_filenames:
                os.remove(town_filename)

            pygame.image.save(self.big_map_surface, full_path)

        self.surface = self.big_map_surface

    def draw_road_map(self, map_surface, carla_world, carla_map, world_to_pixel, world_to_pixel_width):
        map_surface.fill(COLOR_ALUMINIUM_4)
        precision = 0.05

        def lane_marking_color_to_tango(lane_marking_color):
            tango_color = COLOR_BLACK
            if lane_marking_color == carla.LaneMarkingColor.White:
                tango_color = COLOR_ALUMINIUM_2
            elif lane_marking_color == carla.LaneMarkingColor.Blue:
                tango_color = COLOR_SKY_BLUE_0
            elif lane_marking_color == carla.LaneMarkingColor.Green:
                tango_color = COLOR_CHAMELEON_0
            elif lane_marking_color == carla.LaneMarkingColor.Red:
                tango_color = COLOR_SCARLET_RED_0
            elif lane_marking_color == carla.LaneMarkingColor.Yellow:
                tango_color = COLOR_ORANGE_0
            return tango_color

        def draw_solid_line(surface, color, closed, points, width):
            if len(points) >= 2:
                pygame.draw.lines(surface, color, closed, points, width)

        def draw_broken_line(surface, color, closed, points, width):
            broken_lines = [x for n, x in enumerate(zip(*(iter(points),) * 20)) if n % 3 == 0]
            for line in broken_lines:
                pygame.draw.lines(surface, color, closed, line, width)

        def get_lane_markings(lane_marking_type, lane_marking_color, waypoints, sign):
            margin = 0.25
            marking_1 = [world_to_pixel(lateral_shift(w.transform, sign * w.lane_width * 0.5)) for w in waypoints]
            if lane_marking_type == carla.LaneMarkingType.Broken or (lane_marking_type == carla.LaneMarkingType.Solid):
                return [(lane_marking_type, lane_marking_color, marking_1)]
            else:
                marking_2 = [world_to_pixel(lateral_shift(w.transform,
                                                          sign * (w.lane_width * 0.5 + margin * 2))) for w in waypoints]
                if lane_marking_type == carla.LaneMarkingType.SolidBroken:
                    return [(carla.LaneMarkingType.Broken, lane_marking_color, marking_1),
                            (carla.LaneMarkingType.Solid, lane_marking_color, marking_2)]
                elif lane_marking_type == carla.LaneMarkingType.BrokenSolid:
                    return [(carla.LaneMarkingType.Solid, lane_marking_color, marking_1),
                            (carla.LaneMarkingType.Broken, lane_marking_color, marking_2)]
                elif lane_marking_type == carla.LaneMarkingType.BrokenBroken:
                    return [(carla.LaneMarkingType.Broken, lane_marking_color, marking_1),
                            (carla.LaneMarkingType.Broken, lane_marking_color, marking_2)]
                elif lane_marking_type == carla.LaneMarkingType.SolidSolid:
                    return [(carla.LaneMarkingType.Solid, lane_marking_color, marking_1),
                            (carla.LaneMarkingType.Solid, lane_marking_color, marking_2)]
            return [(carla.LaneMarkingType.NONE, carla.LaneMarkingColor.Other, [])]

        def draw_lane(surface, lane, color):
            for side in lane:
                lane_left_side = [lateral_shift(w.transform, -w.lane_width * 0.5) for w in side]
                lane_right_side = [lateral_shift(w.transform, w.lane_width * 0.5) for w in side]
                polygon = lane_left_side + [x for x in reversed(lane_right_side)]
                polygon = [world_to_pixel(x) for x in polygon]
                if len(polygon) > 2:
                    pygame.draw.polygon(surface, color, polygon, 5)
                    pygame.draw.polygon(surface, color, polygon)

        def draw_lane_marking(surface, waypoints):
            draw_lane_marking_single_side(surface, waypoints[0], -1)
            draw_lane_marking_single_side(surface, waypoints[1], 1)

        def draw_lane_marking_single_side(surface, waypoints, sign):
            lane_marking = None
            marking_type = carla.LaneMarkingType.NONE
            previous_marking_type = carla.LaneMarkingType.NONE
            marking_color = carla.LaneMarkingColor.Other
            previous_marking_color = carla.LaneMarkingColor.Other
            markings_list = []
            temp_waypoints = []
            current_lane_marking = carla.LaneMarkingType.NONE

            for sample in waypoints:
                lane_marking = sample.left_lane_marking if sign < 0 else sample.right_lane_marking
                if lane_marking is None:
                    continue
                marking_type = lane_marking.type
                marking_color = lane_marking.color
                if current_lane_marking != marking_type:
                    markings = get_lane_markings(
                        previous_marking_type,
                        lane_marking_color_to_tango(previous_marking_color),
                        temp_waypoints,
                        sign)
                    current_lane_marking = marking_type
                    for marking in markings:
                        markings_list.append(marking)
                    temp_waypoints = temp_waypoints[-1:]
                else:
                    temp_waypoints.append((sample))
                    previous_marking_type = marking_type
                    previous_marking_color = marking_color

            last_markings = get_lane_markings(
                previous_marking_type,
                lane_marking_color_to_tango(previous_marking_color),
                temp_waypoints,
                sign)
            for marking in last_markings:
                markings_list.append(marking)

            for markings in markings_list:
                if markings[0] == carla.LaneMarkingType.Solid:
                    draw_solid_line(surface, markings[1], False, markings[2], 2)
                elif markings[0] == carla.LaneMarkingType.Broken:
                    draw_broken_line(surface, markings[1], False, markings[2], 2)

        def draw_arrow(surface, transform, color=COLOR_ALUMINIUM_2):
            transform.rotation.yaw += 180
            forward = transform.get_forward_vector()
            transform.rotation.yaw += 90
            right_dir = transform.get_forward_vector()
            end = transform.location
            start = end - 2.0 * forward
            right = start + 0.8 * forward + 0.4 * right_dir
            left = start + 0.8 * forward - 0.4 * right_dir
            pygame.draw.lines(surface, color, False, [world_to_pixel(x) for x in [start, end]], 4)
            pygame.draw.lines(surface, color, False, [world_to_pixel(x) for x in [left, start, right]], 4)

        def draw_traffic_signs(surface, font_surface, actor, color=COLOR_ALUMINIUM_2, trigger_color=COLOR_PLUM_0):
            transform = actor.get_transform()
            waypoint = carla_map.get_waypoint(transform.location)
            angle = -waypoint.transform.rotation.yaw - 90.0
            font_surface = pygame.transform.rotate(font_surface, angle)
            pixel_pos = world_to_pixel(waypoint.transform.location)
            offset = font_surface.get_rect(center=(pixel_pos[0], pixel_pos[1]))
            surface.blit(font_surface, offset)

            forward_vector = carla.Location(waypoint.transform.get_forward_vector())
            left_vector = carla.Location(-forward_vector.y, forward_vector.x,
                                         forward_vector.z) * waypoint.lane_width / 2 * 0.7
            line = [(waypoint.transform.location + (forward_vector * 1.5) + (left_vector)),
                    (waypoint.transform.location + (forward_vector * 1.5) - (left_vector))]
            line_pixel = [world_to_pixel(p) for p in line]
            pygame.draw.lines(surface, color, True, line_pixel, 2)

            if self.show_triggers:
                corners = Util.get_bounding_box(actor)
                corners = [world_to_pixel(p) for p in corners]
                pygame.draw.lines(surface, trigger_color, True, corners, 2)

        def lateral_shift(transform, shift):
            transform.rotation.yaw += 90
            return transform.location + shift * transform.get_forward_vector()

        def draw_topology(carla_topology, index):
            topology = [x[index] for x in carla_topology]
            topology = sorted(topology, key=lambda w: w.transform.location.z)
            set_waypoints = []

            for waypoint in topology:
                waypoints = [waypoint]
                nxt = waypoint.next(precision)
                if len(nxt) > 0:
                    nxt = nxt[0]
                    while nxt.road_id == waypoint.road_id:
                        waypoints.append(nxt)
                        nxt = nxt.next(precision)
                        if len(nxt) > 0:
                            nxt = nxt[0]
                        else:
                            break
                set_waypoints.append(waypoints)

                PARKING_COLOR = COLOR_ALUMINIUM_4_5
                SHOULDER_COLOR = COLOR_ALUMINIUM_5
                SIDEWALK_COLOR = COLOR_ALUMINIUM_3

                shoulder = [[], []]
                parking = [[], []]
                sidewalk = [[], []]

                for w in waypoints:
                    l = w.get_left_lane()
                    while l and l.lane_type != carla.LaneType.Driving:
                        if l.lane_type == carla.LaneType.Shoulder:
                            shoulder[0].append(l)
                        if l.lane_type == carla.LaneType.Parking:
                            parking[0].append(l)
                        if l.lane_type == carla.LaneType.Sidewalk:
                            sidewalk[0].append(l)
                        l = l.get_left_lane()

                    r = w.get_right_lane()
                    while r and r.lane_type != carla.LaneType.Driving:
                        if r.lane_type == carla.LaneType.Shoulder:
                            shoulder[1].append(r)
                        if r.lane_type == carla.LaneType.Parking:
                            parking[1].append(r)
                        if r.lane_type == carla.LaneType.Sidewalk:
                            sidewalk[1].append(r)
                        r = r.get_right_lane()

                draw_lane(map_surface, shoulder, SHOULDER_COLOR)
                draw_lane(map_surface, parking, PARKING_COLOR)
                draw_lane(map_surface, sidewalk, SIDEWALK_COLOR)

            for waypoints in set_waypoints:
                waypoint = waypoints[0]
                road_left_side = [lateral_shift(w.transform, -w.lane_width * 0.5) for w in waypoints]
                road_right_side = [lateral_shift(w.transform, w.lane_width * 0.5) for w in waypoints]
                polygon = road_left_side + [x for x in reversed(road_right_side)]
                polygon = [world_to_pixel(x) for x in polygon]

                if len(polygon) > 2:
                    pygame.draw.polygon(map_surface, COLOR_ALUMINIUM_5, polygon, 5)
                    pygame.draw.polygon(map_surface, COLOR_ALUMINIUM_5, polygon)

                if not waypoint.is_junction:
                    draw_lane_marking(map_surface, [waypoints, waypoints])
                    for n, wp in enumerate(waypoints):
                        if ((n + 1) % 400) == 0:
                            draw_arrow(map_surface, wp.transform)

        topology = carla_map.get_topology()
        draw_topology(topology, 0)

        if self.show_spawn_points:
            for sp in carla_map.get_spawn_points():
                draw_arrow(map_surface, sp, color=COLOR_CHOCOLATE_0)

        if self.show_connections:
            dist = 1.5
            def to_pixel(wp): return world_to_pixel(wp.transform.location)
            for wp in carla_map.generate_waypoints(dist):
                col = (0, 255, 255) if wp.is_junction else (0, 255, 0)
                for nxt in wp.next(dist):
                    pygame.draw.line(map_surface, col, to_pixel(wp), to_pixel(nxt), 2)
                if wp.lane_change & carla.LaneChange.Right:
                    r = wp.get_right_lane()
                    if r and r.lane_type == carla.LaneType.Driving:
                        pygame.draw.line(map_surface, col, to_pixel(wp), to_pixel(r), 2)
                if wp.lane_change & carla.LaneChange.Left:
                    l = wp.get_left_lane()
                    if l and l.lane_type == carla.LaneType.Driving:
                        pygame.draw.line(map_surface, col, to_pixel(wp), to_pixel(l), 2)

        actors = carla_world.get_actors()
        font_size = world_to_pixel_width(1)
        font = pygame.font.SysFont('Arial', font_size, True)

        stops = [actor for actor in actors if 'stop' in actor.type_id]
        yields = [actor for actor in actors if 'yield' in actor.type_id]

        stop_font_surface = font.render("STOP", False, COLOR_ALUMINIUM_2)
        stop_font_surface = pygame.transform.scale(
            stop_font_surface, (stop_font_surface.get_width(), stop_font_surface.get_height() * 2))

        yield_font_surface = font.render("YIELD", False, COLOR_ALUMINIUM_2)
        yield_font_surface = pygame.transform.scale(
            yield_font_surface, (yield_font_surface.get_width(), yield_font_surface.get_height() * 2))

        for ts_stop in stops:
            draw_traffic_signs(map_surface, stop_font_surface, ts_stop, trigger_color=COLOR_SCARLET_RED_1)

        for ts_yield in yields:
            draw_traffic_signs(map_surface, yield_font_surface, ts_yield, trigger_color=COLOR_ORANGE_1)

    def world_to_pixel(self, location, offset=(0, 0)):
        x = self.scale * self._pixels_per_meter * (location.x - self._world_offset[0])
        y = self.scale * self._pixels_per_meter * (location.y - self._world_offset[1])
        return [int(x - offset[0]), int(y - offset[1])]

    def world_to_pixel_width(self, width):
        return int(self.scale * self._pixels_per_meter * width)

    def scale_map(self, scale):
        if scale != self.scale:
            self.scale = scale
            width = int(self.big_map_surface.get_width() * self.scale)
            self.surface = pygame.transform.smoothscale(self.big_map_surface, (width, width))


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================

class World(object):
    def __init__(self, name, args, timeout):
        self.client = None
        self.name = name
        self.args = args
        self.timeout = timeout
        self.server_fps = 0.0
        self.simulation_time = 0
        self.server_clock = pygame.time.Clock()

        self.world = None
        self.town_map = None
        self.actors_with_transforms = []

        self.dim = (args.width, args.height)

        self.surface_size = [0, 0]
        self.prev_scaled_size = 0
        self.scaled_size = 0

        self.hero_actor = None
        self.hero_transform = None

        self.mouse_offset = [0.0, 0.0]
        self.wheel_offset = MAP_DEFAULT_SCALE

        self.vehicle_id_surface = None
        self.result_surface = None

        self.traffic_light_surfaces = TrafficLightSurfaces()
        self.affected_traffic_light = None

        self.map_image = None
        self.original_surface_size = None
        self.actors_surface = None

        # Final display surface
        self.display_surface = None

    def _get_data_from_carla(self):
        try:
            self.client = carla.Client(self.args.host, self.args.port)
            self.client.set_timeout(self.timeout)

            if self.args.map is None:
                world = self.client.get_world()
            else:
                world = self.client.load_world(self.args.map)

            town_map = world.get_map()
            return (world, town_map)

        except RuntimeError as ex:
            logging.error(ex)
            return None, None

    def start(self):
        self.world, self.town_map = self._get_data_from_carla()
        if self.world is None:
            return False

        settings = self.world.get_settings()
        settings.no_rendering_mode = True
        self.world.apply_settings(settings)

        self.map_image = MapImage(
            carla_world=self.world,
            carla_map=self.town_map,
            pixels_per_meter=PIXELS_PER_METER,
            show_triggers=self.args.show_triggers,
            show_connections=self.args.show_connections,
            show_spawn_points=self.args.show_spawn_points)

        self.original_surface_size = min(self.dim[0], self.dim[1])
        self.surface_size = self.map_image.big_map_surface.get_width()

        self.scaled_size = int(self.surface_size)
        self.prev_scaled_size = int(self.surface_size)

        self.actors_surface = pygame.Surface((self.map_image.surface.get_width(), self.map_image.surface.get_height()))
        self.actors_surface.set_colorkey(COLOR_BLACK)

        self.vehicle_id_surface = pygame.Surface((self.surface_size, self.surface_size)).convert()
        self.vehicle_id_surface.set_colorkey(COLOR_BLACK)

        self.result_surface = pygame.Surface((self.surface_size, self.surface_size)).convert()
        self.result_surface.set_colorkey(COLOR_BLACK)

        self.display_surface = pygame.Surface(self.dim).convert()

        self._find_hero_actor()

        weak_self = weakref.ref(self)
        self.world.on_tick(lambda timestamp: World.on_world_tick(weak_self, timestamp))

        return True

    def _find_hero_actor(self):
        hero_vehicles = [actor for actor in self.world.get_actors()
                         if 'vehicle' in actor.type_id and actor.attributes['role_name'] == 'hero']
        if len(hero_vehicles) > 0:
            self.hero_actor = random.choice(hero_vehicles)
            self.hero_transform = self.hero_actor.get_transform()
        else:
            self.hero_actor = None
            self.hero_transform = None

    def tick(self):
        actors = self.world.get_actors()
        self.actors_with_transforms = [(actor, actor.get_transform()) for actor in actors]
        if self.hero_actor is not None:
            self.hero_transform = self.hero_actor.get_transform()

    @staticmethod
    def on_world_tick(weak_self, timestamp):
        self = weak_self()
        if not self:
            return
        self.server_clock.tick()
        self.server_fps = self.server_clock.get_fps()
        self.simulation_time = timestamp.elapsed_seconds

    def _split_actors(self):
        vehicles = []
        traffic_lights = []
        speed_limits = []
        walkers = []

        for actor_with_transform in self.actors_with_transforms:
            actor = actor_with_transform[0]
            if 'vehicle' in actor.type_id:
                vehicles.append(actor_with_transform)
            elif 'traffic_light' in actor.type_id:
                traffic_lights.append(actor_with_transform)
            elif 'speed_limit' in actor.type_id:
                speed_limits.append(actor_with_transform)
            elif 'walker.pedestrian' in actor.type_id:
                walkers.append(actor_with_transform)

        return (vehicles, traffic_lights, speed_limits, walkers)

    def _render_traffic_lights(self, surface, list_tl, world_to_pixel):
        self.affected_traffic_light = None
        for tl in list_tl:
            world_pos = tl.get_location()
            pos = world_to_pixel(world_pos)

            if self.args.show_triggers:
                corners = Util.get_bounding_box(tl)
                corners = [world_to_pixel(p) for p in corners]
                pygame.draw.lines(surface, COLOR_BUTTER_1, True, corners, 2)

            if self.hero_actor is not None:
                corners = Util.get_bounding_box(tl)
                corners = [world_to_pixel(p) for p in corners]
                tl_t = tl.get_transform()
                transformed_tv = tl_t.transform(tl.trigger_volume.location)
                hero_location = self.hero_actor.get_location()
                d = hero_location.distance(transformed_tv)
                s = Util.length(tl.trigger_volume.extent) + Util.length(self.hero_actor.bounding_box.extent)
                if (d <= s):
                    self.affected_traffic_light = tl
                    srf = self.traffic_light_surfaces.surfaces['h']
                    surface.blit(srf, srf.get_rect(center=pos))

            srf = self.traffic_light_surfaces.surfaces[tl.state]
            surface.blit(srf, srf.get_rect(center=pos))

    def _render_speed_limits(self, surface, list_sl, world_to_pixel, world_to_pixel_width):
        font_size = world_to_pixel_width(2)
        radius = world_to_pixel_width(2)
        font = pygame.font.SysFont('Arial', font_size)

        for sl in list_sl:
            x, y = world_to_pixel(sl.get_location())
            white_circle_radius = int(radius * 0.75)
            pygame.draw.circle(surface, COLOR_SCARLET_RED_1, (x, y), radius)
            pygame.draw.circle(surface, COLOR_ALUMINIUM_0, (x, y), white_circle_radius)

            limit = sl.type_id.split('.')[2]
            font_surface = font.render(limit, True, COLOR_ALUMINIUM_5)

            if self.args.show_triggers:
                corners = Util.get_bounding_box(sl)
                corners = [world_to_pixel(p) for p in corners]
                pygame.draw.lines(surface, COLOR_PLUM_2, True, corners, 2)

            surface.blit(font_surface, (x - radius / 2, y - radius / 2))

    def _render_walkers(self, surface, list_w, world_to_pixel):
        for w in list_w:
            color = COLOR_PLUM_0
            bb = w[0].bounding_box.extent
            corners = [
                carla.Location(x=-bb.x, y=-bb.y),
                carla.Location(x=bb.x, y=-bb.y),
                carla.Location(x=bb.x, y=bb.y),
                carla.Location(x=-bb.x, y=bb.y)]
            w[1].transform(corners)
            corners = [world_to_pixel(p) for p in corners]
            pygame.draw.polygon(surface, color, corners)

    def _render_vehicles(self, surface, list_v, world_to_pixel):
        for v in list_v:
            color = COLOR_SKY_BLUE_0
            if int(v[0].attributes['number_of_wheels']) == 2:
                color = COLOR_CHOCOLATE_1
            if v[0].attributes['role_name'] == 'hero':
                color = COLOR_CHAMELEON_0
            bb = v[0].bounding_box.extent
            corners = [carla.Location(x=-bb.x, y=-bb.y),
                       carla.Location(x=bb.x - 0.8, y=-bb.y),
                       carla.Location(x=bb.x, y=0),
                       carla.Location(x=bb.x - 0.8, y=bb.y),
                       carla.Location(x=-bb.x, y=bb.y),
                       carla.Location(x=-bb.x, y=-bb.y)]
            v[1].transform(corners)
            corners = [world_to_pixel(p) for p in corners]
            pygame.draw.lines(surface, color, False, corners, int(math.ceil(4.0 * self.map_image.scale)))

    def render_actors(self, surface, vehicles, traffic_lights, speed_limits, walkers):
        self._render_traffic_lights(surface, [tl[0] for tl in traffic_lights], self.map_image.world_to_pixel)
        self._render_speed_limits(surface, [sl[0] for sl in speed_limits], self.map_image.world_to_pixel,
                                  self.map_image.world_to_pixel_width)
        self._render_vehicles(surface, vehicles, self.map_image.world_to_pixel)
        self._render_walkers(surface, walkers, self.map_image.world_to_pixel)

    def _compute_scale(self, scale_factor):
        self.prev_scaled_size = self.scaled_size
        self.map_image.scale_map(scale_factor)

    def render(self):
        if self.actors_with_transforms is None:
            return None

        self.result_surface.fill(COLOR_BLACK)
        self.display_surface.fill(COLOR_ALUMINIUM_4)

        vehicles, traffic_lights, speed_limits, walkers = self._split_actors()

        scale_factor = self.wheel_offset
        self.scaled_size = int(self.map_image.width * scale_factor)

        if self.scaled_size != self.prev_scaled_size:
            self._compute_scale(scale_factor)

        self.actors_surface.fill(COLOR_BLACK)
        self.render_actors(
            self.actors_surface,
            vehicles,
            traffic_lights,
            speed_limits,
            walkers)

        self.traffic_light_surfaces.rotozoom(0, self.map_image.scale)

        # Blit map and actors to result surface
        self.result_surface.fill(COLOR_BLACK)
        self.result_surface.blit(self.map_image.surface, (0, 0))
        self.result_surface.blit(self.actors_surface, (0, 0))
        self.result_surface.blit(self.vehicle_id_surface, (0, 0))

        # Calculate pan offset from mouse drag
        pan_x = self.mouse_offset[0]
        pan_y = self.mouse_offset[1]

        # Blit the scaled map to display, centered with pan offset
        scaled_map_size = self.map_image.surface.get_size()
        dest_x = (self.dim[0] - scaled_map_size[0]) // 2 + int(pan_x)
        dest_y = (self.dim[1] - scaled_map_size[1]) // 2 + int(pan_y)

        self.display_surface.blit(self.result_surface, (dest_x, dest_y),
                                  area=pygame.Rect(0, 0, scaled_map_size[0], scaled_map_size[1]))

        # Draw HUD
        self._render_hud()

        return self.display_surface

    def _render_hud(self):
        """Render HUD overlay on display surface"""
        font = pygame.font.SysFont('monospace', 14)

        # Semi-transparent background
        hud_surface = pygame.Surface((260, 200))
        hud_surface.fill(COLOR_ALUMINIUM_5)
        hud_surface.set_alpha(180)
        self.display_surface.blit(hud_surface, (0, 0))

        # Prepare info text
        map_name = self.town_map.name.split('/')[-1]
        if len(map_name) > 20:
            map_name = map_name[:19] + '\u2026'

        server_fps = 'inf' if self.server_fps == float('inf') else round(self.server_fps)

        lines = [
            f"Server:  {server_fps:>16} FPS",
            f"Simulation: {datetime.timedelta(seconds=int(self.simulation_time))}",
            f"Map: {map_name}",
            "",
        ]

        if self.hero_actor is not None:
            ego_speed = self.hero_actor.get_velocity()
            speed_kmh = int(3.6 * math.sqrt(ego_speed.x ** 2 + ego_speed.y ** 2 + ego_speed.z ** 2))
            lines.extend([
                "EGO VEHICLE",
                f"  ID: {self.hero_actor.id}",
                f"  Speed: {speed_kmh} km/h",
            ])
        else:
            lines.append("EGO: Not connected")

        y = 10
        for line in lines:
            text_surface = font.render(line, True, COLOR_ALUMINIUM_0)
            self.display_surface.blit(text_surface, (10, y))
            y += 20

    def handle_pan(self, dx, dy):
        self.mouse_offset[0] += dx
        self.mouse_offset[1] += dy

    def handle_zoom(self, delta, x, y):
        old_zoom = self.wheel_offset
        self.wheel_offset += delta * 0.025
        self.wheel_offset = max(0.05, min(2.0, self.wheel_offset))

        # Zoom toward cursor position
        # The cursor is at (x, y) on screen
        # The map is centered at (dim/2 + mouse_offset)
        # We want the world point under cursor to stay under cursor after zoom

        # Current map center on screen
        map_center_x = self.dim[0] / 2 + self.mouse_offset[0]
        map_center_y = self.dim[1] / 2 + self.mouse_offset[1]

        # Vector from map center to cursor (in screen space)
        dx = x - map_center_x
        dy = y - map_center_y

        # After zoom, this vector scales by zoom_ratio
        zoom_ratio = self.wheel_offset / old_zoom
        new_dx = dx * zoom_ratio
        new_dy = dy * zoom_ratio

        # Adjust pan so cursor stays over same world point
        self.mouse_offset[0] += dx - new_dx
        self.mouse_offset[1] += dy - new_dy

    def destroy(self):
        if self.world is not None:
            settings = self.world.get_settings()
            settings.no_rendering_mode = False
            self.world.apply_settings(settings)


# ==============================================================================
# -- Web Server ----------------------------------------------------------------
# ==============================================================================

app = Flask(__name__)
app.config['SECRET_KEY'] = 'carla-visualizer'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

world = None
running = True


@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE, width=world.dim[0], height=world.dim[1])


@socketio.on('pan')
def handle_pan(data):
    if world:
        world.handle_pan(data['dx'], data['dy'])


@socketio.on('zoom')
def handle_zoom(data):
    if world:
        world.handle_zoom(data['delta'], data['x'], data['y'])


def frame_emitter():
    """Background thread that emits frames to connected clients"""
    global running
    from PIL import Image
    clock = pygame.time.Clock()

    while running:
        try:
            if world and world.world:
                world.tick()
                surface = world.render()

                if surface:
                    data = pygame.image.tostring(surface, 'RGB')
                    img = Image.frombytes('RGB', surface.get_size(), data)

                    buffer = io.BytesIO()
                    img.save(buffer, format='PNG', optimize=False)
                    frame_data = base64.b64encode(buffer.getvalue()).decode('utf-8')

                    socketio.emit('frame', frame_data)
        except Exception as e:
            logging.error(f"Frame emitter error: {e}")

        clock.tick(30)


def main():
    global world, running

    argparser = argparse.ArgumentParser(description='CARLA Web Visualizer')
    argparser.add_argument('-v', '--verbose', action='store_true', dest='debug', help='print debug information')
    argparser.add_argument('--host', metavar='H', default='127.0.0.1', help='CARLA server IP (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port', metavar='P', default=2000, type=int, help='CARLA TCP port (default: 2000)')
    argparser.add_argument('--res', metavar='WIDTHxHEIGHT', default='1280x720', help='window resolution (default: 1280x720)')
    argparser.add_argument('--map', metavar='TOWN', default=None, help='start a new episode at the given TOWN')
    argparser.add_argument('--show-triggers', action='store_true', help='show trigger boxes of traffic signs')
    argparser.add_argument('--show-connections', action='store_true', help='show waypoint connections')
    argparser.add_argument('--show-spawn-points', action='store_true', help='show recommended spawn points')
    argparser.add_argument('--web-port', default=5000, type=int, help='Web server port (default: 5000)')

    args = argparser.parse_args()
    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    # Initialize pygame with dummy display for headless operation
    pygame.init()
    pygame.font.init()
    pygame.display.set_mode((1, 1))

    logging.info(f'Connecting to CARLA server at {args.host}:{args.port}')

    world = World(TITLE_WORLD, args, timeout=2.0)
    if not world.start():
        logging.error('Failed to connect to CARLA server')
        return

    logging.info(f'Starting web server at http://localhost:{args.web_port}')
    print(f'\n*** Open http://localhost:{args.web_port} in your browser ***\n')

    # Start frame emitter thread
    emitter_thread = threading.Thread(target=frame_emitter, daemon=True)
    emitter_thread.start()

    try:
        socketio.run(app, host='0.0.0.0', port=args.web_port, debug=False, use_reloader=False, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        if world:
            world.destroy()
        pygame.quit()


if __name__ == '__main__':
    main()
