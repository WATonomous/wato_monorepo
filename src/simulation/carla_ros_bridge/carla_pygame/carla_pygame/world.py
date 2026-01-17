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
"""World simulation state and rendering for CARLA pygame visualization."""

import datetime
import math
import random
import weakref

try:
    import pygame
except ImportError:
    pygame = None

try:
    import carla
except ImportError:
    carla = None

from carla_pygame.constants import (
    COLOR_ALUMINIUM_0,
    COLOR_ALUMINIUM_4,
    COLOR_ALUMINIUM_5,
    COLOR_BLACK,
    COLOR_BUTTER_1,
    COLOR_CHAMELEON_0,
    COLOR_CHOCOLATE_1,
    COLOR_MAGENTA,
    COLOR_PLUM_2,
    COLOR_SCARLET_RED_1,
    COLOR_SKY_BLUE_0,
    MAP_DEFAULT_SCALE,
    PIXELS_PER_METER,
)
from carla_pygame.map_image import MapImage
from carla_pygame.traffic_light_surfaces import TrafficLightSurfaces
from carla_pygame.utils import Util


class World:
    """Manages CARLA world state and rendering for visualization."""

    def __init__(self, carla_client, width, height, role_name="ego_vehicle", show_triggers=False, show_connections=False, show_spawn_points=False):
        self.client = carla_client
        self.dim = (width, height)
        self.role_name = role_name
        self.show_triggers = show_triggers
        self.show_connections = show_connections
        self.show_spawn_points = show_spawn_points

        self.server_fps = 0.0
        self.simulation_time = 0
        self.server_clock = pygame.time.Clock()

        self.world = None
        self.town_map = None
        self.actors_with_transforms = []

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
        self.display_surface = None

    def start(self):
        """Start the world visualization by connecting to CARLA."""
        try:
            self.world = self.client.get_world()
            self.town_map = self.world.get_map()
        except RuntimeError as ex:
            return False

        settings = self.world.get_settings()
        settings.no_rendering_mode = True
        self.world.apply_settings(settings)

        self.map_image = MapImage(
            carla_world=self.world,
            carla_map=self.town_map,
            pixels_per_meter=PIXELS_PER_METER,
            show_triggers=self.show_triggers,
            show_connections=self.show_connections,
            show_spawn_points=self.show_spawn_points)

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
        self.world.on_tick(lambda timestamp: World._on_world_tick(weak_self, timestamp))

        return True

    def _find_hero_actor(self):
        """Find the hero (ego) vehicle in the world."""
        hero_vehicles = [actor for actor in self.world.get_actors()
                         if 'vehicle' in actor.type_id and actor.attributes.get('role_name') == self.role_name]
        if len(hero_vehicles) > 0:
            self.hero_actor = random.choice(hero_vehicles)
            self.hero_transform = self.hero_actor.get_transform()
        else:
            self.hero_actor = None
            self.hero_transform = None

    def tick(self):
        """Update actor transforms for the current simulation tick."""
        actors = self.world.get_actors()
        self.actors_with_transforms = [(actor, actor.get_transform()) for actor in actors]
        if self.hero_actor is not None:
            self.hero_transform = self.hero_actor.get_transform()

    @staticmethod
    def _on_world_tick(weak_self, timestamp):
        """Callback for CARLA world tick events."""
        self = weak_self()
        if not self:
            return
        self.server_clock.tick()
        self.server_fps = self.server_clock.get_fps()
        self.simulation_time = timestamp.elapsed_seconds

    def _split_actors(self):
        """Split actors by type for rendering."""
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
        """Render traffic lights on the surface."""
        self.affected_traffic_light = None
        for tl in list_tl:
            world_pos = tl.get_location()
            pos = world_to_pixel(world_pos)

            if self.show_triggers:
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
        """Render speed limit signs on the surface."""
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

            if self.show_triggers:
                corners = Util.get_bounding_box(sl)
                corners = [world_to_pixel(p) for p in corners]
                pygame.draw.lines(surface, COLOR_PLUM_2, True, corners, 2)

            surface.blit(font_surface, (x - radius / 2, y - radius / 2))

    def _render_walkers(self, surface, list_w, world_to_pixel):
        """Render pedestrians on the surface."""
        for w in list_w:
            color = COLOR_MAGENTA
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
        """Render vehicles on the surface."""
        for v in list_v:
            color = COLOR_SKY_BLUE_0
            if int(v[0].attributes['number_of_wheels']) == 2:
                color = COLOR_CHOCOLATE_1
            if v[0].attributes.get('role_name') == self.role_name:
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
        """Render all actors on the given surface."""
        self._render_traffic_lights(surface, [tl[0] for tl in traffic_lights], self.map_image.world_to_pixel)
        self._render_speed_limits(surface, [sl[0] for sl in speed_limits], self.map_image.world_to_pixel,
                                  self.map_image.world_to_pixel_width)
        self._render_vehicles(surface, vehicles, self.map_image.world_to_pixel)
        self._render_walkers(surface, walkers, self.map_image.world_to_pixel)

    def _compute_scale(self, scale_factor):
        """Compute and apply map scaling."""
        self.prev_scaled_size = self.scaled_size
        self.map_image.scale_map(scale_factor)

    def render(self):
        """Render the complete visualization frame."""
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

        self.result_surface.fill(COLOR_BLACK)
        self.result_surface.blit(self.map_image.surface, (0, 0))
        self.result_surface.blit(self.actors_surface, (0, 0))
        self.result_surface.blit(self.vehicle_id_surface, (0, 0))

        pan_x = self.mouse_offset[0]
        pan_y = self.mouse_offset[1]

        scaled_map_size = self.map_image.surface.get_size()
        dest_x = (self.dim[0] - scaled_map_size[0]) // 2 + int(pan_x)
        dest_y = (self.dim[1] - scaled_map_size[1]) // 2 + int(pan_y)

        self.display_surface.blit(self.result_surface, (dest_x, dest_y),
                                  area=pygame.Rect(0, 0, scaled_map_size[0], scaled_map_size[1]))

        self._render_hud()

        return self.display_surface

    def _render_hud(self):
        """Render the heads-up display overlay."""
        font = pygame.font.SysFont('monospace', 14)

        hud_surface = pygame.Surface((260, 200))
        hud_surface.fill(COLOR_ALUMINIUM_5)
        hud_surface.set_alpha(180)
        self.display_surface.blit(hud_surface, (0, 0))

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

        # Render legend in bottom-right corner
        self._render_legend(font)

    def _render_legend(self, font):
        """Render the color legend in the bottom-right corner."""
        legend_items = [
            (COLOR_CHAMELEON_0, "Ego Vehicle"),
            (COLOR_SKY_BLUE_0, "Vehicles"),
            (COLOR_CHOCOLATE_1, "Motorcycles"),
            (COLOR_MAGENTA, "Pedestrians"),
        ]

        legend_width = 140
        legend_height = len(legend_items) * 20 + 15
        legend_x = self.dim[0] - legend_width - 10
        legend_y = self.dim[1] - legend_height - 10

        # Draw legend background
        legend_surface = pygame.Surface((legend_width, legend_height))
        legend_surface.fill(COLOR_ALUMINIUM_5)
        legend_surface.set_alpha(180)
        self.display_surface.blit(legend_surface, (legend_x, legend_y))

        # Draw legend items
        y_offset = 8
        for color, label in legend_items:
            # Draw color box
            pygame.draw.rect(
                self.display_surface, color,
                pygame.Rect(legend_x + 8, legend_y + y_offset, 12, 12)
            )
            # Draw label
            text_surface = font.render(label, True, COLOR_ALUMINIUM_0)
            self.display_surface.blit(text_surface, (legend_x + 28, legend_y + y_offset - 2))
            y_offset += 20

    def handle_pan(self, dx, dy):
        """Handle pan input from user interaction."""
        self.mouse_offset[0] += dx
        self.mouse_offset[1] += dy

    def handle_zoom(self, delta, x, y):
        """Handle zoom input from user interaction."""
        old_zoom = self.wheel_offset
        self.wheel_offset += delta * 0.025
        self.wheel_offset = max(0.05, min(2.0, self.wheel_offset))

        map_center_x = self.dim[0] / 2 + self.mouse_offset[0]
        map_center_y = self.dim[1] / 2 + self.mouse_offset[1]

        dx = x - map_center_x
        dy = y - map_center_y

        zoom_ratio = self.wheel_offset / old_zoom
        new_dx = dx * zoom_ratio
        new_dy = dy * zoom_ratio

        self.mouse_offset[0] += dx - new_dx
        self.mouse_offset[1] += dy - new_dy

    def destroy(self):
        """Clean up world resources."""
        if self.world is not None:
            settings = self.world.get_settings()
            settings.no_rendering_mode = False
            self.world.apply_settings(settings)
