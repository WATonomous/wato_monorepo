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
"""Web-based pygame HUD lifecycle node for CARLA visualization."""

import base64
import io
import os
import threading

# Set SDL to use dummy video driver for headless operation
os.environ['SDL_VIDEODRIVER'] = 'dummy'
os.environ['SDL_AUDIODRIVER'] = 'dummy'

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

try:
    import carla
except ImportError:
    carla = None

try:
    import pygame
except ImportError:
    pygame = None

try:
    from flask import Flask, render_template_string
    from flask_socketio import SocketIO
except ImportError:
    Flask = None
    SocketIO = None

try:
    from PIL import Image
except ImportError:
    Image = None

from carla_pygame.web_template import HTML_TEMPLATE
from carla_pygame.world import World


class PygameHudNode(LifecycleNode):
    """Lifecycle node for web-based pygame HUD visualization."""

    def __init__(self, node_name="carla_pygame"):
        super().__init__(node_name)

        # Parameters
        self.declare_parameter("carla_host", "localhost")
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("carla_timeout", 10.0)
        self.declare_parameter("web_port", 5000)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("role_name", "ego_vehicle")
        self.declare_parameter("show_triggers", False)
        self.declare_parameter("show_connections", False)
        self.declare_parameter("show_spawn_points", False)

        # State
        self.carla_client = None
        self.world = None
        self.app = None
        self.socketio = None
        self.server_thread = None
        self.emitter_thread = None
        self.running = False

        self.get_logger().info(f"{node_name} initialized")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure: Connect to CARLA and initialize pygame."""
        self.get_logger().info("Configuring...")

        # Check dependencies
        if carla is None:
            self.get_logger().error("CARLA Python API not available")
            return TransitionCallbackReturn.FAILURE

        if pygame is None:
            self.get_logger().error("pygame not available")
            return TransitionCallbackReturn.FAILURE

        if Flask is None or SocketIO is None:
            self.get_logger().error("Flask/Flask-SocketIO not available")
            return TransitionCallbackReturn.FAILURE

        if Image is None:
            self.get_logger().error("PIL/Pillow not available")
            return TransitionCallbackReturn.FAILURE

        # Get parameters
        host = self.get_parameter("carla_host").value
        port = self.get_parameter("carla_port").value
        timeout = self.get_parameter("carla_timeout").value
        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        role_name = self.get_parameter("role_name").value
        show_triggers = self.get_parameter("show_triggers").value
        show_connections = self.get_parameter("show_connections").value
        show_spawn_points = self.get_parameter("show_spawn_points").value

        # Initialize pygame
        pygame.init()
        pygame.font.init()
        pygame.display.set_mode((1, 1))

        # Connect to CARLA
        try:
            self.carla_client = carla.Client(host, port)
            self.carla_client.set_timeout(timeout)
            version = self.carla_client.get_server_version()
            self.get_logger().info(f"Connected to CARLA {version} at {host}:{port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {e}")
            return TransitionCallbackReturn.FAILURE

        # Create World object
        self.world = World(
            self.carla_client,
            width,
            height,
            role_name=role_name,
            show_triggers=show_triggers,
            show_connections=show_connections,
            show_spawn_points=show_spawn_points
        )

        if not self.world.start():
            self.get_logger().error("Failed to start world")
            return TransitionCallbackReturn.FAILURE

        # Set up Flask app
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'carla-visualizer'
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", async_mode='threading')

        # Set up routes
        world_ref = self.world

        @self.app.route('/')
        def index():
            return render_template_string(HTML_TEMPLATE, width=world_ref.dim[0], height=world_ref.dim[1])

        @self.socketio.on('pan')
        def handle_pan(data):
            if world_ref:
                world_ref.handle_pan(data['dx'], data['dy'])

        @self.socketio.on('zoom')
        def handle_zoom(data):
            if world_ref:
                world_ref.handle_zoom(data['delta'], data['x'], data['y'])

        self.get_logger().info("Configuration complete")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate: Start the web server and frame emitter."""
        self.get_logger().info("Activating...")

        web_port = self.get_parameter("web_port").value
        self.running = True

        # Start frame emitter thread
        self.emitter_thread = threading.Thread(target=self._frame_emitter, daemon=True)
        self.emitter_thread.start()

        # Start Flask server in a thread
        self.server_thread = threading.Thread(
            target=lambda: self.socketio.run(
                self.app,
                host='0.0.0.0',
                port=web_port,
                debug=False,
                use_reloader=False,
                allow_unsafe_werkzeug=True
            ),
            daemon=True
        )
        self.server_thread.start()

        self.get_logger().info(f"Web server started at http://localhost:{web_port}")
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate: Stop the web server and frame emitter."""
        self.get_logger().info("Deactivating...")

        self.running = False

        # Stop the emitter thread
        if self.emitter_thread and self.emitter_thread.is_alive():
            self.emitter_thread.join(timeout=2.0)

        # Stop Flask-SocketIO server
        if self.socketio:
            try:
                self.socketio.stop()
            except Exception:
                pass

        self.get_logger().info("Deactivation complete")
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup: Release resources."""
        self.get_logger().info("Cleaning up...")

        if self.world:
            self.world.destroy()
            self.world = None

        self.carla_client = None
        self.app = None
        self.socketio = None

        pygame.quit()

        self.get_logger().info("Cleanup complete")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown lifecycle callback."""
        self.get_logger().info("Shutting down...")
        return TransitionCallbackReturn.SUCCESS

    def _frame_emitter(self):
        """Background thread that emits frames to connected clients."""
        clock = pygame.time.Clock()

        while self.running:
            try:
                if self.world and self.world.world:
                    self.world.tick()
                    surface = self.world.render()

                    if surface:
                        data = pygame.image.tostring(surface, 'RGB')
                        img = Image.frombytes('RGB', surface.get_size(), data)

                        buffer = io.BytesIO()
                        img.save(buffer, format='PNG', optimize=False)
                        frame_data = base64.b64encode(buffer.getvalue()).decode('utf-8')

                        self.socketio.emit('frame', frame_data)
            except Exception as e:
                self.get_logger().error(f"Frame emitter error: {e}")

            clock.tick(30)


def main(args=None):
    rclpy.init(args=args)
    node = PygameHudNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
