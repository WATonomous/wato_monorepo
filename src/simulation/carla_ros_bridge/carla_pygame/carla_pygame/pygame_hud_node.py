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

import io
import os
import socket
import threading
import time

# Set SDL to use dummy video driver for headless operation
os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_AUDIODRIVER"] = "dummy"

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rcl_interfaces.msg import ParameterDescriptor

from carla_common import connect_carla

try:
    import carla
except ImportError:
    carla = None  # Still needed for dependency check

try:
    import pygame
except ImportError:
    pygame = None

try:
    from flask import Flask, render_template_string, Response, request
except ImportError:
    Flask = None

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
        self.declare_parameter(
            "carla_host", "localhost",
            ParameterDescriptor(description="CARLA server hostname"))
        self.declare_parameter(
            "carla_port", 2000,
            ParameterDescriptor(description="CARLA server port"))
        self.declare_parameter(
            "carla_timeout", 10.0,
            ParameterDescriptor(description="Connection timeout in seconds"))
        self.declare_parameter(
            "web_port", 5000,
            ParameterDescriptor(description="Web server port for map visualization"))
        self.declare_parameter(
            "width", 1280,
            ParameterDescriptor(description="Render width in pixels"))
        self.declare_parameter(
            "height", 720,
            ParameterDescriptor(description="Render height in pixels"))
        self.declare_parameter(
            "role_name", "ego_vehicle",
            ParameterDescriptor(description="Role name of the ego vehicle to track"))
        self.declare_parameter(
            "show_triggers", False,
            ParameterDescriptor(description="Show trigger volumes on map"))
        self.declare_parameter(
            "show_connections", False,
            ParameterDescriptor(description="Show lane connections on map"))
        self.declare_parameter(
            "show_spawn_points", False,
            ParameterDescriptor(description="Show spawn points on map"))
        self.declare_parameter(
            "frame_rate", 30.0,
            ParameterDescriptor(description="Frame rate for web streaming in Hz"))

        # State
        self.carla_client = None
        self.world = None
        self.app = None
        self.server_thread = None
        self.running = False
        self._server = None
        self._server_shutdown_event = threading.Event()
        self._frame_lock = threading.Lock()
        self._current_frame = None

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

        if Flask is None:
            self.get_logger().error("Flask not available")
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
            self.carla_client = connect_carla(host, port, timeout)
            self.get_logger().info(f"Connected to CARLA at {host}:{port}")
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
            show_spawn_points=show_spawn_points,
        )

        if not self.world.start():
            self.get_logger().error("Failed to start world")
            return TransitionCallbackReturn.FAILURE

        # Set up Flask app
        self.app = Flask(__name__)
        world_ref = self.world
        node_ref = self

        @self.app.route("/")
        def index():
            return render_template_string(
                HTML_TEMPLATE, width=world_ref.dim[0], height=world_ref.dim[1]
            )

        @self.app.route("/stream")
        def stream():
            return Response(
                node_ref._generate_frames(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @self.app.route("/pan", methods=["POST"])
        def handle_pan():
            data = request.get_json()
            if world_ref and data:
                world_ref.handle_pan(data.get("dx", 0), data.get("dy", 0))
            return "", 204

        @self.app.route("/zoom", methods=["POST"])
        def handle_zoom():
            data = request.get_json()
            if world_ref and data:
                world_ref.handle_zoom(
                    data.get("delta", 0), data.get("x", 0), data.get("y", 0)
                )
            return "", 204

        self.get_logger().info("Configuration complete")
        return TransitionCallbackReturn.SUCCESS

    def _generate_frames(self):
        """Generator that yields MJPEG frames."""
        frame_rate = self.get_parameter("frame_rate").value
        frame_interval = 1.0 / frame_rate

        while self.running:
            try:
                if self.world and self.world.world:
                    self.world.tick()
                    surface = self.world.render()

                    if surface:
                        data = pygame.image.tostring(surface, "RGB")
                        img = Image.frombytes("RGB", surface.get_size(), data)

                        buffer = io.BytesIO()
                        img.save(buffer, format="JPEG", quality=85)
                        frame_bytes = buffer.getvalue()

                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
                        )

                time.sleep(frame_interval)
            except Exception:
                break

    def _wait_for_port_available(self, port: int, timeout: float = 5.0) -> bool:
        """Wait for a port to become available."""
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    s.bind(("0.0.0.0", port))
                    return True
            except OSError:
                time.sleep(0.1)
        return False

    def _run_server(self, port: int):
        """Run the Flask server with proper shutdown support."""
        from werkzeug.serving import make_server, WSGIRequestHandler

        # Create a custom request handler that doesn't log
        class QuietHandler(WSGIRequestHandler):
            def log_request(self, code="-", size="-"):
                pass  # Suppress request logging

        # Create the WSGI server with SO_REUSEADDR
        self._server = make_server(
            "0.0.0.0",
            port,
            self.app,
            threaded=True,
            request_handler=QuietHandler,
        )
        self._server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self._server_shutdown_event.clear()
        try:
            self._server.serve_forever()
        except Exception:
            pass
        finally:
            # Ensure socket is closed
            try:
                self._server.server_close()
            except Exception:
                pass
            self._server_shutdown_event.set()

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate: Start the web server."""
        self.get_logger().info("Activating...")

        web_port = self.get_parameter("web_port").value
        self.running = True
        self._server = None

        # Wait for port to be available (in case previous server didn't release it)
        if not self._wait_for_port_available(web_port, timeout=3.0):
            self.get_logger().warn(
                f"Port {web_port} may still be in use, attempting to start anyway"
            )

        # Start Flask server in a thread
        self.server_thread = threading.Thread(
            target=self._run_server, args=(web_port,), daemon=True
        )
        self.server_thread.start()

        self.get_logger().info(f"Web server started at http://localhost:{web_port}")
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate: Stop the web server."""
        self.get_logger().info("Deactivating...")

        self.running = False

        # Stop the werkzeug server properly
        if self._server:
            try:
                self._server.shutdown()
            except Exception as e:
                self.get_logger().warn(f"Error shutting down server: {e}")

        # Wait for server thread to finish
        if self.server_thread and self.server_thread.is_alive():
            self._server_shutdown_event.wait(timeout=3.0)
            self.server_thread.join(timeout=2.0)

        self._server = None

        self.get_logger().info("Deactivation complete")
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup: Release resources."""
        self.get_logger().info("Cleaning up...")

        self.world = None

        self.carla_client = None
        self.app = None
        self._server = None
        self.server_thread = None

        pygame.quit()

        self.get_logger().info("Cleanup complete")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown lifecycle callback."""
        self.get_logger().info("Shutting down...")
        return TransitionCallbackReturn.SUCCESS


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
