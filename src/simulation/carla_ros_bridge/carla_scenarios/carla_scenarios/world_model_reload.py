"""Reload an optional, independently launched world_model lifecycle node."""

import threading
import time

from lanelet_msgs.srv import GetLaneletAhead
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter as ParameterMessage


def call(client, request, timeout=30.0):
    if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
        raise RuntimeError(f"service unavailable: {client.srv_name}")
    future = client.call_async(request)
    completed = threading.Event()
    future.add_done_callback(lambda _: completed.set())
    if not completed.wait(timeout):
        client.remove_pending_request(future)
        raise RuntimeError(f"service timed out: {client.srv_name}")
    if future.exception() is not None:
        raise future.exception()
    return future.result()


class WorldModelReload:
    def __init__(self, node, name, callback_group):
        self.state = node.create_client(GetState, name + "/get_state", callback_group=callback_group)
        self.transition = node.create_client(ChangeState, name + "/change_state", callback_group=callback_group)
        self.parameters = node.create_client(SetParameters, name + "/set_parameters", callback_group=callback_group)
        self.get_parameters = node.create_client(GetParameters, name + "/get_parameters", callback_group=callback_group)
        namespace = name.rsplit("/", 1)[0]
        self.query = node.create_client(GetLaneletAhead, namespace + "/get_lanelet_ahead", callback_group=callback_group)
        self.was_active = False
        self.present = False
        self.applied = None
        self.pending = False
        self.clients = [self.state, self.transition, self.parameters, self.get_parameters, self.query]

    @staticmethod
    def values(bundle):
        return {"osm_map_path": str(bundle.osm_path), "projector_type": "local_cartesian",
                "origin_lat": float(bundle.origin["lat"]), "origin_lon": float(bundle.origin["lon"]),
                "use_sim_time": True}

    def needs_reload(self, bundle, generation):
        if not self.state.service_is_ready():
            self.applied = None
            return False
        state = call(self.state, GetState.Request()).current_state.id
        # Independently launched lifecycle managers must finish startup first.
        if state not in (1, 2, 3):
            return False
        if self.applied != generation or state == 1:
            return True
        expected = self.values(bundle)
        request = GetParameters.Request()
        request.names = list(expected)
        response = call(self.get_parameters, request)
        actual = [Parameter.from_parameter_msg(ParameterMessage(name=name, value=value)).value
                  for name, value in zip(request.names, response.values)]
        return actual != list(expected.values())

    def change(self, transition):
        request = ChangeState.Request()
        request.transition.id = transition
        if not call(self.transition, request).success:
            raise RuntimeError(f"world_model lifecycle transition {transition} failed")

    def prepare(self):
        self.present = self.state.service_is_ready()
        if not self.present:
            return
        deadline = time.monotonic() + 30.0
        state = call(self.state, GetState.Request()).current_state.id
        while state not in (1, 2, 3) and time.monotonic() < deadline:
            time.sleep(0.1)
            state = call(self.state, GetState.Request()).current_state.id
        if state not in (1, 2, 3):
            raise RuntimeError(f"world_model is transitioning (state {state}); try again")
        if not self.pending:
            self.was_active = state == 3
        self.pending = True
        if state == 3:
            self.change(4)  # deactivate
        if state in (2, 3):
            self.change(2)  # cleanup

    def apply(self, bundle, pose, generation):
        if not self.present:
            return
        request = SetParameters.Request()
        request.parameters = [Parameter(name, value=value).to_parameter_msg()
                              for name, value in self.values(bundle).items()]
        results = call(self.parameters, request).results
        if len(results) != len(request.parameters) or any(not result.successful for result in results):
            raise RuntimeError("world_model rejected map parameters: " + "; ".join(r.reason for r in results))
        self.change(1)  # configure
        if self.was_active:
            self.change(3)  # activate
            request = GetLaneletAhead.Request()
            request.position.x, request.position.y = pose[0], pose[1]
            request.heading_rad = pose[2]
            request.radius_m = 5.0
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline:
                if self.query.service_is_ready():
                    response = call(self.query, request, timeout=2.0)
                    if response.success:
                        self.applied = generation
                        self.pending = False
                        return
                    if response.error_message != "map_not_loaded":
                        raise RuntimeError(f"world_model map alignment/query failed: {response.error_message}")
                time.sleep(0.1)
            raise RuntimeError("world_model did not load the selected Lanelet2 map")
        self.applied = generation
        self.pending = False
