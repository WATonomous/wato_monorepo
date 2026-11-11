"""Actual published SDK against a transport fixture. No simulator fidelity claim."""
import numpy as np
from simforge_gym import SimForgeEnv


class TransportFixture:
    def __init__(self):
        self.last_action = None
        self.closed = False
    def hello(self):
        return {"actionFields": ["target_speed_mps", "target_acceleration_mps2", "throttle", "brake", "steer"],
                "actionWidth": 5, "ego": "ego", "decisionHz": 10,
                "maxObjects": 1, "objectFeatures": 5, "stateVectorSize": 4}
    def response(self):
        state = np.array([1, 2, 3, 4], dtype="<f8").tobytes()
        objects = np.zeros((1, 5), dtype="<f4").tobytes()
        return {"observation": {
                    "state_vector": {"offset": 0, "length": 32, "dtype": "<f8", "shape": [4]},
                    "objects": {"offset": 32, "length": 20, "dtype": "<f4", "shape": [1, 5]}},
                "info": {"t_s": 1, "ego": {}, "object_ids": [], "reward_terms": {}, "events": []},
                "reward": 0, "terminated": False, "truncated": False}, state+objects
    def reset(self, seed=None, *, scenario=None):
        return self.response()
    def step(self, row):
        self.last_action = row
        return self.response()
    def close(self):
        self.closed = True


def test_published_sdk_applies_controls_by_named_slots_and_exposes_events():
    transport = TransportFixture()
    env = SimForgeEnv(transport, action_mode="control", observation="state", info_channel=True)
    obs, info = env.reset(seed=7)
    assert obs["state_vector"].tolist() == [1, 2, 3, 4]
    assert info["events"] == []
    env.step([0.2, 0.0, -0.5])
    assert np.isnan(transport.last_action[:2]).all()
    assert transport.last_action[2:].tolist() == [0.2, 0, -0.5]
    env.close()
    assert transport.closed
