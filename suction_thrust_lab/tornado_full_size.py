"""Full-size tornado race using the existing experimental seven-jet hardware."""
import mujoco

from .full_size import full_size_config
from .tornado import TornadoSimulation, demo_config


def tornado_full_size_config():
    c = full_size_config()
    c['tornado'] = demo_config()['tornado'] | dict(
        top_speed=60., opponent_speed=30., race_scale=16.,
        cruise_duration=.35, lift_duration=.15, lift_acceleration=35.,
        roll_duration=3., turns=6, roll_ramp=.5,
        roll_height=1.2, roll_arch=3.3, descend_duration=.35,
        touchdown_limit=.5, drag_coefficient=.4)
    c['flight']['attitude_response_time'] = 1.
    return c


class FullSizeTornadoSimulation(TornadoSimulation):
    def __init__(self, config=None):
        super().__init__(config or tornado_full_size_config())
        # Rolling race start: all traffic is already at speed. This lets the
        # overtaking sequence use the same three seconds as the miniature demo.
        self.data.qvel[0] = self.settings['top_speed']
        self.data.qvel[self.wheel_dofs] = self.settings['top_speed']/self.wheel_radius
        mujoco.mj_forward(self.model, self.data)
