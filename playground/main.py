import numpy as np
from playground.customenv import OmniboardEnv
from robosuite.controllers import load_controller_config
from robosuite.robots import register_robot_class
from robosuite.models.robots import Kinova3
import robosuite as suite
from robosuite.models.objects import MujocoXMLObject
from robosuite.controllers import load_composite_controller_config, load_part_controller_config
import numpy as np
import mujoco

@register_robot_class("LeggedRobot")
class RRLSpot(Kinova3):
    @property
    def default_base(self):
        return "Spot"

    @property
    def default_arms(self):
        return {"right": "Kinova3"}

    @property
    def default_gripper(self):
        return {"right": "Robotiq140Gripper"}
# Load controller configuration (e.g., IK)
controller_config = load_controller_config(default_controller="IK_POSE")

# Initialize custom environment
env = OmniboardEnv(
    robots="Kinova3",
    controller_config=controller_config,
    has_renderer=True,  # Render in real-time
    has_offscreen_renderer=False,
    render_camera="frontview",
    use_camera_obs=False,
    control_freq=20,
)

# Run the simulation
env.reset()

# Simple control loop (random actions for demonstration)
while True:
    action = np.random.randn(env.robots[0].dof) * 0.1  # Random action
    obs, reward, done, info = env.step(action)
    env.render()  # Real-time visualization
