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

    def load_model(self):
        # Call parent load method
        super().load_model()

        # Load Omniboard object and attach it to the environment
        self.omniboard = MujocoXMLObject(fname="/home/guts/alert_auto_dex_rl/omni-board.xml")

        # Set object position and orientation (mounted on the wall)
        self.omniboard.set("pos", "0 -0.8 1.2")  # (x, y, z) - Adjust for wall placement
        self.omniboard.set("quat", "1 0 0 0")
        
        # Merge object into the model
        self.model.merge_assets(self.omniboard)
        self.model.worldbody.append(self.omniboard.get_model())

# Create environment
env = suite.make(
    env_name="customenv",
    robots="RRLSpot",
    controller_configs=[
        load_composite_controller_config(controller="WHOLE_BODY_IK"),
        load_part_controller_config(default_controller="IK_POSE")
    ],
    has_renderer=True,
    has_offscreen_renderer=False,
    render_camera="agentview",
    use_camera_obs=False,
    control_freq=20,
)

# Run the simulation
env.reset()
# Main simulation loop
while True:
    action = np.random.randn(*env.action_spec[0].shape) * 0.1
    obs, reward, done, info = env.step(action)  # take random actions
    env.render()  # Render simulation in real-time