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

    @property
    def default_mount(self):
        # Add the custom mesh here
        return {
            "right": {
                "pos": [0, -8.0, 0.0],  # Position relative to robot's base
                "quat": [1.0, 0.0, 0.0, 0.0],  # Orientation
                "custom_mesh": {
                    "file": "/home/guts/alert_auto_dex_rl/playground/meshes/model.stl",
                    "name": "omni_board",
                    "type": "mesh",
                    "scale": [0.1, 0.1, 0.1]  # Adjust size if needed
                }
            }
        }
""

# Create environment
env = suite.make(
    env_name="Door",
    robots="RRLSpot",
    controller_configs=[load_composite_controller_config(controller="WHOLE_BODY_IK"), 
                        load_part_controller_config(default_controller="IK_POSE")],
    has_renderer=True,
    has_offscreen_renderer=False,
    render_camera="agentview",
    use_camera_obs=False,
    control_freq=20,

)

# Run the simulation, and visualize it
env.reset()
mujoco.viewer.launch(env.sim.model._model, env.sim.data._data)

""" while True:
    action = np.random.randn(*env.action_spec[0].shape) * 0.1
    obs, reward, done, info = env.step(action)  # take action in the environment
    print(obs, reward, done, info)
    env.render()  # render on display """

