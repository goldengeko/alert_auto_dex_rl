from robosuite.robots import register_robot_class
from robosuite.models.robots import Kinova3
import robosuite as suite
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

# Create environment
env = suite.make(
    env_name="Door",
    robots="RRLSpot",
    controller_configs=[load_composite_controller_config(controller="HYBRID_MOBILE_BASE"), 
                        load_part_controller_config(default_controller="JOINT_POSITION")],
    has_renderer=True,
    has_offscreen_renderer=False,
    render_camera="agentview",
    use_camera_obs=False,
    control_freq=20,
)

# Run the simulation, and visualize it
env.reset()
mujoco.viewer.launch(env.sim.model._model, env.sim.data._data)

""" for i in range(1000):
    action = np.random.randn(*env.action_spec[0].shape) * 0.1
    obs, reward, done, info = env.step(action)  # take action in the environment
    print(i, obs, reward, done, info)
    env.render()  # render on display """

