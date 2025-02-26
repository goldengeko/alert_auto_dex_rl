from robosuite.models import MujocoWorldBase
from robosuite.models.robots import Panda
from robosuite.models.grippers import gripper_factory
from robosuite.models.arenas import TableArena
from robosuite.models.objects import OmniboardObject
from robosuite.utils.mjcf_utils import new_joint
import mujoco
import os

## Create Mujoco world and robot
world = MujocoWorldBase()
mujoco_robot = Panda()

# Add gripper to robot
gripper = gripper_factory('PandaGripper')
mujoco_robot.add_gripper(gripper)
mujoco_robot.set_base_xpos([0, 0, 0])
world.merge(mujoco_robot)

# Add OmniBoard object
omniboard = OmniboardObject(name="omniboard")
omniboard.set_pos([0, 0, 0])
world.merge(omniboard)

# Debugging: Print the state of omniboard._obj
print(f"omniboard._obj: {omniboard._obj}")

# Ensure the object is not None before accessing its attributes
if omniboard._obj is not None:
    omniboard._obj.attrib["name"] = "main"
else:
    print("Error: Omniboard object is None")

# Get Mujoco model and data
model = mujoco.MjModel.from_xml_string(world.get_model(mode="mujoco"))
data = mujoco.MjData(model)

# Launch viewer and run simulation
mujoco.viewer.launch(model, data)
while True:
    mujoco.mj_step(model, data)