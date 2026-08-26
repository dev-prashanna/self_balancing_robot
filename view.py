"""Quick MuJoCo viewer for robot.xml."""
import mujoco
from mujoco.viewer import launch
import os

xml = os.path.join(os.path.dirname(__file__), "robot.xml")
model = mujoco.MjModel.from_xml_path(xml)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
launch(model, data)
