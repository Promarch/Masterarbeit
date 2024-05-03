import mujoco
import mujoco.viewer

path_robot = "Mujoco/Franka/franka_emika_panda/scene.xml"
model = mujoco.MjModel.from_xml_path(path_robot)
data = mujoco.MjData(model)

mujoco.viewer.launch(model)

