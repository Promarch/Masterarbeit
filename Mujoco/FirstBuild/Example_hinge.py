import mujoco
import mujoco.viewer

# Path to the Mujoco XML model file
xml = """
<mujoco>
  <worldbody>
    <light name="top" pos="0 0 1"/>
    <body name="box_and_sphere" euler="0 0 -30">
      <joint name="swing" type="hinge" axis="1 -1 0" pos="-.2 -.2 -.2"/>
      <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
      <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""

# Load the Mujoco model
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# Look here for viewer stuff https://mujoco.readthedocs.io/en/stable/python.html

# Launch the viewer
mujoco.viewer.launch(model)
# mujoco.viewer.user_scn.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

