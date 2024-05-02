import mujoco_py
import mujoco
import os

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

# Create a simulation environment
sim = mujoco_py.MjSim(model)

# Run simulation steps
for _ in range(1000):
    sim.step()

# Close the simulation environment
sim.close()
