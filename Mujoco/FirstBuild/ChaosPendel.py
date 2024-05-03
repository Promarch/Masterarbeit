import time

import mujoco
import mujoco.viewer


Chaospendel = """
<mujoco>
  <option timestep=".001">
    <flag energy="enable" contact="disable"/>
  </option>

  <default>
    <joint type="hinge" axis="0 -1 0"/>
    <geom type="capsule" size=".02"/>
  </default>

  <worldbody>
    
    <light pos="0 -.4 1"/>
    <camera name="fixed" pos="0 -1 0" xyaxes="1 0 0 0 0 1"/>
    <body name="0" pos="0 0 0">
      <joint name="root"/>
      <geom fromto="0 0 0 0 0 -.2" rgba="1 1 0 1"/>
      <body name="1" pos="0 0 -.2">
        <joint/>
        <geom fromto="0 0 0 .05 0 -.2" rgba="1 0 0 1"/>
        <body name="2" pos=".05 0 -.2">
          <joint/>
          <geom fromto="0 0 0 0 0 -.2" rgba="1 0 1 1"/>
      </body>
      </body>
    </body>

  </worldbody>

</mujoco>
"""

m = mujoco.MjModel.from_xml_string(Chaospendel)
d = mujoco.MjData(m)

mujoco.viewer.launch(m)
