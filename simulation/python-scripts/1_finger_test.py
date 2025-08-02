# Code to test a simplified version of the MuJoCo hand model. However, was not able to get it running as changing the force applied by the motor had no effect on the motion of the finger. 

import mujoco
import numpy as np
import mujoco.viewer
from mujoco.renderer import Renderer
import mediapy as media
from mujoco.viewer import launch_passive


# xml code of hand with one finger with strings and springs and actuator
XML = """
<mujoco model="finger">
    <compiler angle="degree" coordinate="local"/>
    <option gravity="0 0 -9.81" integrator="RK4"/>

    <asset>
        <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3" rgb2=".2 .3 .4" width="300" height="300"/>
        <material name="grid" texture="grid" texrepeat="8 8" reflectance=".2"/>
    </asset>

    <worldbody>
        <geom size="10 10 .02" type="plane" material="grid" pos="0 0 0"/>
        <light pos="0 0 5"/>
        <camera name="camera" pos="0 -5 7" xyaxes="1 0 0 0 2 2"/>

        <!-- Palm -->
        <body name="palm" pos="0 0 0">
            <geom name="palm" type="cylinder" pos="0 0 0.25" size="1 0.25"/>
            <site name="finger1_str_base" pos="0 0.9 0.5" size="0.02" rgba="1 0 0 1"/>
            <site name="finger1_spr_base" pos="0 0.9 0" size="0.02" rgba="1 0 0 1"/>
        </body>

        <!-- Finger -->
        <body name="finger1_pp" pos="0 1 0.25">
            <joint name="mpj1" type="hinge" axis="1 0 0" damping="1" range="0 90"/>
            <geom name="pp1" type="capsule" fromto="0 0 0 0 1.5 0" size="0.25"/>
            <site name="finger1_str_mid" pos="0 0.75 0.25" size="0.02" rgba="1 0 0 1"/>
            <site name="finger1_spr_mid" pos="0 0.75 -0.25" size="0.02" rgba="1 0 0 1"/>
                <!-- <body name="finger1_mp" pos="0 1.5 0">
                    <joint name="pipj1" type="hinge" axis="1 0 0" damping="1" range="0 90"/>
                    <geom name="mp1" type="capsule" fromto="0 0 0 0 1 0" size="0.25"/>
                    <site name="finger1_str_tip" pos="0 1 0.25" size="0.02" rgba="1 0 0 1"/>
                    <site name="finger1_spr_tip" pos="0 1 -0.25" size="0.02" rgba="1 0 0 1"/>
                </body> -->
        </body>
    </worldbody>

    <tendon>
        <spatial limited="true" range="0 0.6" width="0.005" name="flexor_finger1">
            <site site="finger1_str_base"/>
            <site site="finger1_str_mid"/>
            <!-- <site site="finger1_str_tip"/> -->
        </spatial>
    </tendon>

    <actuator>
        <motor name="motor_finger1" tendon="flexor_finger1" gear="50"/>
    </actuator>
    

    <tendon>
        <spatial limited="true" range="0 0.6" springlength="0.40" stiffness="10" damping="1" width="0.05" name="extendor_finger1">
            <site site="finger1_spr_base"/>
            <site site="finger1_spr_mid"/>
            <!-- <site site="finger1_spr_tip"/> -->
        </spatial>
    </tendon>

</mujoco>"""

# Loading the model & data
model = mujoco.MjModel.from_xml_string(XML)
data  = mujoco.MjData(model)

# Applying ctrl and printing joint angles
for step in range(200):
    # 0 = first (and only) actuator
    data.ctrl[0] = 1.0         # full inward pull
    mujoco.mj_step(model, data)
    angle_deg = np.degrees(data.qpos[0])
    #print(f"Step {step:03d}  time={data.time:.3f}s  angle={angle_deg:.1f}°")

mujoco.mj_forward(model, data)
with mujoco.Renderer(model) as renderer:
  renderer.update_scene(data, camera="camera")

  media.show_image(renderer.render())

  with launch_passive(model, data) as viewer:
    while viewer.is_running():
        # set control and step
        data.ctrl[0] = 1.0    # full inward pull
        mujoco.mj_step(model, data)
        viewer.sync()
