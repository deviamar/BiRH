# Code to change the design of the hand (position of the fingers)

import sys
import math
import mujoco
from mujoco import viewer

# Will need to change the orientation of the pads using quaternion - about the x - first and last number rotation
# Euler angles to quaternion, remember pads are drawn along the x axis (longer length) and will need to adjust for final angle

if len(sys.argv) != 4:
    print("Please enter three integers representing the position of the fingers as angle about the center of the palm")
    print("Usage: python hand_design.py <angle1> <angle2> <angle3>")
    sys.exit(1)

a1 = math.radians(float(sys.argv[1]))
a2 = math.radians(float(sys.argv[2]))
a3 = math.radians(float(sys.argv[3]))

palm_r = 0.05
pp_len = .075   # proximal phalange length
mp_len = .05    # middle phalange length

def xyz(angle: float, len: float, z) -> str:
  x = (math.cos(angle) * len)
  y = (math.sin(angle) * len)
  return f"{x:.3f} {y:.6f} {z}"

pos = []
pos.append(xyz(a1, palm_r, 0))
pos.append(xyz(a1, pp_len, 0))
pos.append(xyz(a2, palm_r, 0))
pos.append(xyz(a2, pp_len, 0))
pos.append(xyz(a3, palm_r, 0))
pos.append(xyz(a3, pp_len, 0))


ft = []
ft.append("0 0 0 " + xyz(a1, pp_len, 0))
ft.append("0 0 0 " + xyz(a1, mp_len, 0))
ft.append("0 0 0 " + xyz(a2, pp_len, 0))
ft.append("0 0 0 " + xyz(a2, mp_len, 0))
ft.append("0 0 0 " + xyz(a3, pp_len, 0))
ft.append("0 0 0 " + xyz(a3, mp_len, 0))

pad_pos = []
pad_pos.append(xyz(a1, pp_len/2, .013))
pad_pos.append(xyz(a1, mp_len/2, .013))
pad_pos.append(xyz(a2, pp_len/2, .013))
pad_pos.append(xyz(a2, mp_len/2, .013))
pad_pos.append(xyz(a3, pp_len/2, .013))
pad_pos.append(xyz(a3, mp_len/2, .013))

def euler_to_quat(roll, pitch, yaw):
    roll  = math.radians(roll)
    pitch = math.radians(pitch)
    yaw   = math.radians(yaw)

    cr, sr = math.cos(roll/2),  math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2),   math.sin(yaw/2)

    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy

    # normalize (numerical safety)
    n = math.sqrt(w*w + x*x + y*y + z*z)
    return (w/n, x/n, y/n, z/n)

def axes(angle: float, len: float, z) -> str:
  x = (math.cos(angle) * len)
  y = (math.sin(angle) * len)
  return f"{y:.3f} -{x:.6f} {z}"

axis = []
axis.append(axes(a1, palm_r, 0))
axis.append(xyz(a2, palm_r, 0))
axis.append(xyz(a3, palm_r, 0))

def euler_to_quat(roll, pitch, yaw, degrees=False):
    if degrees:
        roll  = math.radians(roll)
        pitch = math.radians(pitch)
        yaw   = math.radians(yaw)

    cr, sr = math.cos(roll/2),  math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2),   math.sin(yaw/2)

    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    w = cr*cp*cy + sr*sp*sy

    # normalize (numerical safety)
    n = math.sqrt(x*x + y*y + z*z + w*w)
    return f"{x/n} {y/n} {z/n} {w/n}"

#need to revise
quat = []
quat.append(euler_to_quat(a1, 0, 0, True))
quat.append(euler_to_quat(60, 0, 0, True))
quat.append(euler_to_quat(-60, 0, 0, True))

xml = f"""<mujoco model="hand_ur10e">

   <worldbody>
    <light pos="0 -1 2"/>
    <!--  Hand  -->
    <body name="hand" pos="0 0.12 0" quat="0.707 -0.707 0 0">
      <!--<joint name="wrist" type="hinge" damping="100" axis="1 0 0"/>-->
      <geom name="palm" type="cylinder" pos="0 0 0" size="0.05 0.0125"/>
      <geom name="palm_pad" type="cylinder" pos="0 0 0.013" size="0.05 0.001"
            margin="0.003" solref="0.0005 1.6" solimp="0.99 0.999 0.0003 0.5 2"
            friction="1.2 0.01 0.0005" rgba="0.2 0.2 0.2 0.8" mass="0.005"/>

      <!--  Finger 1  -->
      <body name="finger1_pp" pos="{pos[0]}">
        <joint name="mpj1" type="hinge" axis="{axis[0]}" damping="5" range="0 1.571"/>
        <geom name="pp1" type="capsule" fromto="{ft[0]}" size="0.0125" mass=".1"/>
        <geom name="pp1_pad" type="box" pos="{pad_pos[0]}" size="0.0375 0.0125 0.001" quat="{quat[0]}"
              margin="0.003" solref="0.0005 1.6" solimp="0.99 0.999 0.0003 0.5 2"
              friction="1.2 0.01 0.0005" rgba="0.2 0.2 0.2 0.8" mass="0.005"/>
          <body name="finger1_mp" pos="{pos[1]}">
            <joint name="pipj1" type="hinge" axis="{axis[0]}" damping="5" range="0 1.571"/>
            <geom name="mp1" type="capsule" fromto="{ft[1]}" size="0.0125" mass=".1"/> quat="{quat[0]}"
              <geom name="mp1_pad" type="box" pos="{pad_pos[1]}" size="0.025 0.0125 0.001"
                margin="0.003" solref="0.0005 1.6" solimp="0.99 0.999 0.0003 0.5 2"
                friction="1.2 0.01 0.0005" rgba="0.2 0.2 0.2 0.8" mass="0.005"/>
          </body>
      </body>

      <!--  Finger 2  -->
      <body name="finger2_pp" pos="{pos[2]}">
        <joint name="mpj2" type="hinge" axis="{axis[1]}" damping="5" range="0 1.571"/>
        <geom name="pp2" type="capsule" fromto="{ft[2]}" size="0.0125" mass=".1"/>
        <geom name="pp2_pad" type="box" pos="{pad_pos[2]}" size="0.0375 0.0125 0.001" quat="{quat[1]}"
              margin="0.003" solref="0.0005 1.6" solimp="0.99 0.999 0.0003 0.5 2"
              friction="1.2 0.01 0.0005" rgba="0.2 0.2 0.2 0.8" mass="0.005"/>
          <body name="finger2_mp" pos="{pos[3]}">
            <joint name="pipj2" type="hinge" axis="{axis[1]}" damping="5" range="0 1.571"/>
            <geom name="mp2" type="capsule" fromto="{ft[3]}" size="0.0125" mass=".1"/>
              <geom name="mp2_pad" type="box" pos="{pad_pos[3]}" size="0.025 0.0125 0.001" quat="{quat[1]}"
                margin="0.003" solref="0.0005 1.6" solimp="0.99 0.999 0.0003 0.5 2"
                friction="1.2 0.01 0.0005" rgba="0.2 0.2 0.2 0.8" mass="0.005"/>
          </body>
      </body>

      <!-- Finger 3 -->
      <body name="finger3_pp" pos="{pos[4]}">
        <joint name="mpj3" type="hinge" axis="{axis[2]}" damping="5" range="0 1.571"/>
        <geom name="pp3" type="capsule" fromto="{ft[4]}" size="0.0125" mass=".1"/>
        <geom name="pp3_pad" type="box" pos="{pad_pos[4]}" size="0.0375 0.0125 0.001" quat="{quat[2]}"
              margin="0.003" solref="0.0005 1.6" solimp="0.99 0.999 0.0003 0.5 2"
              friction="1.2 0.01 0.0005" rgba="0.2 0.2 0.2 0.8" mass="0.005"/>
          <body name="finger3_mp" pos="{pos[5]}">
            <joint name="pipj3" type="hinge" axis="{axis[2]}" damping="5" range="0 1.571"/>
            <geom name="mp3" type="capsule" fromto="{ft[5]}" size="0.0125" mass=".1"/>
            <geom name="mp3_pad" type="box" pos="{pad_pos[5]}" size="0.025 0.0125 0.001" quat="{quat[2]}"
                margin="0.003" solref="0.0005 1.6" solimp="0.99 0.999 0.0003 0.5 2"
                friction="1.2 0.01 0.0005" rgba="0.2 0.2 0.2 0.8" mass="0.005"/>
          </body>
      </body>

      
    </body>

  </worldbody>
  
</mujoco>
"""
print(xml)

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

with viewer.launch_passive(model, data) as v:
    while v.is_running():
        mujoco.mj_step(model, data)
        v.sync()