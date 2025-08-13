# Code to change the design of the hand (position of the fingers)

import sys
import Math

a1 = sys.argv[1]
a2 = sys.argv[2]
a3 = sys.argv[3]

print(f"Three integers representing the position of the fingers as angle about the center of the palm: {a1}, {a2}, {a3}")

pp_len = .075
mp_len = .05

def fromto(angle, len):
  str = "0 0 0 "
  str += (Math.cos(angle) * len) + " "
  str += (Math.cos(angle) * len) + " "
  str += "0"
  return 1

def main():
  pos = []
  pos.append(fromto(a1, pp_len))
  pos.append(fromto(a1, mp_len))
  pos.append(fromto(a2, pp_len))
  pos.append(fromto(a2, mp_len))
  pos.append(fromto(a3, pp_len))
  pos.append(fromto(a3, mp_len))

  xml = """
<!--  Hand  -->
<body name="hand" pos="0 0.12 0" quat="0.707 -0.707 0 0">
  <!--<joint name="wrist" type="hinge" damping="100" axis="1 0 0"/>-->
  <geom name="palm" type="cylinder" pos="0 0 0" size="0.05 0.0125"/>
  <geom name="palm_pad" type="cylinder" pos="0 0 0.013" size="0.05 0.001"
        margin="0.003" solref="0.0005 1.6" solimp="0.99 0.999 0.0003 0.5 2"
        friction="1.2 0.01 0.0005" rgba="0.2 0.2 0.2 0.8" mass="0.005"/>

  <!--  Finger 1  -->
  <body name="finger1_pp" pos="0 .05 0">
    <joint name="mpj1" type="hinge" axis="1 0 0" damping="5" range="0 1.571"/>
    <geom name="pp1" type="capsule" fromto="{pos[0]}" size="0.0125" mass=".1"/>
    <geom name="pp1_pad" type="box" pos="0 0.0375 0.013" size="0.0125 0.0375 0.001"
          margin="0.003" solref="0.0005 1.6" solimp="0.99 0.999 0.0003 0.5 2"
          friction="1.2 0.01 0.0005" rgba="0.2 0.2 0.2 0.8" mass="0.005"/>
      <body name="finger1_mp" pos="0 .075 0">
        <joint name="pipj1" type="hinge" axis="1 0 0" damping="5" range="0 1.571"/>
        <geom name="mp1" type="capsule" fromto="{pos[1]}" size="0.0125" mass=".1"/>
          <geom name="mp1_pad" type="box" pos="0 0.025 0.013" size="0.0125 0.025 0.001"
            margin="0.003" solref="0.0005 1.6" solimp="0.99 0.999 0.0003 0.5 2"
            friction="1.2 0.01 0.0005" rgba="0.2 0.2 0.2 0.8" mass="0.005"/>
      </body>
  </body>

  <!--  Finger 2  -->
  <body name="finger2_pp" pos="-0.03535 0.03535 0">
    <joint name="mpj2" type="hinge" axis="1 1 0" damping="5" range="0 1.571"/>
    <geom name="pp2" type="capsule" fromto="{pos[2]}" size="0.0125" mass=".1"/>
    <geom name="pp2_pad" type="box" pos="-0.025 0.025 0.013" size="0.0125 0.0375 0.001"
          margin="0.003" solref="0.0005 1.6" solimp="0.99 0.999 0.0003 0.5 2"
          friction="1.2 0.01 0.0005" rgba="0.2 0.2 0.2 0.8" mass="0.005" quat="0.927 0 0 0.375"/>
      <body name="finger2_mp" pos="-.053 .053 0">
        <joint name="pipj2" type="hinge" axis="1 1 0" damping="5" range="0 1.571"/>
        <geom name="mp2" type="capsule" fromto="{pos[3]}" size="0.0125" mass=".1"/>
          <geom name="mp2_pad" type="box" pos="-0.0155 0.0155 0.013" size="0.0125 0.025 0.001"
            margin="0.003" solref="0.0005 1.6" solimp="0.99 0.999 0.0003 0.5 2"
            friction="1.2 0.01 0.0005" rgba="0.2 0.2 0.2 0.8" mass="0.005" quat="0.927 0 0 0.375"/>
      </body>
  </body>

  <!-- Finger 3 -->
  <body name="finger3_pp" pos="0.03535 -0.03535 0">
    <joint name="mpj3" type="hinge" axis="-1 -1 0" damping="5" range="0 1.571"/>
    <geom name="pp3" type="capsule" fromto="{pos[4]}" size="0.0125" mass=".1"/>
    <geom name="pp3_pad" type="box" pos="0.025 -0.025 0.013" size="0.0125 0.0375 0.001"
          margin="0.003" solref="0.0005 1.6" solimp="0.99 0.999 0.0003 0.5 2"
          friction="1.2 0.01 0.0005" rgba="0.2 0.2 0.2 0.8" mass="0.005" quat="0.927 0 0 0.375"/>
      <body name="finger3_mp" pos=".053 -.053 0">
        <joint name="pipj3" type="hinge" axis="-1 -1 0" damping="5" range="0 1.571"/>
        <geom name="mp3" type="capsule" fromto="{pos[5]}" size="0.0125" mass=".1"/>
        <geom name="mp3_pad" type="box" pos="0.0155 -0.0155 0.013" size="0.0125 0.025 0.001"
            margin="0.003" solref="0.0005 1.6" solimp="0.99 0.999 0.0003 0.5 2"
            friction="1.2 0.01 0.0005" rgba="0.2 0.2 0.2 0.8" mass="0.005" quat="0.927 0 0 0.375"/>
      </body>
  </body>

  
</body>
"""
  print(xml)