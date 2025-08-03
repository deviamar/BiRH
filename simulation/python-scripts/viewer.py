import mujoco #type: ignore
from mujoco import viewer #type: ignore

# Load the scene
model = mujoco.MjModel.from_xml_path("/Users/stellaluna/Documents/GitHub/BiRH/simulation/mujoco-models/hand_ur10e.xml")
data = mujoco.MjData(model)

# Launch viewer - this is the easiest and fastest way
with viewer.launch_passive(model, data) as v:
    print("Scene loaded! Use mouse to navigate:")
    print("- Left click + drag: rotate view")
    print("- Right click + drag: zoom")
    print("- Middle click + drag: pan")
    print("- Press ESC or close window to exit")
    
    # Keep the simulation running
    while v.is_running():
        mujoco.mj_step(model, data)
        v.sync()