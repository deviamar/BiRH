# Code to simply view the scene.xml file
import mujoco #type: ignore
from mujoco import viewer #type: ignore

# Load the scene
model = mujoco.MjModel.from_xml_path("/Users/stellaluna/Documents/GitHub/BiRH/simulation/mujoco-models/scene.xml")
data = mujoco.MjData(model)

# Launch viewer
with viewer.launch_passive(model, data) as v:
    # Keep the simulation running
    while v.is_running():
        mujoco.mj_step(model, data)
        v.sync()


# Or simply run "python -m mujoco.viewer" on terminal to get blank viewer shell