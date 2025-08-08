# Simple code to control the mp joint motors in each of the three fingers

import mujoco # type: ignore
import numpy as np
from mujoco import viewer # type: ignore
import time

# Load model and data
model = mujoco.MjModel.from_xml_path("/Users/stellaluna/Documents/GitHub/BiRH/simulation/mujoco-models/hand.xml")
data = mujoco.MjData(model)

# Map actuator names to IDs
actuator_ids = {
    "mpj1_motor": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "mpj1_motor"),
    "mpj2_motor": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "mpj2_motor"),
    "mpjt_motor": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "mpjt_motor"),
}

# Set target angles
data.ctrl[actuator_ids["mpj1_motor"]] = 45
data.ctrl[actuator_ids["mpj2_motor"]] = 30
data.ctrl[actuator_ids["mpjt_motor"]] = -90

# Launch viewer and run simulation
with viewer.launch_passive(model, data) as v:
    for _ in range(20):  # Run for a while to see the result
        mujoco.mj_step(model, data)
        v.sync()
        time.sleep(0.1)
    time.sleep(10)