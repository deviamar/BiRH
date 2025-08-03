import mujoco
import numpy as np
from mujoco import viewer
import time

# Load model and data
model = mujoco.MjModel.from_xml_path("hand.xml")
data = mujoco.MjData(model)

# Map actuator names to IDs
actuator_ids = {
    "mpj1_motor": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "mpj1_motor"),
    "mpj2_motor": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "mpj2_motor"),
    "mpjt_motor": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "mpjt_motor"),
}

# Control the target angle in degrees
targets = {
    "mpj1_motor": np.linspace(0, 90, 10),
    "mpj2_motor": np.linspace(0, 90, 10),
    "mpjt_motor": np.linspace(-90, 0, 10),  # thumb goes -90 to 0
}

# Launch interactive viewer
with viewer.launch_passive(model, data) as v:
    for name in ["mpj1_motor", "mpj2_motor", "mpjt_motor"]:
        idx = actuator_ids[name]
        for val_rad in targets[name]:
            
            data.ctrl[idx] = val_rad
            
            for _ in range(20):  # simulate a few steps for smoother motion
                mujoco.mj_step(model, data)
                v.sync()
                time.sleep(0.2)

    time.sleep(1)
                