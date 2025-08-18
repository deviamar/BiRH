# Code to test grasp stability

import mujoco #type: ignore
from mujoco import viewer #type: ignore
import numpy as np
import time
import math

model = mujoco.MjModel.from_xml_path("/Users/stellaluna/Documents/GitHub/BiRH/simulation/mujoco-models/scene.xml")
data = mujoco.MjData(model)

# Bring arm and hand to initial position
def reset_pos():
    home_keyframe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_keyframe_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home_keyframe_id)

reset_pos()

# Given the array of contacts, check if two geoms are in contact
def in_contact(model, data, geom_1, geom_2):
    geom_1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_1)
    geom_2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_2)

    for i in range(data.ncon):
        c = data.contact[i]
        # Check both possible orderings of the geom pair
        if (c.geom1 == geom_1_id and c.geom2 == geom_2_id) or \
           (c.geom1 == geom_2_id and c.geom2 == geom_1_id):
            return True
    
    return False

# Get in-depth info on all contacts
def get_contact_info(model, data):
    contacts = []
    
    # Iterate through all active contacts
    for i in range(data.ncon):
        contact = data.contact[i]
        
        # Get geom names (if they exist)
        geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
        geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
        
        contacts.append({
            'geom1_id': contact.geom1,
            'geom2_id': contact.geom2,
            'geom1_name': geom1_name if geom1_name else f"geom_{contact.geom1}",
            'geom2_name': geom2_name if geom2_name else f"geom_{contact.geom2}",
            'pos': contact.pos.copy(),
            'frame': contact.frame.copy(),
            'dist': contact.dist,
            'includemargin': contact.includemargin,
            'friction': contact.friction,
        })
    
    return contacts

# take a step toward grasp
def grasp_step(model, data, obj_name, finger_states, increment=0.001):
    obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, obj_name)
    any_moving = False

    for finger_name, finger in finger_states.items():
        if finger['current'] >= len(finger['joints']):
            continue
        any_moving = True

        joint_name = finger['joints'][finger['current']]
        pad_name   = finger['pads'][finger['current']]
        actuator_name = finger['actuators'][finger['current']]

        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        pad_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,  pad_name)
        act_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,  actuator_name)
        qadr     = model.jnt_qposadr[joint_id]
        
        # Initialize per-joint state
        if 'start_pos' not in finger:
            finger['start_pos'] = float(data.qpos[qadr])
            finger['steps'] = 0

        finger['steps'] += 1
        target = finger['start_pos'] + increment * finger['steps']

        # Respect joint limits
        lo, hi = (model.jnt_range[joint_id] if model.jnt_limited[joint_id] else (-np.inf, np.inf))
        target = max(lo, min(hi, target))

        # Drive via actuator if present (position actuators in your XML), else set qpos
        if act_id >= 0:
            data.ctrl[act_id] = target
        else:
            data.qpos[qadr] = target
            mujoco.mj_forward(model, data)

        if in_contact(model, data, pad_name, obj_name):
            finger['current'] += 1
            finger.pop('start_pos', None)
            finger['steps'] = 0
            print(f"{finger_name} contact detected, advancing to next joint")

    return any_moving

#check if all fingers are done
def all_fingers_done(finger_states, contacts):
    phalanges = {"pp1", "mp1", "pp2", "mp2", "pp3", "mp3", "palm"}
    for f in finger_states.values():
        if f['current'] < len(f['joints']):
            pad = f['pads'][f['current']]
            for ph in phalanges:
                if in_contact(model, data, pad, ph):
                    f['current'] += 1
            return False
    return True

# Initialize finger states
finger_states = {
    'finger1': {'joints': ['mpj1', 'pipj1'], 'pads': ['pp1_pad', 'mp1_pad'], 'actuators': ['mpj1_motor', 'pipj1_motor'], 'current': 0},
    'finger2': {'joints': ['mpj2', 'pipj2'], 'pads': ['pp2_pad', 'mp2_pad'], 'actuators': ['mpj2_motor', 'pipj2_motor'], 'current': 0},
    'finger3': {'joints': ['mpj3', 'pipj3'], 'pads': ['pp3_pad', 'mp3_pad'], 'actuators': ['mpj3_motor', 'pipj3_motor'], 'current': 0}
}


# Check if object slips more than 1 cm
def linear_slip(model, data, dist_i, obj, slip_dist=0.01) -> bool:
    obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, obj)
    palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "palm")
    obj_com = data.xipos[obj_id].copy()
    palm_com = data.xipos[palm_id].copy()
    dist_vec = obj_com - palm_com
    dist = np.linalg.norm(dist_vec)
    return (dist - dist_i) > slip_dist

def quat_conj(q):
    x, y, z, w = q
    return np.array([-x, -y, -z, w], dtype=float)

def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    q = np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,  # x
        w1*y2 - x1*z2 + y1*w2 + z1*x2,  # y
        w1*z2 + x1*y2 - y1*x2 + z1*w2,  # z
        w1*w2 - x1*x2 - y1*y2 - z1*z2   # w
    ], dtype=float)
    n = np.linalg.norm(q)
    return q if n == 0 else q / n

# Check if object slips more than 5 degrees
def rot_slip(model, data, quat_i, obj, slip_ang=5) -> bool:
    obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, obj)
    palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "palm")
    quat_obj = data.xquat[obj_id].copy()
    quat_palm = data.xquat[palm_id].copy()
    r_0 = quat_i
    r_t = quat_mul(quat_conj(quat_palm), quat_obj)
    r = quat_mul(quat_conj(r_0), r_t)
    w = float(np.clip(r[3], -1.0, 1.0))
    deg = np.degrees(2.0 * np.arccos(abs(w)))
    return deg > slip_ang

# Create random unit vectors
def random_unit_vectors(n, rng=0):
    rng = np.random.default_rng(rng)
    v = rng.normal(size=(n,3))
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    return v

# Apply force until grasp failure
def max_force(model, data, obj, grasp):
    obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, obj)
    palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "palm")
    obj_com = data.xipos[obj_id].copy()
    palm_com = data.xipos[palm_id].copy()
    dist_vec = obj_com - palm_com
    dist_i = np.linalg.norm(dist_vec)
    
    quat_obj = data.xquat[obj_id].copy()
    quat_palm = data.xquat[palm_id].copy()
    quat_i = quat_mul(quat_conj(quat_palm), quat_obj)

    max_forces = []

    vectors = random_unit_vectors(10, 0)
    for v in vectors:
        n = 1
        while not (linear_slip(model, data, dist_i, obj, slip_dist=0.01) or rot_slip(model, data, quat_i, obj, slip_ang=5)):
            data.xfrc_applied[obj_id, :3] = np.asarray(v * n, float)
            mujoco.mj_step(model, data)
            n += .1
        max_forces.append(v * (n-1))
        clear_wrench(model, data, obj)
        restore_keyframe(model, data, grasp)

    return max_forces

# Clear all external wrenches
def clear_wrench(model, data, obj):
    obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, obj)
    data.xfrc_applied[obj_id, :] = 0.0

def capture_keyframe(model, data):
    return {
        "qpos": data.qpos.copy(),
        "qvel": data.qvel.copy(),
        "act":  data.act.copy()  if model.na   > 0 else None,  # internal actuator states
        "ctrl": data.ctrl.copy() if model.nu   > 0 else None,  # actuator targets
        "mpos": data.mocap_pos.copy()   if model.nmocap > 0 else None,
        "mquat":data.mocap_quat.copy()  if model.nmocap > 0 else None,
    }

def restore_keyframe(model, data, snap):
    data.qpos[:] = snap["qpos"]
    data.qvel[:] = snap["qvel"]
    if snap["act"]  is not None: data.act[:]  = snap["act"]
    if snap["ctrl"] is not None: data.ctrl[:] = snap["ctrl"]
    if snap["mpos"] is not None: data.mocap_pos[:]  = snap["mpos"]
    if snap["mquat"] is not None: data.mocap_quat[:] = snap["mquat"]

    # Clear any residual forces/warm-start junk
    data.xfrc_applied[:] = 0.0
    data.qfrc_applied[:] = 0.0
    data.qacc[:] = 0.0
    data.qacc_warmstart[:] = 0.0

    mujoco.mj_forward(model, data)

phase = "closing"         # "closing" -> "testing" -> "done"
#obj_geom_name = "cyl_g"
obj_geom_name = "apple_g"
#grasp = capture_keyframe(model, data)
max_forces = []
with viewer.launch_passive(model, data) as v:
    while v.is_running():
        # 1) advance physics
        mujoco.mj_step(model, data)
        
        if phase == "closing":
            
            all_contacts = get_contact_info(model, data)
            #print(all_contacts)

            # Nudge the fingers inwards toward a grasp
            moving = grasp_step(model, data, obj_geom_name, finger_states, increment=0.001)
            for _ in range(5):
                mujoco.mj_step(model, data)
            #time.sleep(0.01)
            
            # Option A: declare done when all fingers advanced through their joints
            if all_fingers_done(finger_states, all_contacts):
                phase = "testing"
                print("Grasp complete!")
                grasp = capture_keyframe(model, data)
            
        """if phase == "testing":
            max_forces = max_force(model, data, obj_geom_name, grasp)
            print(max_forces)"""
        v.sync()