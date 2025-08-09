# Code to detect contact and test grasp stability
import mujoco #type: ignore
from mujoco import viewer #type: ignore
import numpy as np
import time
import math

model = mujoco.MjModel.from_xml_path("/Users/stellaluna/Documents/GitHub/BiRH/simulation/mujoco-models/scene.xml")
data = mujoco.MjData(model)

# method to bring arm and hand to initial position
def reset_pos():
    home_keyframe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_keyframe_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home_keyframe_id)

reset_pos()

# given the array of contacts, check if two geoms are in contact
def in_contact(model, contacts, geom_1, geom_2):
    geom_1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_1)
    geom_2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_2)

    for contact in contacts:
        # Check both possible orderings of the geom pair
        if (contact.geom1 == geom_1_id and contact.geom2 == geom_2_id) or \
           (contact.geom1 == geom_2_id and contact.geom2 == geom_1_id):
            return True
    
    return False

# get in-depth info on all contacts
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
            'pos': contact.pos.copy(),  # Contact position in world coordinates
            'frame': contact.frame.copy(),  # Contact frame (normal, tangent vectors)
            'dist': contact.dist,  # Contact distance
            'includemargin': contact.includemargin,
            'friction': contact.friction,
        })
    
    return contacts

# take a step toward grasp
def grasp_step(model, data, obj_name, finger_states, increment=0.005):
    obj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, obj_name)
    any_moving = False

    for finger_name, finger in finger_states.items():
        if finger['current'] >= len(finger['joints']):
            continue
        any_moving = True

        joint_name = finger['joints'][finger['current']]
        pad_name   = finger['pads'][finger['current']]

        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        pad_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,  pad_name)
        qadr     = model.jnt_qposadr[joint_id]
        act_id   = _actuator_for_joint(model, joint_id)

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

        # Contact check
        contact_detected = False
        for i in range(data.ncon):
            c = data.contact[i]
            if (c.geom1 == pad_id and c.geom2 == obj_id) or (c.geom1 == obj_id and c.geom2 == pad_id):
                contact_detected = True
                break

        if contact_detected:
            finger['current'] += 1
            finger.pop('start_pos', None)
            finger['steps'] = 0
            print(f"{finger_name} contact detected, advancing to next joint")

    return any_moving
    
def _actuator_for_joint(model, joint_id):
    for a in range(model.nu):
        if model.actuator_trnid[a][0] == joint_id:
            return a
    return -1

def get_ids(model, obj_geom_name, palm_body_name, pad_geom_names):
    obj_geom_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, obj_geom_name)
    obj_body_id  = model.geom_bodyid[obj_geom_id]
    palm_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, palm_body_name)
    pad_geom_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for n in pad_geom_names]
    return obj_geom_id, obj_body_id, palm_body_id, pad_geom_ids

def in_contact_live(model, data, geom_1, geom_2):
    id1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_1)
    id2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_2)
    for i in range(data.ncon):
        c = data.contact[i]
        if (c.geom1 == id1 and c.geom2 == id2) or (c.geom1 == id2 and c.geom2 == id1):
            return True
    return False

#check if all fingers are done
def all_fingers_done(finger_states, contacts):
    phalanges = {"pp1", "mp1", "pp2", "mp2", "ppt", "mpt", "palm"}
    for f in finger_states.values():
        if f['current'] < len(f['joints']):
            pad = f['pads'][f['current']]
            for ph in phalanges:
                if in_contact_live(model, data, pad, ph):
                    f['current'] += 1
            return False
    return True

# Initialize finger states
finger_states = {
    'finger1': {'joints': ['mpj1', 'pipj1'], 'pads': ['pp1_pad', 'mp1_pad'], 'current': 0},
    'finger2': {'joints': ['mpj2', 'pipj2'], 'pads': ['pp2_pad', 'mp2_pad'], 'current': 0},
    'thumb': {'joints': ['mpjt', 'pipjt'], 'pads': ['ppt_pad', 'mpt_pad'], 'current': 0}
}

phase = "closing"         # "closing" -> "monitor_ready" -> "testing" -> "done"
obj_geom_name = "apple_g"

# Launch viewer - this is the easiest and fastest way
with viewer.launch_passive(model, data) as v:
    while v.is_running():
        # 1) advance physics
        mujoco.mj_step(model, data)

        if phase == "closing":
            all_contacts = get_contact_info(model, data)

            # Nudge the fingers inwards toward a grasp
            moving = grasp_step(model, data, obj_geom_name, finger_states, increment=0.005)
            time.sleep(0.01)
            
            # Option A: declare done when all fingers advanced through their joints
            if all_fingers_done(finger_states, all_contacts):
                phase = "monitor_ready"
                print("Grasp complete!")
            
            """ print("Number of contacts: ", data.ncon)
            for contact in all_contacts:
                print(f"  {contact['geom1_name']} <-> {contact['geom2_name']}")
                print() """
        v.sync()