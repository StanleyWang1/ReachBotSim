import os
import time
import mujoco
import mujoco.viewer
import random
import numpy as np

## --------------------------------------------------
# Change directory to location of MuJuCo Model
os.chdir("C:/Users/stanl/OneDrive/Documents/GitHub/ReachBotSim/GIRAF/anybotics_anymal_c")

ITERS_PER_STEP = 10
TIMESTEP = 0.01

Z_STABLE_THRESHOLD = 0.55 # if the Z pos of ANYmal base falls below, indicates a fall (unstable)

control_dict = {
    'LF_HAA' : 0,
    'LF_HFE' : 1,
    'LF_KFE' : 2,
    'RF_HAA' : 3,
    'RF_HFE' : 4,
    'RF_KFE' : 5,
    'LH_HAA' : 6,
    'LH_HFE' : 7,
    'LH_KFE' : 8,
    'RH_HAA' : 9,
    'RH_HFE' : 10,
    'RH_KFE' : 11,
    'motor11p' : 12,
    'motor12p' : 13,
    'motor13p' : 14
}

def simulate(model, data):
    global ITERS_PER_STEP, TIMESTEP
    viewer = mujoco.viewer.launch_passive(model, data)

    base_id = model.body("base").id    # Body 1
    boom_id = model.body("boom1").id   # Body 7

    start_time = time.perf_counter()

    for i in range(10000):
        cycle_start = time.perf_counter()

        data.ctrl[control_dict['LF_HAA']] = -0.4
        data.ctrl[control_dict['RF_HAA']] = 0.4
        data.ctrl[control_dict['LH_HAA']] = -0.4
        data.ctrl[control_dict['RH_HAA']] = 0.4

        data.ctrl[control_dict['LF_HFE']] = 0.25
        data.ctrl[control_dict['RF_HFE']] = 0.25
        data.ctrl[control_dict['LH_HFE']] = -0.25
        data.ctrl[control_dict['RH_HFE']] = -0.25

        data.ctrl[control_dict['LF_KFE']] = -0.5
        data.ctrl[control_dict['RF_KFE']] = -0.5
        data.ctrl[control_dict['LH_KFE']] = 0.5
        data.ctrl[control_dict['RH_KFE']] = 0.5

        # if i > 250:
        #     data.ctrl[control_dict['motor13p']] = 0.5

        for _ in range(ITERS_PER_STEP):  # Step multiple times per rendering frame
            mujoco.mj_step(model, data)

        set_camera(viewer, model, data, i)
        viewer.sync()

        base_pos = data.xpos[base_id]
        boom_pos = data.xpos[boom_id]
        print(base_pos)

        # Maintain real-time speed
        elapsed = time.perf_counter() - cycle_start
        sleep_time = max(0, TIMESTEP - elapsed)
        time.sleep(sleep_time)

    viewer.close()

def set_camera(viewer, model, data, i):
    viewer.cam.azimuth = (i/3) % 360   # Rotation around the Z-axis (degrees)
    viewer.cam.elevation = -30  # Up/Down tilt angle
    viewer.cam.distance = 8.0  # Distance from the model
    base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")  
    if base_body_id != -1:
        viewer.cam.lookat[:] = data.xpos[base_body_id]  # Set camera focus to base position

def main():
    with open("GIRAF.xml") as file:
        xml = file.read()
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    model_mass = model.body_subtreemass[0]
    print(f"Total mass of the model: {model_mass}")

    # for i in range(model.nbody):
    #     print(f"Body {i}: {model.body(i).name}")
    simulate(model, data)

if __name__ == "__main__":
    main()
