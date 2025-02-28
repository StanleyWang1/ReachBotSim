import os
import csv
import time
import mujoco
import mujoco.viewer
import numpy as np

from config import ITERS_PER_STEP, TIMESTEP, Z_STABLE_THRESHOLD, CONTROL_DICT
## --------------------------------------------------
# Change directory to location of MuJuCo Model
os.chdir("C:/Users/stanl/OneDrive/Documents/GitHub/ReachBotSim/GIRAF/anybotics_anymal_c")

def simulate(model, data):
    global ITERS_PER_STEP, TIMESTEP
    iters = 1000

    viewer = mujoco.viewer.launch_passive(model, data)
    base_id = model.body("base").id    # Body 1
    boom_id = model.body("boom1").id   # Body 7

    start_time = time.perf_counter()

    for i in range(iters):
        print(f"Starting iteration {i}")
        
        # Generate random coordinates in jointspace for GIRAF
        roll_pos = np.random.uniform(-np.pi, np.pi)
        pitch_pos = np.random.uniform(0, np.pi)
        boom_pos = np.random.uniform(0, 3)

        # Reset Simulation
        mujoco.mj_resetData(model, data)

        # Sim for 2 sec, examine failure
        for i in range(100):
            cycle_start = time.perf_counter()

            data.ctrl[CONTROL_DICT['LF_HAA']] = -0.4
            data.ctrl[CONTROL_DICT['RF_HAA']] = 0.4
            data.ctrl[CONTROL_DICT['LH_HAA']] = -0.4
            data.ctrl[CONTROL_DICT['RH_HAA']] = 0.4

            data.ctrl[CONTROL_DICT['LF_HFE']] = 0.25
            data.ctrl[CONTROL_DICT['RF_HFE']] = 0.25
            data.ctrl[CONTROL_DICT['LH_HFE']] = -0.25
            data.ctrl[CONTROL_DICT['RH_HFE']] = -0.25

            data.ctrl[CONTROL_DICT['LF_KFE']] = -0.5
            data.ctrl[CONTROL_DICT['RF_KFE']] = -0.5
            data.ctrl[CONTROL_DICT['LH_KFE']] = 0.5
            data.ctrl[CONTROL_DICT['RH_KFE']] = 0.5
            
            data.ctrl[CONTROL_DICT['motor11p']] = roll_pos
            data.ctrl[CONTROL_DICT['motor12p']] = pitch_pos
            data.ctrl[CONTROL_DICT['motor13p']] = boom_pos

            for _ in range(ITERS_PER_STEP):  # Step multiple times per rendering frame
                mujoco.mj_step(model, data)

            set_camera(viewer, model, data)
            viewer.sync()

            # Maintain real-time speed
            elapsed = time.perf_counter() - cycle_start
            sleep_time = max(0, TIMESTEP - elapsed)
            time.sleep(sleep_time)

        # Get results
        ANYmal_pos = data.xpos[base_id]
        EE_pos = data.xpos[boom_id]
        if ANYmal_pos[-1] < Z_STABLE_THRESHOLD:
            stable = False
        else:
            stable = True

        csv_filename = "narrow_stance_1kg_i100n100.csv"
        file_exists = os.path.isfile(csv_filename)
        with open(csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            if not file_exists:
                writer.writerow(["EE_pos_x", "EE_pos_y", "EE_pos_z", "stable"])
            writer.writerow([EE_pos[0], EE_pos[1], EE_pos[2], stable])

    viewer.close()

def set_camera(viewer, model, data):
    viewer.cam.azimuth = 180   # Rotation around the Z-axis (degrees)
    viewer.cam.elevation = -5  # Up/Down tilt angle
    viewer.cam.distance = 4.0  # Distance from the model
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
