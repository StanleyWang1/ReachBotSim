import os
import csv
import mujoco
import numpy as np

from config import ITERS_PER_STEP, TIMESTEP, Z_STABLE_THRESHOLD, CONTROL_DICT

# Change directory to the location of the MuJoCo model
os.chdir("C:/Users/stanl/OneDrive/Documents/GitHub/ReachBotSim/GIRAF/anybotics_anymal_c")

def simulate(model, data):
    iters = 10000

    base_id = model.body("base").id    # Body 1
    boom_id = model.body("boom1").id   # Body 7

    for i in range(iters):
        print(f"Starting iteration {i}")

        # Generate random coordinates in joint space for GIRAF
        roll_pos = np.random.uniform(-np.pi, np.pi)
        pitch_pos = np.random.uniform(0, np.pi)
        boom_pos = np.random.uniform(0, 3)

        # Reset simulation
        mujoco.mj_resetData(model, data)

        # Simulate for 2 seconds
        for _ in range(100):
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

            # Step the simulation
            for _ in range(ITERS_PER_STEP):
                mujoco.mj_step(model, data)

        # Get results
        ANYmal_pos = data.xpos[base_id]
        EE_pos = data.xpos[boom_id]
        stable = ANYmal_pos[-1] >= Z_STABLE_THRESHOLD

        # Write results to CSV
        csv_filename = "narrow_stance_1kg_hl_i100n100_x10000.csv"
        file_exists = os.path.isfile(csv_filename)
        with open(csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            if not file_exists:
                writer.writerow(["EE_pos_x", "EE_pos_y", "EE_pos_z", "stable"])
            writer.writerow([EE_pos[0], EE_pos[1], EE_pos[2], stable])

def main():
    with open("GIRAF.xml") as file:
        xml = file.read()
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    model_mass = model.body_subtreemass[0]
    print(f"Total mass of the model: {model_mass}")

    simulate(model, data)

if __name__ == "__main__":
    main()
