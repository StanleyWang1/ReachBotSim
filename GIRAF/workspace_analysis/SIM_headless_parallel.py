import os
import csv
import mujoco
import numpy as np
import multiprocessing

from config import ITERS_PER_STEP, Z_STABLE_THRESHOLD, CONTROL_DICT

# Change directory to MuJoCo model
os.chdir("C:/Users/stanl/OneDrive/Documents/GitHub/ReachBotSim/GIRAF/anybotics_anymal_c")

def simulate(instance_id):
    with open("GIRAF.xml") as file:
        xml = file.read()
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    base_id = model.body("base").id
    boom_id = model.body("boom1").id

    for _ in range(250):  # Divide total iterations across processes
        roll_pos = np.random.uniform(-np.pi, np.pi)
        pitch_pos = np.random.uniform(0, np.pi)
        boom_pos = np.random.uniform(0, 3)

        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)

        for _ in range(50):
            data.ctrl[CONTROL_DICT['LF_HAA']] = -0.4
            data.ctrl[CONTROL_DICT['RF_HAA']] = 0.4
            data.ctrl[CONTROL_DICT['LH_HAA']] = -0.4
            data.ctrl[CONTROL_DICT['RH_HAA']] = 0.4
            data.ctrl[CONTROL_DICT['motor11p']] = roll_pos
            data.ctrl[CONTROL_DICT['motor12p']] = pitch_pos
            data.ctrl[CONTROL_DICT['motor13p']] = boom_pos

            for _ in range(2):  # Run multiple steps per iteration
                mujoco.mj_step(model, data)

        ANYmal_pos = data.xpos[base_id]
        EE_pos = data.xpos[boom_id]
        stable = ANYmal_pos[-1] >= Z_STABLE_THRESHOLD

        # Save results
        csv_filename = f"narrow_stance_1kg_process_{instance_id}.csv"
        with open(csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([EE_pos[0], EE_pos[1], EE_pos[2], stable])

if __name__ == "__main__":
    num_processes = os.cpu_count()  # Use all available CPU cores
    print(f"Running {num_processes} parallel Mujoco simulations...")

    with multiprocessing.Pool(num_processes) as pool:
        pool.map(simulate, range(num_processes))
