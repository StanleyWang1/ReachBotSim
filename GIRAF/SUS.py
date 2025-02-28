import os
import time
import mujoco
import mujoco.viewer
import numpy as np

## --------------------------------------------------
# Change directory to location of MuJoCo Model
os.chdir("C:/Users/stanl/OneDrive/Documents/GitHub/ReachBotSim/GIRAF/anybotics_anymal_c")

ITERS_PER_STEP = 10
TIMESTEP = 0.01

# Define gait cycle sequence (quadruped cycle: LF → RH → RF → LH)
GAIT_SEQUENCE = ['LF', 'RH', 'RF', 'LH']
STEP_DURATION = 0.25  # Time spent in each step phase

# Joint control mapping
control_dict = {
    'LF_HAA': 0, 'LF_HFE': 1, 'LF_KFE': 2,
    'RF_HAA': 3, 'RF_HFE': 4, 'RF_KFE': 5,
    'LH_HAA': 6, 'LH_HFE': 7, 'LH_KFE': 8,
    'RH_HAA': 9, 'RH_HFE': 10, 'RH_KFE': 11
}

def step_leg(data, leg, lift=True):
    """Applies control to lift or place a leg"""
    hip_target = -0.3 if lift else 0.1  # Move forward when lifting
    knee_target = 0.4 if lift else -0.2  # Bend knee when lifting
    
    data.ctrl[control_dict[f'{leg}_HFE']] = hip_target
    data.ctrl[control_dict[f'{leg}_KFE']] = knee_target

def simulate(model, data):
    """Runs the MuJoCo simulation with a step-based gait."""
    viewer = mujoco.viewer.launch_passive(model, data)
    start_time = time.perf_counter()
    step_index = 0

    for _ in range(10000):  # Run for fixed steps
        cycle_start = time.perf_counter()
        t = time.perf_counter() - start_time

        # Determine current leg in gait sequence
        current_leg = GAIT_SEQUENCE[step_index]

        # Lift current leg
        step_leg(data, current_leg, lift=True)

        # Keep other legs planted
        for leg in GAIT_SEQUENCE:
            if leg != current_leg:
                step_leg(data, leg, lift=False)

        # Advance to next step in gait cycle
        if (t % (len(GAIT_SEQUENCE) * STEP_DURATION)) < STEP_DURATION:
            step_index = (step_index + 1) % len(GAIT_SEQUENCE)

        # Step physics engine
        for _ in range(ITERS_PER_STEP):  
            mujoco.mj_step(model, data)

        viewer.sync()

        # Maintain real-time sync
        elapsed = time.perf_counter() - cycle_start
        time.sleep(max(0, TIMESTEP - elapsed))

    viewer.close()

def main():
    with open("GIRAF.xml") as file:
        xml = file.read()
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    print(f"Total mass of the model: {model.body_subtreemass[0]}")

    simulate(model, data)

if __name__ == "__main__":
    main()
