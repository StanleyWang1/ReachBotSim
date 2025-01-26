import time
import mujoco
import mujoco.viewer
import random
import numpy as np
from ReachBotActuators import Motor, Arm, Body

timestep = 0.01


def simulate(model, model_ids, data):
    global timestep
    viewer = mujoco.viewer.launch_passive(model, data)
    arms_vars = vars(model_ids).items()
    arms_dict = dict(arms_vars)
    arms_list = list(arms_vars)
    test = 0
    for i in range(10000):
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(timestep)
        if i%1000 == 0:
            test += 1
            for i in range(3):
                curr_pos = data.qpos[model.jnt_qposadr[i]]
                target_pos = data.ctrl[i]
                percent_error = abs((curr_pos-target_pos)/target_pos)
                if abs(percent_error) > 0.005:
                    print("test " + str(test) + " fail")
                else:
                    print("test " + str(test) + " success")

            data.ctrl[arms_dict['revolute1']] = random.uniform(*arms_dict['motor1'].ctrl_limits)
            data.ctrl[arms_dict['revolute2']] = random.uniform(*arms_dict['motor2'].ctrl_limits)
            data.ctrl[arms_dict['prismatic']] = random.uniform(*arms_dict['motor3'].ctrl_limits)

            print('hello world')

    viewer.close()


def moveLimbXYZ(arm, model, data, viewer, x, y, z):
    thetaA = 0
    thetaB = 0
    length = 0

    # first, retract limb for lowest inertia
    for i in range(200):
        data.ctrl[arm.motor3.id] = 0
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(timestep)
        x_curr = data.qpos[model.jnt_qposadr[arm.prismatic]]
    for i in range(200):
        data.ctrl[arm.motor1.id] = thetaA
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(timestep)

    for i in range(200):
        data.ctrl[arm.motor2.id] = thetaB
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(timestep)

    for i in range(200):
        data.ctrl[arm.motor3.id] = length
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(timestep)


def moveLimb(arm, model, data, viewer, thetaA, thetaB, length):
    thetaA_curr = data.qpos[model.jnt_qposadr[arm.revolute1]]
    thetaB_curr = data.qpos[model.jnt_qposadr[arm.revolute2]]
    length_curr = data.qpos[model.jnt_qposadr[arm.prismatic]]

    # first, retract limb for lowest inertia
    for i in range(200):
        data.ctrl[arm.motor3.id] = 0
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(timestep)
        length_curr = data.qpos[model.jnt_qposadr[arm.prismatic]]

    for i in range(200):
        data.ctrl[arm.motor1.id] = thetaA
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(timestep)

    for i in range(200):
        data.ctrl[arm.motor2.id] = thetaB
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(timestep)

    for i in range(200):
        data.ctrl[arm.motor3.id] = length
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(timestep)


def findIntermediatePts(desired, current):
    intermediate_pts = 10
    path = []
    timestepheta = np.pi/intermediate_pts
    r = abs(desired - current)/2
    c = (desired + current)/2
    for i in range(intermediate_pts):
        path_pt = r*np.cos(i*timestepheta) + c
        path.append(path_pt)
    return path


# currentPos = data.qpos[ZaxisJointID]
# data.ctrl[ZaxisMotorID] = targetPos  # Assuming a single motor, indexed at 0
# error = abs(currentPos - targetPos)
# velocity = data.qvel[ZaxisMotorID]
# if error < 0.01 and velocity<0.01:
#     if targetPos >= ctrl_limitsZ[1] - 0.1:
#         CW = True
#     if targetPos <= ctrl_limitsZ[0] + 0.1:
#         CW = False
#     targetPos = targetPos + (np.pi/4)*(1 - 2*CW)


def organize_ids(model):
    # grab ID's for arm 1 and assemble the arm 1 class
    motor11id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor11p")
    ctrl_limits11 = model.actuator_ctrlrange[motor11id]
    motor11 = Motor(motor11id, ctrl_limits11)
    revolver11 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolver11")

    motor12id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor12p")
    ctrl_limits12 = model.actuator_ctrlrange[motor12id]
    motor12 = Motor(motor12id, ctrl_limits12)
    revolver12 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolver12")

    motor13id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor13p")
    ctrl_limits13 = model.actuator_ctrlrange[motor13id]
    motor13 = Motor(motor13id, ctrl_limits13)
    prismatic1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "prismatic1")

    arm1 = Arm(motor11, revolver11, motor12, revolver12, motor13, prismatic1)

    return arm1


def main():
    with open("ReachBotSingleArm.Xml") as file:
        xml = file.read()
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    model_mass = model.body_subtreemass[0]
    print(f"Total mass of the model: {model_mass}")
    reachbot_ids = organize_ids(model)

    simulate(model, reachbot_ids, data)


if __name__ == "__main__":
    main()
