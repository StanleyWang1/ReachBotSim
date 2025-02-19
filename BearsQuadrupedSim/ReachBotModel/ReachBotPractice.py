import time
import mujoco
import mujoco.viewer
import numpy as np
from ReachBotActuators import Motor, Arm, Body

timestep = 0.001


def simulate(model, model_ids, data):
    global timestep
    viewer = mujoco.viewer.launch_passive(model, data)
    arms_vars = vars(model_ids).items()
    arms_dict = dict(arms_vars)
    arms_list = list(arms_vars)


    for i in range(5000):
        data.ctrl[model_ids.arm1.motor2.id] = np.pi / 4
        data.ctrl[model_ids.arm2.motor2.id] = np.pi / 4
        data.ctrl[model_ids.arm3.motor2.id] = np.pi / 4
        data.ctrl[model_ids.arm4.motor2.id] = np.pi / 4
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(timestep)

    for i in range(800):
        data.ctrl[model_ids.arm1.motor3.id] = .5
        data.ctrl[model_ids.arm2.motor3.id] = .5
        data.ctrl[model_ids.arm3.motor3.id] = .5
        data.ctrl[model_ids.arm4.motor3.id] = .5
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(timestep)

    for i in range(4):
        for i in range(800):
            data.ctrl[model_ids.arm1.motor1.id] = np.pi / 8
            data.ctrl[model_ids.arm2.motor1.id] = np.pi / 8
            data.ctrl[model_ids.arm3.motor1.id] = np.pi / 8
            data.ctrl[model_ids.arm4.motor1.id] = np.pi / 8
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(timestep)

        for i in range(len(arms_list)):
            for j in range(600):
                data.ctrl[arms_dict[arms_list[(i + 2) % 4][0]].motor3.id] = .36
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(timestep)
            moveLimb(arms_dict[arms_list[i][0]], model, data, viewer, -np.pi/8, data.qpos[model.jnt_qposadr[arms_dict[arms_list[i][0]].revolute2]], 0.5)
            for j in range(600):
                data.ctrl[arms_dict[arms_list[(i + 2) % 4][0]].motor3.id] = .5
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(timestep)




        for i in range(800):
            data.ctrl[model_ids.arm1.motor3.id] = .5
            data.ctrl[model_ids.arm2.motor3.id] = .5
            data.ctrl[model_ids.arm3.motor3.id] = .5
            data.ctrl[model_ids.arm4.motor3.id] = .5
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(timestep)

    for i in range(1000):
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(timestep)

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

    # grab ID's for arm 2 and assemble the arm 2 class
    motor21id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor21p")
    ctrl_limits21 = model.actuator_ctrlrange[motor21id]
    motor21 = Motor(motor21id, ctrl_limits21)
    revolver21 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolver21")

    motor22id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor22p")
    ctrl_limits22 = model.actuator_ctrlrange[motor22id]
    motor22 = Motor(motor22id, ctrl_limits22)
    revolver22 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolver22")

    motor23id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor23p")
    ctrl_limits23 = model.actuator_ctrlrange[motor23id]
    motor23 = Motor(motor23id, ctrl_limits23)
    prismatic2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "prismatic2")

    arm2 = Arm(motor21, revolver21, motor22, revolver22, motor23, prismatic2)

    # grab ID's for arm 3 and assemble the arm 3 class
    motor31id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor31p")
    ctrl_limits31 = model.actuator_ctrlrange[motor31id]
    motor31 = Motor(motor31id, ctrl_limits31)
    revolver31 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolver31")

    motor32id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor32p")
    ctrl_limits32 = model.actuator_ctrlrange[motor32id]
    motor32 = Motor(motor32id, ctrl_limits32)
    revolver32 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolver3")

    motor33id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor33p")
    ctrl_limits33 = model.actuator_ctrlrange[motor33id]
    motor33 = Motor(motor33id, ctrl_limits33)
    prismatic3 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "prismatic3")

    arm3 = Arm(motor31, revolver31, motor32, revolver32, motor33, prismatic1)

    # grab ID's for arm 4 and assemble the arm 4 class
    motor41id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor41p")
    ctrl_limits41 = model.actuator_ctrlrange[motor41id]
    motor41 = Motor(motor41id, ctrl_limits41)
    revolver41 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolver41")

    motor42id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor42p")
    ctrl_limits42 = model.actuator_ctrlrange[motor42id]
    motor42 = Motor(motor42id, ctrl_limits42)
    revolver42 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolver42")

    motor43id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor43p")
    ctrl_limits43 = model.actuator_ctrlrange[motor43id]
    motor43 = Motor(motor43id, ctrl_limits43)
    prismatic4 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "prismatic4")

    arm4 = Arm(motor41, revolver41, motor42, revolver42, motor43, prismatic4)

    body = Body(arm1, arm2, arm3, arm4)

    return body


def main():
    with open(r"Archive\ReachBot.Xml") as file:
        xml = file.read()
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    model_mass = model.body_subtreemass[0]
    print(f"Total mass of the model: {model_mass}")
    reachbot_ids = organize_ids(model)

    simulate(model, reachbot_ids, data)


if __name__ == "__main__":
    main()
