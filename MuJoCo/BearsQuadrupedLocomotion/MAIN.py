from pathlib import Path

import mujoco
import numpy as np

from ReachBotActuators import Motor, Arm, Body
from ReachBotController import ReachBotController
from ReachBotKinematics import forward_kinematics

def organize_ids(model) -> Body:
    # grab ID's for arm 1 and assemble the arm 1 class
    motor11id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor11")
    ctrl_limits11 = model.actuator_ctrlrange[motor11id]
    motor11 = Motor(motor11id, ctrl_limits11)
    revolver11 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolver11")

    motor12id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor12")
    ctrl_limits12 = model.actuator_ctrlrange[motor12id]
    motor12 = Motor(motor12id, ctrl_limits12)
    revolver12 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolver12")

    motor13id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor13")
    ctrl_limits13 = model.actuator_ctrlrange[motor13id]
    motor13 = Motor(motor13id, ctrl_limits13)
    prismatic1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "prismatic1")

    arm1 = Arm(motor11, revolver11, motor12, revolver12, motor13, prismatic1)

    # grab ID's for arm 2 and assemble the arm 2 class
    motor21id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor21")
    ctrl_limits21 = model.actuator_ctrlrange[motor21id]
    motor21 = Motor(motor21id, ctrl_limits21)
    revolver21 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolver21")

    motor22id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor22")
    ctrl_limits22 = model.actuator_ctrlrange[motor22id]
    motor22 = Motor(motor22id, ctrl_limits22)
    revolver22 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolver22")

    motor23id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor23")
    ctrl_limits23 = model.actuator_ctrlrange[motor23id]
    motor23 = Motor(motor23id, ctrl_limits23)
    prismatic2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "prismatic2")

    arm2 = Arm(motor21, revolver21, motor22, revolver22, motor23, prismatic2)

    # grab ID's for arm 3 and assemble the arm 3 class
    motor31id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor31")
    ctrl_limits31 = model.actuator_ctrlrange[motor31id]
    motor31 = Motor(motor31id, ctrl_limits31)
    revolver31 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolver31")

    motor32id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor32")
    ctrl_limits32 = model.actuator_ctrlrange[motor32id]
    motor32 = Motor(motor32id, ctrl_limits32)
    revolver32 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolver3")

    motor33id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor33")
    ctrl_limits33 = model.actuator_ctrlrange[motor33id]
    motor33 = Motor(motor33id, ctrl_limits33)
    prismatic3 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "prismatic3")

    arm3 = Arm(motor31, revolver31, motor32, revolver32, motor33, prismatic1)

    # grab ID's for arm 4 and assemble the arm 4 class
    motor41id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor41")
    ctrl_limits41 = model.actuator_ctrlrange[motor41id]
    motor41 = Motor(motor41id, ctrl_limits41)
    revolver41 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolver41")

    motor42id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor42")
    ctrl_limits42 = model.actuator_ctrlrange[motor42id]
    motor42 = Motor(motor42id, ctrl_limits42)
    revolver42 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolver42")

    motor43id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor43")
    ctrl_limits43 = model.actuator_ctrlrange[motor43id]
    motor43 = Motor(motor43id, ctrl_limits43)
    prismatic4 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "prismatic4")

    arm4 = Arm(motor41, revolver41, motor42, revolver42, motor43, prismatic4)

    body = Body(arm1, arm2, arm3, arm4)

    return body

def main() -> None:
    np.set_printoptions(suppress=True)

    with open(Path.cwd() / "QuadrupedLocomotion" / "Model" / "ReachBot.Xml") as file:
        xml = file.read()
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    model_mass = model.body_subtreemass[0]
    print(f"Total mass of the model: {model_mass}")
    reachbot_ids = organize_ids(model)

    robot = ReachBotController(model, data, reachbot_ids)

    # ---------- [1] ----------
    #  Let robot fall down and set legs to initial pose
    init_pos = {
        reachbot_ids.arm1.motor2.id: np.pi / 4,  
        reachbot_ids.arm2.motor2.id: np.pi / 4,        
        reachbot_ids.arm3.motor2.id: np.pi / 4,   
        reachbot_ids.arm4.motor2.id: np.pi / 4,
        reachbot_ids.arm1.motor3.id: 0.5,  
        reachbot_ids.arm2.motor3.id: 0.5,        
        reachbot_ids.arm3.motor3.id: 0.5,   
        reachbot_ids.arm4.motor3.id: 0.5  
    }
    robot.set_control(init_pos)
    robot.simulate(300)

    ## IGNORE THIS STUFF FOR NOW (GETTING FRAME READOUTS...)
    # print(np.array_str(robot.get_body_pose(), precision=3))
    # # position of arm 2
    # p_arm2_world = robot.get_arm_end_effector_pose(2)
    # p_arm2_world = p_arm2_world[:3, 3]
    # p_arm2_world = np.append(p_arm2_world, 1).reshape(4, 1)
    # p_arm2_body = robot.get_body_pose() @ p_arm2_world
    # print(p_arm2_body)
    # # ground truth pose of arm2 in body frame
    # t1, t2, d3 = robot.get_arm_joint_variables(2)
    # print(forward_kinematics(t1, t2, d3))
    
    while True:
        # ---------- [2] ----------
        # Bring the front right leg (3) forwards
        robot.set_control({reachbot_ids.arm1.motor3.id: 0.4})
        robot.simulate(100)
        robot.set_control({reachbot_ids.arm3.motor3.id: 0.25,
                        reachbot_ids.arm3.motor1.id: np.pi/4})
        robot.simulate(100)
        robot.set_control({reachbot_ids.arm1.motor3.id: 0.5,
                        reachbot_ids.arm3.motor3.id: 0.75})
        robot.simulate(200)

        # ---------- [3] ----------
        # Bring the front left leg (2) forwards
        robot.set_control({reachbot_ids.arm4.motor3.id: 0.4})
        robot.simulate(100)
        robot.set_control({reachbot_ids.arm2.motor3.id: 0.25,
                        reachbot_ids.arm2.motor1.id: -np.pi/4})
        robot.simulate(100)
        robot.set_control({reachbot_ids.arm4.motor3.id: 0.5,
                        reachbot_ids.arm2.motor3.id: 0.75})
        robot.simulate(200)

        # ---------- [4] ----------
        # Smoothly transfer body towards front leg pair
        traj_start = {
            reachbot_ids.arm1.motor1.id: 0,
            reachbot_ids.arm1.motor3.id: 0.5,
            reachbot_ids.arm4.motor1.id: 0,
            reachbot_ids.arm4.motor3.id: 0.5,
            reachbot_ids.arm2.motor1.id: -np.pi/4,
            reachbot_ids.arm2.motor3.id: 0.75,
            reachbot_ids.arm3.motor1.id: np.pi/4,
            reachbot_ids.arm3.motor3.id: 0.75
        }
        traj_end = {
            reachbot_ids.arm1.motor1.id: np.pi/4,
            reachbot_ids.arm1.motor3.id: 0.75,
            reachbot_ids.arm4.motor1.id: -np.pi/4,
            reachbot_ids.arm4.motor3.id: 0.75,
            reachbot_ids.arm2.motor1.id: 0,
            reachbot_ids.arm2.motor3.id: 0.5,
            reachbot_ids.arm3.motor1.id: 0,
            reachbot_ids.arm3.motor3.id: 0.5
        }
        def body_transfer_traj(i, steps):
            ramped_controls = {key: traj_start[key] + (traj_end[key] - traj_start[key]) * (i / steps) for key in traj_start}
            robot.set_control(ramped_controls)
        robot.soft_simulate(300, body_transfer_traj)

        # ---------- [5] ----------
        # Bring back left leg (1) forwards
        robot.set_control({reachbot_ids.arm3.motor3.id: 0.4})
        robot.simulate(100)
        robot.set_control({reachbot_ids.arm1.motor3.id: 0.25,
                        reachbot_ids.arm1.motor1.id: 0})
        robot.simulate(100)
        robot.set_control({reachbot_ids.arm3.motor3.id: 0.5,
                        reachbot_ids.arm1.motor3.id: 0.5})
        robot.simulate(200)

        # ---------- [6] ----------
        # Bring back right leg (4) forwards
        robot.set_control({reachbot_ids.arm2.motor3.id: 0.4})
        robot.simulate(100)
        robot.set_control({reachbot_ids.arm4.motor3.id: 0.25,
                        reachbot_ids.arm4.motor1.id: 0})
        robot.simulate(100)
        robot.set_control({reachbot_ids.arm2.motor3.id: 0.5,
                        reachbot_ids.arm4.motor3.id: 0.5})
        robot.simulate(200)
        
    robot.close_viewer()

if __name__ == "__main__":
    main()
