import mujoco
import numpy as np
from pathlib import Path
from actuators import Motor, Arm, Body
from .controller import Controller


class Simulation:
    @staticmethod
    def __create_body(model) -> Body:
        arms = []
        for i in range(1, 5):
            parts = []
            for j in range(1, 4):
                motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"motor{i}{j}")
                ctrl_limits = model.actuator_ctrlrange[motor_id]
                aux_name = f"revolver{i}{j}" if i < 3 else f"prismatic{i}"

                motor = Motor(motor_id, ctrl_limits)
                aux = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, aux_name)
                parts.extend([motor, aux])
            arms.append(Arm(*parts))
        return Body(*arms)

    @staticmethod
    def run() -> None:
        np.set_printoptions(suppress=True)

        with open(Path.cwd() / "QuadrupedLocomotion" / "Model" / "ReachBot.Xml") as file:
            xml = file.read()
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        model_mass = model.body_subtreemass[0]
        print(f"Total mass of the model: {model_mass}")
        reachbot_ids = Simulation.__create_body(model)

        robot = Controller(model, data, reachbot_ids)

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
