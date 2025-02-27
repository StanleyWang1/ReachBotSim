import time

import mujoco
import mujoco.viewer
import numpy as np


class ReachBotController:
    """General class for control of ReachBot MuJoCo simulation."""
    
    def __init__(self, model, data, model_ids):
        self.model = model
        self.data = data
        self.model_ids = model_ids
        self.viewer = mujoco.viewer.launch_passive(model, data)

        self.timestep = 0.001  # adjust to make sim run slower/faster

    def simulate(self, steps, control_func=lambda : None):
        """
        Simulate for specified number of steps, with option to apply control function.
        """
        for _ in range(steps):
            control_func()
            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()
            time.sleep(self.timestep)
    
    def soft_simulate(self, steps, control_func=lambda : None):
        """
        Simulate for specified number of steps, with option to apply control function.
        """
        for i in range(steps):
            control_func(i, steps)
            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()
            time.sleep(self.timestep)

    def set_control(self, control_dict):
        """
        Send position control signal of actuators.
        """
        for actuator_id, value in control_dict.items():
            self.data.ctrl[actuator_id] = value
    
    def get_body_pose(self):
        """
        Get SE(3) pose of central robot body in world frame.
        """
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'mainBody')
        position = self.data.xpos[body_id]
        rotation_matrix = self.data.xmat[body_id].reshape(3, 3)
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = position
        return T

    def get_arm_end_effector_pose(self, arm_id):
        """
        Get SE(3) pose of specified arm end effector in world frame.
        """
        body_name = f"arm{arm_id}_end_effector"
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        position = self.data.xpos[body_id]
        rotation_matrix = self.data.xmat[body_id].reshape(3, 3)
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = position
        return T
    
    def get_arm_joint_variables(self, arm_id):
        arm = getattr(self.model_ids, f"arm{arm_id}")
        theta1 = self.data.qpos[self.model.jnt_qposadr[arm.revolute1]]
        theta2 = self.data.qpos[self.model.jnt_qposadr[arm.revolute2]]
        d3 = self.data.qpos[self.model.jnt_qposadr[arm.prismatic]]
        return theta1, theta2, d3
    
    def close_viewer(self):
        self.viewer.close()

        
    
    