from dynamixel_sdk import *
import numpy as np
import threading
import time

from control_table import *
from dynamixel_controller import DynamixelController

# Motor IDs
ROLL_MOTOR = 11
PITCH_MOTOR = 12
BOOM_MOTOR = 13

# Physical Robot Parameters
h = 0.135 # [m]
l = 0.016 # [m]
r = 0.063 # [m]
m = 0.1 # [kg]
g = 9.81 # [m/s^2]

# Initialize controller
controller = DynamixelController('COM3', 57600, 2.0)
group_sync_write = GroupSyncWrite(controller.port_handler, controller.packet_handler, GOAL_POSITION[0], GOAL_POSITION[1])

# Set Control Mode
controller.WRITE(ROLL_MOTOR, OPERATING_MODE, 4) # extended position control 
controller.WRITE(PITCH_MOTOR, OPERATING_MODE, 4) # extended position control 
controller.WRITE(BOOM_MOTOR, OPERATING_MODE, 4) # extended position control 

# Velocity/Accel Limits
controller.WRITE(ROLL_MOTOR, PROFILE_VELOCITY, 100)
controller.WRITE(PITCH_MOTOR, PROFILE_VELOCITY, 100)
controller.WRITE(BOOM_MOTOR, PROFILE_VELOCITY, 100)
controller.WRITE(ROLL_MOTOR, PROFILE_ACCELERATION, 50)
controller.WRITE(PITCH_MOTOR, PROFILE_ACCELERATION, 50)

# Torque Enable
controller.WRITE(ROLL_MOTOR, TORQUE_ENABLE,  1)
controller.WRITE(PITCH_MOTOR, TORQUE_ENABLE, 1)
controller.WRITE(BOOM_MOTOR, TORQUE_ENABLE,  1)

def sync_write_positions(ticks1, ticks2, ticks3):
    param_success = group_sync_write.addParam(ROLL_MOTOR, ticks1.to_bytes(4, 'little'))
    param_success &= group_sync_write.addParam(PITCH_MOTOR, ticks2.to_bytes(4, 'little'))
    param_success &= group_sync_write.addParam(BOOM_MOTOR, ticks3.to_bytes(4, 'little'))
    if not param_success:
        print("Failed to add parameters for SyncWrite")
        return False
    dxl_comm_result = group_sync_write.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print(f"SyncWrite communication error: {controller.packet_handler.getTxRxResult(dxl_comm_result)}")
        return False
    group_sync_write.clearParam()
    return True

def ticks_to_radians(ticks):
    return ticks / 4095 * 2 * np.pi
def radians_to_ticks(theta):
    return int(theta / (2 * np.pi) * 4095)

def IK(x, y, z):
    global h, r, l
    theta1 = np.arctan2(y, x)
    theta2 = -2*np.arctan2((np.sqrt(x**2 + y**2 + (z-h)**2 - r**2) - np.sqrt(x**2+y**2)), (z-h) - r)
    d3 = np.sqrt((np.sqrt(x**2+y**2) - r*np.sin(theta2))**2 + (z-h+r*np.cos(theta2))**2) - l 
    
    ticks1 = radians_to_ticks(theta1) + 1556
    ticks2 = radians_to_ticks(theta2) + 2780
    ticks3 = int((d3 - 0.2)/0.0000412 + 300)
    return ticks1, ticks2, ticks3

def FK(ticks1, ticks2, ticks3):
    global h, r, l
    theta1 = ticks_to_radians(ticks1 - 1556)
    theta2 = ticks_to_radians(ticks2 - 2780)
    d3 = (ticks3 - 300)*0.0000412 + 0.2

    x = np.cos(theta1)*np.cos(theta2)*(l+d3) - np.cos(theta1)*np.sin(theta2)*r
    y = np.sin(theta1)*np.cos(theta2)*(l+d3) - np.sin(theta1)*np.sin(theta2)*r
    z = h - np.sin(theta2)*(l+d3) - np.cos(theta2)*r
    return x, y, z

# goal_x = [0.2, 0.2, 0.2, 0.2, 0.2]
# goal_y = [-0.2, -0.2, 0.2, 0.2, -0.2]
# goal_z = [0.2, 0.6, 0.6, 0.2, 0.2]

goal_x = [0.2, 0.45, 0.55, 0.55, 0.2]
goal_y = [0, 0, 0 , 0, 0]
goal_z = [0.2, 0.2, 0.2, 0.4, 0.4]

x_min = 0.3
x_max = 0.49
x_steps = 200

z_min = 0.2
z_max = 0.3
z_steps = 100

x1 = np.linspace(x_min, x_max, x_steps)
z1 = z_min*np.ones(x_steps)

x2 = x_max*np.ones(z_steps)
z2 = np.linspace(z_min, z_max, z_steps)

x3 = x_max*np.ones(z_steps)
z3 = np.linspace(z_max, z_min, z_steps)

x4 = np.linspace(x_max, x_min, x_steps)
z4 = z_min*np.ones(x_steps)

X_TRAJ = np.concatenate((x1, x2, x3, x4))
Y_TRAJ = np.zeros(2*(x_steps + z_steps))
Z_TRAJ = np.concatenate((z1, z2, z3, z4))

# goal_x = [0.2, 0.433, 0.522, 0.522, 0.517, 0.517, 0.437, 
#           0.2, 0.437, 0.517, 0.517, 0.522, 0.522, 0.433]
# goal_y = [0, -0.115, -0.135, -0.135, 0.155, 0.155, 0.140,
#           0, 0.140, 0.155, 0.155, -0.135, -0.135, -0.115]
# goal_z = [0.15, 0.110, 0.110, 0.3, 0.3, 0.110, 0.110,
#           0.15, 0.110, 0.110, 0.3, 0.3, 0.110, 0.110]

def motion_control_loop():
    """
    Thread function for motion control loop.
    """
    ticks1, ticks2, ticks3 = IK(X_TRAJ[0], Y_TRAJ[0], Z_TRAJ[0])
    print(ticks1, ticks2, ticks3)
    # sync_write_positions(ticks1, ticks2, ticks3)
    time.sleep(5)
    while False:
        for x, y, z in zip(X_TRAJ, Y_TRAJ, Z_TRAJ):
            ticks1, ticks2, ticks3 = IK(x, y, z)
            sync_write_positions(ticks1, ticks2, ticks3)

            # ticks1_actual = controller.READ(ROLL_MOTOR, PRESENT_POSITION)
            # ticks2_actual = controller.READ(PITCH_MOTOR, PRESENT_POSITION)
            # ticks3_actual = controller.READ(BOOM_MOTOR, PRESENT_POSITION)
            # # x_fk, y_fk, z_fk = FK(ticks1_actual, ticks2_actual, ticks3_actual)
            # x_fk, y_fk, z_fk = FK(ticks1, ticks2, ticks3)
            # print(f"[FK] x:{x_fk:.3f}, y:{y_fk:.3f}, z:{z_fk:.3f}")

            time.sleep(0.025)

        # for x, y, z in zip(goal_x, goal_y, goal_z):
        #     ticks1, ticks2, ticks3 = IK(x, y, z)
        #     # print(ticks1, ticks2, ticks3)
        #     sync_write_positions(ticks1, ticks2, ticks3)
        #     time.sleep(0.1)  # time for Dynamixel to spin up
        #     while True:
        #         ticks1_actual = controller.READ(ROLL_MOTOR, PRESENT_POSITION)
        #         ticks2_actual = controller.READ(PITCH_MOTOR, PRESENT_POSITION)
        #         ticks3_actual = controller.READ(BOOM_MOTOR, PRESENT_POSITION)
        #         x, y, z = FK(ticks1_actual, ticks2_actual, ticks3_actual)
        #         print(f"x:{x:.3f}, y:{y:.3f}, z:{z:.3f}")
            
        #         moving_status = controller.READ(BOOM_MOTOR, MOVING)
        #         if moving_status == 0:  # Check if motor has stopped
        #             time.sleep(2)
        #             break  # Exit the inner loop and proceed to the next waypoint
        #         time.sleep(0.05)  # Small delay to avoid excessive reading

def feedback_loop():
    """
    Thread function for feedback loop.
    """
    while True:
        ticks1_actual = controller.READ(ROLL_MOTOR, PRESENT_POSITION)
        ticks2_actual = controller.READ(PITCH_MOTOR, PRESENT_POSITION)
        ticks3_actual = controller.READ(BOOM_MOTOR, PRESENT_POSITION)
        x, y, z = FK(ticks1_actual, ticks2_actual, ticks3_actual)
        print(f"x:{x:.3f}, y:{y:.3f}, z:{z:.3f}")
        time.sleep(0.1)  # Adjust feedback rate as needed

if __name__ == "__main__":
    try:
        motion_thread = threading.Thread(target=motion_control_loop)
        motion_thread.start()
        motion_thread.join()
        # feedback_thread = threading.Thread(target=feedback_loop)
        # feedback_thread.start()
        # feedback_thread.join()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user. Exiting...")
        controller.close_port()  # Ensure the port is closed safely
