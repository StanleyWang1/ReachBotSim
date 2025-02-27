from dynamixel_sdk import *
import numpy as np
from pynput import keyboard
import time

from control_table import *
from dynamixel_controller import DynamixelController

# Motor IDs
ROLL_MOTOR = 11
PITCH_MOTOR = 12
BOOM_MOTOR = 13
WRIST_PITCH = 14
WRIST_ROLL = 15
GRIPPER = 16

# Initialize controller
controller = DynamixelController('COM3', 57600, 2.0)
group_sync_write = GroupSyncWrite(controller.port_handler, controller.packet_handler, GOAL_POSITION[0], GOAL_POSITION[1])

# Set Control Mode
controller.WRITE(ROLL_MOTOR, OPERATING_MODE, 4) # extended position control 
controller.WRITE(PITCH_MOTOR, OPERATING_MODE, 4) # extended position control 
controller.WRITE(BOOM_MOTOR, OPERATING_MODE, 4) # extended position control 
controller.WRITE(WRIST_PITCH, OPERATING_MODE, 4) # extended position control 
controller.WRITE(WRIST_ROLL, OPERATING_MODE, 4) # extended position control 
controller.WRITE(GRIPPER, OPERATING_MODE, 4) # extended position control 

# Velocity/Accel Limits
# controller.WRITE(ROLL_MOTOR, PROFILE_VELOCITY, 20)
# controller.WRITE(PITCH_MOTOR, PROFILE_VELOCITY, 100)
# controller.WRITE(BOOM_MOTOR, PROFILE_VELOCITY, 100)
# controller.WRITE(ROLL_MOTOR, PROFILE_ACCELERATION, 20)
# controller.WRITE(PITCH_MOTOR, PROFILE_ACCELERATION, 100)
controller.WRITE(GRIPPER, PWM_LIMIT, 250)

# Torque Enable
controller.WRITE(ROLL_MOTOR, TORQUE_ENABLE,  1)
controller.WRITE(PITCH_MOTOR, TORQUE_ENABLE, 1)
controller.WRITE(BOOM_MOTOR, TORQUE_ENABLE,  1)
controller.WRITE(WRIST_PITCH, TORQUE_ENABLE,  1)
controller.WRITE(WRIST_ROLL, TORQUE_ENABLE, 1)
controller.WRITE(GRIPPER, TORQUE_ENABLE,  1)

def sync_write_positions(ticks1, ticks2, ticks3, ticks4, ticks5, ticks6):
    param_success = group_sync_write.addParam(ROLL_MOTOR, ticks1.to_bytes(4, 'little'))
    param_success &= group_sync_write.addParam(PITCH_MOTOR, ticks2.to_bytes(4, 'little'))
    param_success &= group_sync_write.addParam(BOOM_MOTOR, ticks3.to_bytes(4, 'little'))
    param_success = group_sync_write.addParam(WRIST_PITCH, ticks4.to_bytes(4, 'little'))
    param_success &= group_sync_write.addParam(WRIST_ROLL, ticks5.to_bytes(4, 'little'))
    param_success &= group_sync_write.addParam(GRIPPER, ticks6.to_bytes(4, 'little'))
    if not param_success:
        print("Failed to add parameters for SyncWrite")
        return False
    dxl_comm_result = group_sync_write.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print(f"SyncWrite communication error: {controller.packet_handler.getTxRxResult(dxl_comm_result)}")
        return False
    group_sync_write.clearParam()
    return True

def on_press(key):
    global ticks1, ticks2, ticks3, ticks4, ticks5, ticks6
    try:
        if key.char == 'a':
            ticks1 += 3
        elif key.char == 'd':
            ticks1 -= 3
        elif key.char == 's':
            ticks2 += 3
        elif key.char == 'w':
            ticks2 -= 3
        elif key.char == 'e':
            ticks3 += 50
        elif key.char == 'r':
            ticks3 -= 50
        elif key.char == 'k':
            ticks4 += 10
        elif key.char == 'i':
            ticks4 -= 10
        elif key.char == 'l':
            ticks5 += 10
        elif key.char == 'j':
            ticks5 -= 10
        elif key.char == 'p':
            ticks6 += 10
        elif key.char == 'o':
            ticks6 -= 10
    except AttributeError:
        pass

if __name__ == "__main__":
    ticks1 = controller.READ(ROLL_MOTOR, PRESENT_POSITION)
    ticks2 = controller.READ(PITCH_MOTOR, PRESENT_POSITION)
    ticks3 = controller.READ(BOOM_MOTOR, PRESENT_POSITION)
    ticks4 = controller.READ(WRIST_PITCH, PRESENT_POSITION)
    ticks5 = controller.READ(WRIST_ROLL, PRESENT_POSITION)
    ticks6 = controller.READ(GRIPPER, PRESENT_POSITION)
    
    # READ KEYBOARD INPUTS AND MODIFY TICKS
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    while True: 
        sync_write_positions(ticks1, ticks2, ticks3, ticks4, ticks5, ticks6)