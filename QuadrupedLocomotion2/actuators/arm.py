from .motor import Motor


class Arm:
    def __init__(self, motor1: Motor, revolute1, motor2: Motor, revolute2, motor3: Motor, prismatic):
        self.motor1 = motor1
        self.motor2 = motor2
        self.motor3 = motor3
        self.revolute1 = revolute1
        self.revolute2 = revolute2
        self.prismatic = prismatic
