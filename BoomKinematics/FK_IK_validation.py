import numpy as np

# Physical Robot Parameters
h = 0.135 # [m]
l = 0.016 # [m]
r = 0.063 # [m]
m = 0.1 # [kg]
g = 9.81 # [m/s^2]

def ticks_to_radians(ticks):
    return ticks / 4095 * (2 * np.pi)
def radians_to_ticks(theta):
    return int(theta / (2 * np.pi) * 4095)

def IK(x, y, z):
    global h, r, l
    theta1 = np.arctan2(y, x)
    theta2 = -2*np.arctan2((np.sqrt(x**2 + y**2 + (z-h)**2 - r**2) - np.sqrt(x**2+y**2)), ((z-h) - r))
    d3 = np.sqrt((np.sqrt(x**2+y**2) - r*np.sin(theta2))**2 + (z-h+r*np.cos(theta2))**2) - l 
    # d3 = np.sqrt(x**2 + y**2 + (z-h)**2 - r**2) - l
    # theta2 = np.arctan2(r, d3 + l) + np.arctan2(z-h, np.sqrt(x**2 + y**2))

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

x_des = 0.5
y_des = 0.3
z_des = 0.2

ticks1, ticks2, ticks3 = IK(x_des, y_des, z_des)
x_pred, y_pred, z_pred = FK(ticks1, ticks2, ticks3)

print(ticks1, ticks2, ticks3)
print(x_pred, y_pred, z_pred)
