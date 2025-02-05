import numpy as np

def forward_kinematics(theta1: float, theta2: float, d3: float) -> np.ndarray:
    l = 0.016  # in meters
    r = 0.063  # in meters
    h = 0.080 + 0.090  # in meters

    c1 = np.cos(theta1)
    s1 = np.sin(theta1)
    c2 = np.cos(theta2)
    s2 = np.cos(theta2)

    return np.array([
        [c1 * c2, -s1, c1 * s2, c1 * c2 * (l + d3) - c1 * s2 * r],
        [s1 * c2, c1, s1 * s2, s1 * c2 * (l + d3) - s1 * s2 * r],
        [-s2, 0, c2, -s2 * (l + d3) - c2 * r + h],
        [0, 0, 0, 1]
    ])
