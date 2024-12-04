# Stanley Wang
# Symbolic Calculation of Prismatic Manipulator Kinematics/Dynamics

import sympy as sp

# -------------------- SETUP --------------------
# Jointspace Symbols
theta1, theta2, d3 = sp.symbols('theta1 theta2 d3')
jointspace = sp.Matrix([theta1, theta2, d3])
# Robot Geometry Symbols
h, l, r = sp.symbols('h l r')
# Define Trig Functions
c1 = sp.cos(theta1)
s1 = sp.sin(theta1)
c2 = sp.cos(theta2)
s2 = sp.sin(theta2)

# -------------------- FORWARD KINEMATICS --------------------
# Joint Transformations
T_0_1 = sp.Matrix([
    [c1, -s1, 0, 0],
    [s1, c1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
T_1_2 = sp.Matrix([
    [c2, 0, s2, 0],
    [0, 1, 0, 0],
    [-s2, 0, c2, h],
    [0, 0, 0, 1]
])
T_2_3 = sp.Matrix([
    [1, 0, 0, l + d3],
    [0, 1, 0, 0],
    [0, 0, 1, -r],
    [0, 0, 0, 1]
])
# Forward Kinematics
T_0_3 = sp.simplify(T_0_1 * T_1_2 * T_2_3)

# -------------------- JACOBIANS --------------------
# Linear Velocity Jacobian
ee_position = T_0_3[:3, 3]
Jv = sp.simplify(sp.Matrix.hstack(*[sp.diff(ee_position, var) for var in jointspace]))
# Angular Velocity Jacobian
Jw = sp.Matrix([
    [0, -s1, 0],
    [0, c1, 0],
    [1, 0, 0]
])

# -------------------- MASS MATRIX --------------------
# Inertia of End Effector (point mass)
m = sp.symbols('m')
ee_position_outer_product = ee_position * ee_position.T
I_ee = m * (sp.Matrix.eye(3) * ee_position.dot(ee_position) - ee_position_outer_product)
# Mass Matrix
M_matrix = sp.simplify(m * Jv.T * Jv + Jw.T * I_ee * Jw)

# -------------------- CORIOLIS MATRIX --------------------
# Generalized coordinates/velocities
q = jointspace
q_dot = sp.Matrix([sp.Symbol(f'{str(var)}_dot') for var in q])
# Coriolis matrix
C_matrix = sp.zeros(len(q), len(q))
for i in range(len(q)):
    for j in range(len(q)):
        C_ij = 0
        for k in range(len(q)):
            C_ij += (1/2) * (
                sp.diff(M_matrix[i, j], q[k]) +
                sp.diff(M_matrix[i, k], q[j]) -
                sp.diff(M_matrix[j, k], q[i])
            ) * q_dot[k]
        C_matrix[i, j] = sp.simplify(C_ij)

# -------------------- GRAVITY VECTOR --------------------
g = sp.symbols('g')  
ee_z = T_0_3[2, 3]            
U = m * g * ee_z       
g_vec = sp.Matrix([sp.diff(U, var) for var in jointspace])

# -------------------- PACKAGE JOINT DYNAMICS AS LAMBDA FUNCTIONS --------------------
# Bring in physical values for robot parameters
h_real = 0.080 # [m]
l_real = 0.016 # [m]
r_real = 0.063 # [m]
m_real = 0.1 # [kg]
g_real = 9.81 # [m/s^2]
physical_subs = {h:h_real, l:l_real, r:r_real, m:m_real, g:g_real}

M_matrix_sub = M_matrix.subs(physical_subs)
C_matrix_sub = C_matrix.subs(physical_subs)
g_vec_sub = g_vec.subs(physical_subs)

# lambdify the sympy functions so they can be evaluated with real joint values
