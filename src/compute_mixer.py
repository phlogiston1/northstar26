import numpy as np
import math


rotor_center_dist = 0.2
arm_length = math.sqrt(2*(rotor_center_dist/2)*(rotor_center_dist/2))
print(arm_length)
thrust_coeff = 0.01


def compute_mixer(L, c):
    """
    Compute the quadcopter mixer for the paper's coordinate frame:
    +X forward, +Y right, motors on ±X and ±Y axes.

    L = arm length (meters)
    c = torque-to-thrust ratio = k_m / k_f
    """

    # Actuator effectiveness matrix:
    # [ T
    #   τφ
    #   τθ
    #   τψ ] = A_eff * [f1 f2 f3 f4]^T
    A_eff = np.array([
        [ 1,  1,  1,  1],     # total thrust
        [ L,  0, -L,  0],     # roll torque (about X)
        [ 0, -L,  0,  L],     # pitch torque (about Y)
        [-c,  c, -c,  c]      # yaw torque (motor spin directions)
    ], dtype=float)

    # Mixer is inverse of effectiveness matrix
    M = np.linalg.inv(A_eff)
    return M


M = compute_mixer(arm_length, thrust_coeff)

print("Mixer matrix M:\n")
print("{")
for i in M:
    print("{", end="")
    for j in i[:-1]:
        print('{:f}'.format(j),end=",")
    print('{:f}'.format(i[-1]),end="},\n")
print("}")
print("\nCheck: M * A_eff should be identity:")
A_eff = np.array([
    [ 1,  1,  1,  1],
    [ arm_length,  0, -arm_length,  0],
    [ 0, -arm_length,  0,  arm_length],
    [-thrust_coeff,  thrust_coeff, -thrust_coeff,  thrust_coeff]
])
print(M @ A_eff)