import numpy as np
import math


rotor_center_dist = 0.2
arm_length = math.sqrt(2*(rotor_center_dist/2)*(rotor_center_dist/2))
print(arm_length)
thrust_coeff = 1 # torque to thrust, not velocity to thrust?


def compute_mixer(L, c):
    """
    Compute mixer for conventions used in the paper:
    +Y forward, +X right, motors at front(+Y)=1, left(-X)=2, back(-Y)=3, right(+X)=4
    L = arm length (m)
    c = km / kf (torque-to-thrust ratio)
    Returns A_eff (4x4) and M = inv(A_eff)
    """
    A_eff = np.array([
        [ 1.0,  1.0,  1.0,  1.0],   # T = sum fi
        [  L,   0.0,  -L,   0.0 ],   # tau_roll  = L*(f1 - f3)
        [ 0.0,   L,   0.0,  -L  ],   # tau_pitch = L*(f2 - f4)
        [ -c,    c,  -c,    c  ]    # tau_yaw = c*(-f1 + f2 - f3 + f4)
    ], dtype=float)

    M = np.linalg.inv(A_eff)
    return A_eff, M


(A_eff, M) = compute_mixer(arm_length, thrust_coeff)

print("Mixer matrix M:\n")
print("{")
for i in M:
    print("{", end="")
    for j in i[:-1]:
        print('{:f}'.format(j),end=",")
    print('{:f}'.format(i[-1]),end="},\n")
print("}")
print("\nCheck: M * A_eff should be identity:")

print(M @ A_eff)