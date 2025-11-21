import numpy as np

# ------ PARAMS ------
m = 1.2      # mass kg
g = 9.81    # gravity constant
Ix, Iy, Iz = 0.02, 0.02, 0.04 # moments of inertia
Ts = 0.018  # Loop Time Seconds

#These determine the agressiveness of the controller:
#Q: penalizes state error. Higher values correct more aggresively
Q = np.diag([
    50,50,50,      # position (x,y,z)
    30,30,30,      # velocity (x,y,z)
    80,80,80,      # angles (r,p,y)
    30,30,30       # angular velocities (r,p,y)
])
#R: penalizes control effort. Higher values reduce control effort, correcting less aggresively.
R = np.diag([1, 1, 1, 1]) * 1





# ------ MATHS (Don't change) ------
kT = 1.0/m
kphi = 1.0/Ix
ktheta = 1.0/Iy
kpsi = 1.0/Iz

A = np.zeros((12,12))
# position derivatives = velocity
A[0,3] = 1; A[1,4] = 1; A[2,5] = 1
# acceleration small-angle linearization
A[3,7] = g;   # ẍ ≈ g*θ
A[4,6] = -g;  # ÿ ≈ -g*φ
# rest (angular rates to angles)
A[6,9] = 1; A[7,10] = 1; A[8,11] = 1

B = np.zeros((12,4))
B[5,0] = kT       # vertical acceleration (z̈)
B[9,1] = kphi     # roll acceleration
B[10,2] = ktheta  # pitch acceleration
B[11,3] = kpsi    # yaw acceleration


from scipy.linalg import expm

n = A.shape[0]
m = B.shape[1]

M = np.zeros((n + m, n + m))
M[:n, :n] = A * Ts
M[:n, n:] = B * Ts
Md = expm(M)
Ad = Md[:n, :n]
Bd = Md[:n, n:]


from scipy.linalg import solve_discrete_are, inv

P = solve_discrete_are(Ad, Bd, Q, R)
K = inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)


print("K shape:", K.shape)
print("K MATRIX:")
print("{")
for i in K:
    print("{", end="")
    for j in i[:-1]:
        print('{:f}'.format(j),end=",")
    print('{:f}'.format(i[-1]),end="},\n")
print("}")

print("\n\nNEGATIVE K MATRIX:")
print("{")
for i in -K:
    print("{", end="")
    for j in i[:-1]:
        print('{:f}'.format(j),end=",")
    print('{:f}'.format(i[-1]),end="},\n")
print("}")