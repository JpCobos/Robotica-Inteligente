import numpy as np
import matplotlib.pyplot as plt

def trotz(theta, x, y, z):
    s = np.sin(theta)
    c = np.cos(theta)
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

def graph_frames(name, T1, T2):
      scale = 10
      limit = 100
      fig = plt.figure()
      ax = fig.add_subplot(111, projection='3d')
      ax.quiver(0, 0, 0, scale, 0, 0, color = 'r', arrow_length_ratio = 0.1)
      ax.quiver(0, 0, 0, 0, scale, 0, color = 'g', arrow_length_ratio = 0.1)
      ax.quiver(0, 0, 0, 0, 0, scale, color = 'b', arrow_length_ratio = 0.1)
      ax.quiver(T1[0][3], T1[1][3], T1[2][3], scale * T1[0][0], scale * T1[1][0], scale * T1[2][0], color = 'r', arrow_length_ratio = 0.1)
      ax.quiver(T1[0][3], T1[1][3], T1[2][3], scale * T1[0][1], scale * T1[1][1], scale * T1[2][1], color = 'g', arrow_length_ratio = 0.1)
      ax.quiver(T1[0][3], T1[1][3], T1[2][3], scale * T1[0][2], scale * T1[1][2], scale * T1[2][2], color = 'b', arrow_length_ratio = 0.1)
      ax.quiver(T2[0][3], T2[1][3], T2[2][3], scale * T2[0][0], scale * T2[1][0], scale * T2[2][0], color = 'r', arrow_length_ratio = 0.1)
      ax.quiver(T2[0][3], T2[1][3], T2[2][3], scale * T2[0][1], scale * T2[1][1], scale * T2[2][1], color = 'g', arrow_length_ratio = 0.1)
      ax.quiver(T2[0][3], T2[1][3], T2[2][3], scale * T2[0][2], scale * T2[1][2], scale * T2[2][2], color = 'b', arrow_length_ratio = 0.1)
      ax.set_xlim([-limit/2, limit/2])
      ax.set_ylim([-limit, limit])
      ax.set_zlim([-limit/2, limit/2])
      plt.title(name)

def FK2DRR(L, q):
    theta = 0
    x = 0
    y = 0
    for i in range(len(L)):
        theta += q[i]
        x += L[i] * np.cos(theta)
        y += L[i] * np.sin(theta)
    P = [x, y]
    return P
   
def fkine(theta_offset, d_offset, alpha, a, q, joint_type):
    FK = np.identity(4)
    for i in range(len(q)):
        if joint_type[i] == "R":
            theta = theta_offset[i] + q[i]
            d = d_offset[i]
        elif joint_type[i] == "P":
            theta = theta_offset[i]
            d = d_offset[i] + q[i]
        cz = np.cos(theta)
        sz = np.sin(theta)
        cx = np.cos(alpha[i])
        sx = np.sin(alpha[i])
        R1 = np.array([[cz, -sz, 0, 0],
                       [sz, cz, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
        R2 = np.array([[1, 0, 0, 0],
                       [0, cx, -sx, 0],
                       [0, sx, cx, 0],
                       [0, 0, 0, 1]])
        T1 = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, d],
                       [0, 0, 0, 1]])
        T2 = np.array([[1, 0, 0, a[i]],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
        FK = FK @ (R1 @ T1 @ T2 @ R2)
    return FK

def rot2rpyfull(T):
    R = T[0:3, 0:3]
    sy = np.sqrt(R[0,0] * 2 + R[1,0] * 2)
    singular = sy < 1e-6
    if not singular:
        x1 = np.arctan2(R[2,1], R[2,2])
        y1 = np.arctan2(-R[2,0], sy)
        z1 = np.arctan2(R[1,0], R[0,0])
        x2 = np.arctan2(-R[2,1], -R[2,2])
        y2 = np.arctan2(-R[2,0], -sy)
        z2 = np.arctan2(-R[1,0], -R[0,0])
        return [[x1, y1, z1], [x2, y2, z2]]
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
        return [[x, y, z], None]

print("\t\tRobot RR (Case 1)")
L = [20, 30]
q = np.deg2rad([80, 40])
P = FK2DRR(L, q)
print("lengths (mm):\n", L, "\njoint configuration (deg):\n", np.rad2deg(q), "\nend coordinates:\n", np.round(P, 4))
v = FK2DRR(L[0:1], q[0:1])
T1 = trotz(q[0], v[0], v[1], 0)
T2 = trotz(q[0] + q[1], P[0], P[1], 0)
graph_frames("1", T1, T2)

print("\n\t\tRobot RR (Case 2)")
L = [50, 40]
q = np.deg2rad([0, 90])
P = FK2DRR(L, q)
print("lengths (mm):\n", L, "\njoint configuration (deg):\n", np.rad2deg(q), "\nend coordinates:\n", P)

print("\n\t\tRobot RP")
theta_offset = np.transpose(np.deg2rad([0, 0]))
d_offset = np.transpose([50, 80])
alpha = np.transpose(np.deg2rad([90, 0]))
a = np.transpose([0, 40])
q = [0, 0]
joint_type = ["R", "P"]
FK = fkine(theta_offset, d_offset, alpha, a, q, joint_type)
print("joint configuration (deg, mm):\n", q, "\nforward kinematics (fkine):\n", np.round(FK, 6))
RZ1 = np.array([[np.cos(0), -np.sin(0), 0, 0], [np.sin(0), np.cos(0), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
RX1 = np.array([[1, 0, 0, 0], [0, np.cos(np.deg2rad(90)), -np.sin(np.deg2rad(90)), 0], [0, np.sin(np.deg2rad(90)), np.cos(np.deg2rad(90)), 0], [0, 0, 0, 1]])
TZ1 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 50], [0, 0, 0, 1]])
TX1 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
T_0_1 = RZ1 @ TZ1 @ TX1 @ RX1
RZ2 = np.array([[np.cos(0), -np.sin(0), 0, 0], [np.sin(0), np.cos(0), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
RX2 = np.array([[1, 0, 0, 0], [0, np.cos(0), -np.sin(0), 0], [0, np.sin(0), np.cos(0), 0], [0, 0, 0, 1]])
TZ2 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 80], [0, 0, 0, 1]])
TX2 = np.array([[1, 0, 0, 40], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
T_1_2 = RZ2 @ TZ2 @ TX2 @ RX2
T_0_2 = T_0_1 @ T_1_2
print("forward kinematics (hard code):\n", np.round(T_0_2, 6))
graph_frames("2", T_0_1, T_0_2)

print("\n\t\tRobot Ufactory Lite6")
theta_offset = np.transpose(np.deg2rad([0, -90, -90, 0, 0, 0]))
d_offset = np.transpose([243.3, 0.0, 0.0, 227.6, 0.0, 61.5])
alpha = np.transpose(np.deg2rad([-90, 180, 90, 90, -90, 0]))
a = np.transpose([0.0, 200.0, 87.0, 0.0, 0.0, 0.0])
q = np.deg2rad([0.0, 9.7, 33.4, 0.0, -21.5, 0.0])
joint_type = ["R", "R", "R", "R", "R", "R"]
FK = fkine(theta_offset, d_offset, alpha, a, q, joint_type)
solutions = rot2rpyfull(FK)
print("joint configuration (deg):\n", np.rad2deg(q), "\nforward kinematics:\n", np.round(FK, 6), "\nRPY solution 1 (deg):", np.round(np.rad2deg(solutions[0]), 6))
if solutions[1]:
    print("RPY solution 2 (deg):", np.round(np.rad2deg(solutions[1]), 6))
else:
    print("no second solution")

plt.show()