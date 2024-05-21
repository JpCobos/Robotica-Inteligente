import numpy as np

def rot2rpy(R):
    yaw = np.arctan2(R[1, 0],R[0, 0])
    pitch = np.arctan2(-R[2, 0],R[0, 0]*np.cos(yaw)+R[1, 0]*np.sin(yaw))
    roll = np.arctan2(-R[1, 2]*np.cos(yaw)+R[0, 2]*np.sin(yaw),R[1, 1]*np.cos(yaw)-R[0, 1]*np.sin(yaw))
    return [roll, pitch, yaw]

def T2DH(T):
    rpy = rot2rpy(T[0:3, 0:3])
    d = T[2][3]
    a = T[0:3, 0] @ T[0:3, 3]
    return [np.rad2deg(rpy[2]), d, np.rad2deg(rpy[0]), a]

print("\t\tRobot RP\nDH parameters\n[theta, d, alpha, a]:")
T = np.append(np.transpose([[1, 0, 0], [0, 0, 1], [0, -1, 0], [0, 0, 50]]), np.array([[0, 0, 0, 1]]), axis = 0)
print(np.round(T2DH(T), 4))
T = np.append(np.transpose([[1, 0, 0], [0, 1, 0], [0, 0, 1], [40, 0, 80]]), np.array([[0, 0, 0, 1]]), axis = 0)
print(np.round(T2DH(T), 4))

print("\n\t\tRobot Ufactory Lite6\nDH parameters\n[theta, d, alpha, a]:")
T = np.append(np.transpose([[1, 0, 0], [0, 0, -1], [0, 1, 0], [0, 0, 243.3]]), np.array([[0, 0, 0, 1]]), axis = 0)
print(np.round(T2DH(T), 4))
T = np.append(np.transpose([[0, -1, 0], [-1, 0, 0], [0, 0, -1], [0, -200, 0]]), np.array([[0, 0, 0, 1]]), axis = 0)
print(np.round(T2DH(T), 4))
T = np.append(np.transpose([[0, -1, 0], [0, 0, 1], [-1, 0, 0], [0, -87, 0]]), np.array([[0, 0, 0, 1]]), axis = 0)
print(np.round(T2DH(T), 4))
T = np.append(np.transpose([[1, 0, 0], [0, 0, 1], [0, -1, 0], [0, 0, 227.6]]), np.array([[0, 0, 0, 1]]), axis = 0)
print(np.round(T2DH(T), 4))
T = np.append(np.transpose([[1, 0, 0], [0, 0, -1], [0, 1, 0], [0, 0, 0]]), np.array([[0, 0, 0, 1]]), axis = 0)
print(np.round(T2DH(T), 4))
T = np.append(np.transpose([[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, 61.5]]), np.array([[0, 0, 0, 1]]), axis = 0)
print(np.round(T2DH(T), 4))