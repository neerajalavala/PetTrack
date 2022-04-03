import math
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
def euler_to_quaternion(phi, theta, psi):

    qw = math.cos(phi/2) * math.cos(theta/2) * math.cos(psi/2) + math.sin(phi/2) * math.sin(theta/2) * math.sin(psi/2)
    qx = math.sin(phi/2) * math.cos(theta/2) * math.cos(psi/2) - math.cos(phi/2) * math.sin(theta/2) * math.sin(psi/2)
    qy = math.cos(phi/2) * math.sin(theta/2) * math.cos(psi/2) + math.sin(phi/2) * math.cos(theta/2) * math.sin(psi/2)
    qz = math.cos(phi/2) * math.cos(theta/2) * math.sin(psi/2) - math.sin(phi/2) * math.sin(theta/2) * math.cos(psi/2)

    return [qw, qx, qy, qz]

def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# RPY/Euler angles to Rotation Vector
def rotationMatrixToEulerAngles(R) :
    assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

def euler_to_rotVec(yaw, pitch, roll):
    # compute the rotation matrix
    Rmat = euler_to_rotMat(yaw, pitch, roll)

    theta = math.acos(((Rmat[0, 0] + Rmat[1, 1] + Rmat[2, 2]) - 1) / 2)
    sin_theta = math.sin(theta)
    if sin_theta == 0:
        rx, ry, rz = 0.0, 0.0, 0.0
    else:
        multi = 1 / (2 * math.sin(theta))
        rx = multi * (Rmat[2, 1] - Rmat[1, 2]) * theta
        ry = multi * (Rmat[0, 2] - Rmat[2, 0]) * theta
        rz = multi * (Rmat[1, 0] - Rmat[0, 1]) * theta
    return rx, ry, rz

def euler_to_rotMat(yaw, pitch, roll):
    Rz_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [          0,            0, 1]])
    Ry_pitch = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [             0, 1,             0],
        [-np.sin(pitch), 0, np.cos(pitch)]])
    Rx_roll = np.array([
        [1,            0,             0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]])
    # R = RzRyRx
    rotMat = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))
    return rotMat

def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z

def qv_mult(q1, v1):
    q2 = (0.0,) + v1
    print(q2)
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]
roll = 2.6335
pitch = 0.4506
yaw = 1.1684

print("roll = ", roll)
print("pitch = ", pitch)
print("yaw = ", yaw)
print("")
quat = euler_to_quaternion(yaw, pitch, roll)
v2 = qv_mult(quat, (1,0,0))
v3 = qv_mult(quat, (0,1,0))
v4 = qv_mult(quat, (0,0,1))
print(np.round(v2, decimals=2))
print(np.round(v3, decimals=2))
print(np.round(v4, decimals=2))

import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# Cartesian axes
ax.quiver(-1, 0, 0, 3, 0, 0, color='#aaaaaa',linestyle='dashed')
ax.quiver(0, -1, 0, 0,3, 0, color='#aaaaaa',linestyle='dashed')
ax.quiver(0, 0, -1, 0, 0, 3, color='#aaaaaa',linestyle='dashed')

ax.quiver(0, 0, 0, 1, 0, 0, color='b')
ax.quiver(0, 0, 0, 0, 1, 0, color='b')
ax.quiver(0, 0, 0, 0, 0, 1, color='b')

ax.quiver(0, 0, 0, -0.79, 0.44, -0.44, color='c')
ax.quiver(0, 0, 0, -0.54, -0.15 , 0.83, color='m')
ax.quiver(0, 0, 0, 0.3, 0.89, 0.35, color='y')

ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])
ax.set_zlim([-1.5, 1.5])
plt.show()

#print rx, ry, rz