import numpy as np
np.set_printoptions(suppress=True)

from utilities import *

from tqdm import *

#referance https://github.com/George-Delagrammatikas/DukeMEMS-MotionPlanning/blob/master/jacobi.py



# Creating Gradient for the Newton Rapson method
def GetGradient(joints, DHTable):
    delta = 0.0001
    jac = np.zeros((16, joints.shape[0]))
    for i, joint in enumerate(joints):
        joints_m = joints.copy()
        joints_p = joints.copy()
        joints_m[i] -= delta
        joints_p[i] += delta
        Tm ,_ = FKinDHParam(joints_m, DHTable)
        Tp ,_ = FKinDHParam(joints_p, DHTable)
        jac[:, i] = (Tp - Tm).flatten() / (2 * delta)
    # print(jac)
    return jac

# Finding the joint angle using the ik solver
def ik(DHTable, T_tar, thetalist0, tolerance = 1e-17):
    step = 0.5
    joints = thetalist0.copy()
    for i in tqdm(range(10000)):
        T_cur,_ = FKinDHParam(joints, DHTable)  
        deltaT = (T_tar - T_cur).flatten()
        error = np.linalg.norm(deltaT)
        if error < tolerance:
            return joints, True
        jac = GetGradient(joints, DHTable)
        deltaq = jac.T@np.linalg.pinv((jac@jac.T)) @ deltaT


        # print(deltaT)
        # print(deltaq)
        joints = joints + step * deltaq
    return joints, False


