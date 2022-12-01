import numpy as np
import quaternion

def quaternion_from_array(float_array):
    assert len(float_array) == 4
    float_array = np.array(float_array)
    q = float_array / (np.linalg.norm(float_array) + 1e-5)
    return quaternion.from_float_array(q)


def rotation_matrix_from_array(float_array):
    q = quaternion_from_array(float_array)
    R = quaternion.as_rotation_matrix(q)
    return R

# x is a b c d not normal vector
def project(R, T, x):
    Rx = R @ x
    Rx_norm = np.linalg.norm(Rx, axis=0)
    return (np.dot(T, Rx) / (Rx_norm ** 2) + 1) * Rx

def trans(R0, T0, R1, T1, RD, TD):
    R0 = rotation_matrix_from_array(R0)
    R1 = rotation_matrix_from_array(R1)
    RD = rotation_matrix_from_array(RD)

    RD0 = np.column_stack((R0, T0.T))
    RD1 = np.insert(R1, 3, values = T1.T, axis = 1)
    RDD = np.insert(RD, 3, values = TD, axis = 1)

    RD0 = np.insert(RD0, 3, values = np.asarray([0, 0, 0, 1]), axis = 0)
    RD1 = np.insert(RD1, 3, values = np.asarray([0, 0, 0, 1]), axis = 0)
    RDD = np.insert(RDD, 3, values = np.asarray([0, 0, 0, 1]), axis = 0)

    return np.linalg.inv(RD1) - RDD @ np.linalg.inv(RD0), RD0, RD1, RDD
