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
