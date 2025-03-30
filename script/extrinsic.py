import numpy as np
from scipy.spatial.transform import Slerp, Rotation as R

b_T_L0 = np.array([[1.0,  0.0,  0.0, -0.050],
                   [0.0,  1.0,  0.0,  0.000],
                   [0.0,  0.0,  1.0,  0.055],
                   [0.0,  0.0,  0.0,  1.000]])
b_T_L1 = np.array([[-1.0,  0.0,  0.0, -0.550],
                   [0.0,  0.0,  1.0,  0.030],
                   [0.0,  1.0,  0.0,  0.050],
                   [0.0,  0.0,  0.0,  1.000]])
L0_T_L1 = np.dot(np.linalg.inv(b_T_L0), b_T_L1)
b_q_L0 = R.from_matrix(b_T_L0[:3, :3]).as_quat()
L0_q_L1 = R.from_matrix(L0_T_L1[:3, :3]).as_quat()
print('extrinsic_T is: ', b_T_L0[:3,3], L0_T_L1[:3,3])
print('extrinsic_R is: ', np.array([[b_q_L0[3], b_q_L0[0], b_q_L0[1], b_q_L0[2]],
                                    [L0_q_L1[3], L0_q_L1[0], L0_q_L1[1], L0_q_L1[2]]]))
