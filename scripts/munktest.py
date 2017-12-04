import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.linalg import block_diag, norm


dim_x = 4
dim_z = 2

def assert_shape(ar, desired):
	actual = ar.shape
	msg = "Array needs shape %s but has %s" % (
		desired,
		actual,
	)
	assert actual == desired, msg

def const_vel_filter_2d(dt, pt, x_ndim=1, P_diag=(1., 1, 1, 1), R_std=1., Q_var=0.0001):
    """ helper, constructs 1d, constant velocity filter"""

    kf = KalmanFilter (dim_x=4, dim_z=2)

    kf.x = np.array([[pt[0], 0.0, pt[1], 0.]]).T
    kf.P *=  np.diag(P_diag)
    kf.F = np.array([[1, dt, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, dt],
                     [0, 0, 0, 1]])

    kf.H = np.array([[1., 0, 0,  0],
                     [0., 0, 1, 0]])

    kf.R *= np.eye(2) * (R_std**2)
    q = Q_discrete_white_noise(dim=2, dt=dt, var=Q_var)
    kf.Q = block_diag(q, q)

    return kf

f = const_vel_filter_2d(1000.0/30.0, (2, 2))

# f = KalmanFilter (dim_x=4, dim_z=2)

# f.x = np.array([2, 3, 0, 0]).reshape(dim_x, 1)


# # f.F = np.array([[1.,1.],
# #                 [0.,1.]])

# # f.H = np.array([[1.,0.]])
# f.F = np.matrix([
# 	[1, 0, 1, 0],
# 	[0, 1, 0, 1],
# 	[0, 0, 1, 0],
# 	[0, 0, 0, 1]
# ])

# f.H = np.matrix([
# 	[1, 0, 0, 0],
# 	[0, 1, 0, 0]
# ])


# f.P *= 1000

# f.R = np.array([[5., 5.],
# 				[5., 5.]])

# # f.Q = Q_discrete_white_noise(dim=4, dt=0.1, var=0.13)
# f.Q = np.random.rand(4, 4)

# assert_shape(f.x, (dim_x, 1))
# assert_shape(f.P, (dim_x, dim_x))
# assert_shape(f.Q, (dim_x, dim_x))
# assert_shape(f.R, (dim_z, dim_z))
# assert_shape(f.H, (dim_z, dim_x))
# assert_shape(f.F, (dim_x, dim_x))

f.predict()
for i in range(10):
	d = np.array([0.2 * i, 4]).reshape(2, 1)
	f.update(d)
	print f.x


