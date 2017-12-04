import rospy
import random
import string
import numpy as np
from scipy.optimize import linear_sum_assignment
from collections import deque
from munkres import Munkres, print_matrix, make_cost_matrix, DISALLOWED
from threading import Thread
from multiprocessing import Process, Value, Queue
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.linalg import block_diag, norm


class Track(object):
	def __init__(self, initial_pos, config):
		self.last_pos = initial_pos
		self.age = 1
		self.skipped = 0
		self.positions = deque([initial_pos], config["history"])
		self.id = ""
		for i in range(4):
			self.id += random.choice(string.ascii_letters)
		self.kf = self.const_vel_filter_2d(30.0/1000.0, initial_pos)

	def const_vel_filter_2d(self, dt, pt, x_ndim=1, P_diag=(1., 1, 1, 1), R_std=1., Q_var=0.0001):
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

	def pos(self):
		# return self.kf.x[0], self.kf.x[2]
		x, y = 0, 0
		for x_, y_ in self.positions:
			x += x_
			y += y_

		x /= len(self.positions)
		y /= len(self.positions)

		return x, y

	def observe(self, pos):
		self.last_pos = pos
		self.positions.append(pos)
		self.age += 1
		self.skipped = 0

		self.kf.predict()
		self.kf.update(np.array(list(pos)).reshape(2, 1))

	def missed(self):
		# self.age = 0
		self.kf.predict()
		self.skipped += 1

	def __repr__(self):
		return "[age: %d | skipped: %d]" % (self.age, self.skipped)

class Tracker(object):
	def __init__(self, config):
		self.config = config
		self.tracks = list()

	def filter_tracks(self, costs):
		removed_tracks = list()
		track_map = dict()

		for i, row in enumerate(costs):
			if all([c == DISALLOWED for c in row]):
				removed_tracks.append(i)

		for i in range(len(costs)):
			n_greater = 0
			for j in removed_tracks:
				if i > j:
					n_greater += 1

			track_map[i - n_greater] = i

		c = 0
		for i in removed_tracks:
			del costs[i - c]
			c += 1

		return track_map

	def filter_keys(self, costs):
		removed_keys = list()
		key_map = dict()

		for j in range(len(costs[0])):
			if all([costs[i][j]] == DISALLOWED for i in range(len(costs))):
				removed_keys.append(j)

		for j in range(len(costs[0])):
			n_greater = 0
			for k in removed_keys:
				if j > k:
					n_greater += 1

			key_map[j - n_greater] = j

		c = 0
		for j in removed_keys:
			for i in range(len(costs)):
				del costs[i][j - c]
				c += 1

		return key_map


	def calc_munkres(self, costs, result=None, ret=False):
		try:
			m = Munkres()
			assignments = m.compute(costs)
			if ret:
				return assignments
			result.put(assignments)
		except:
			if ret:
				return list()
			# rospy.loginfo("UNSOLVEABLE")			
			result.put(list())


	def get_assignments(self, costs):
		track_map, key_map, assignments = dict(), dict(), list()
		empty = track_map, key_map, assignments

		if len(costs) == 0 or len(costs[0]) == 0:
			return empty

		key_map = self.filter_keys(costs) 

		if len(costs) == 0 or len(costs[0]) == 0:
			return empty

		track_map = self.filter_tracks(costs)

		if len(costs) == 0 or len(costs[0]) == 0:
			return empty

		assignments = self.calc_munkres(costs, ret=True)

		return track_map, key_map, assignments

		# result = Queue()
		# p = Process(target=self.calc_munkres, args=(costs, result))
		# p.daemon = True
		# p.start()
		# p.join(10.0/1000.0)
		# if p.is_alive():
		# 	rospy.loginfo("HUNG THREAD")
		# 	p.terminate()
		# 	rospy.loginfo("TERMINATED")
		# 	return empty
		# else:
		# 	assignments = result.get()
		# 	return track_map, key_map, assignments

	def update(self, keypoints):
		n_before = len(self.tracks)
		n_after = len(keypoints)


		used_tracks = set()
		used_keys = set()

		costs = np.zeros((n_before, n_after)).tolist()
		for i, t in enumerate(self.tracks):
			for j, b in enumerate(keypoints):
				a = t.last_pos
				c = np.sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2))
				costs[i][j] = c if c < 10 else DISALLOWED


		track_map, key_map, assignments = self.get_assignments(costs)

		for track_i, key_i in assignments:
			track_i = track_map[track_i]
			key_i = key_map[key_i]
			used_tracks.add(track_i)
			used_keys.add(key_i)

			self.tracks[track_i].observe(keypoints[key_i])

		for i in range(len(self.tracks)):
			if i not in used_tracks:
				self.tracks[i].missed()

		for i in range(len(keypoints)):
			if i not in used_keys:
				new_t = Track(keypoints[i], self.config)
				self.tracks.append(new_t)

		# row_ind, col_ind = linear_sum_assignment(costs)

		# if n_before == n_after:
		# 	for track_i, key_i in enumerate(row_ind):
		# 		self.tracks[track_i].observe(keypoints[key_i])

		# elif n_before < n_after:
		# 	for track_i, key_i in enumerate(row_ind):
		# 		self.tracks[track_i].observe(keypoints[key_i])

		# 	for key_i in range(n_before, n_after):
		# 		new_t = Track(keypoints[key_i])
		# 		self.tracks.append(new_t)

		# elif n_before > n_after:
		# 	for key_i, track_i in enumerate(col_ind):
		# 		self.tracks[track_i].observe(keypoints[key_i])

		# 	for track_i in range(n_after, n_before):
		# 		self.tracks[track_i].skipped += 1


		self.tracks = filter(lambda t: t.skipped <= self.config["maxSkip"], self.tracks)
		old_enough = filter(lambda t: t.age >= self.config["minAge"], self.tracks)
		positions = map(lambda t: t.pos(), old_enough)
		ids = map(lambda t: t.id, old_enough)
		return positions, ids