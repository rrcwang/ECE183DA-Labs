#!/usr/bin/python
import numpy as np

################
## Define constant environmental variables

# Walls
# 			 wall 3
# 		(0,L2)-----(L1,L2)
# 		|				|
# 		|				|
# wall 1|				| wall 2
# 		|				|
# 		(0,0)------(L1,0)
#     		wall 0
#
L1_BOX_WIDTH = 450 		# (mm)
L2_BOX_LENGTH = 630 	# (mm)
L3_PROBE_LENGTH = np.sqrt(L1_BOX_WIDTH**2 + L2_BOX_LENGTH**2)
WALL_CORNERS = {0: [[0, 0], [L1_BOX_WIDTH, 0]],
				1: [[0, 0], [0, L2_BOX_LENGTH]],
				2: [[L1_BOX_WIDTH, 0], [L1_BOX_WIDTH, L2_BOX_LENGTH]],
				3: [[0,L2_BOX_LENGTH], [L1_BOX_WIDTH, L2_BOX_LENGTH]] }

# Motor actuation model, simplified to include only the possible input cases
# MOTOR_ACTUATION = np.loadtxt('ac', delimiter=',')
# left turn 	(0, 0) + theta
# right turn 	(180, 180) - theta
# forward 		(0, 180)
# backwards 	(180, 0)
SATURATION_VAL = 5.07
MOTOR_ACTUATION = np.zeros([2,181]) # (rad/s)
MOTOR_ACTUATION[0,0] = SATURATION_VAL	# right motor 0
MOTOR_ACTUATION[0,180] = -SATURATION_VAL	# right motor 180
MOTOR_ACTUATION[1,0] = -SATURATION_VAL	# left motor 0
MOTOR_ACTUATION[1,180] = SATURATION_VAL	# left motor 180

# Robot dimensions
D1_WIDTH = 65 	# (mm) D1 distance between two wheels
D2_LENGTH = 95 	# (mm) D2 length of robot
D3_RADIUS = 20 	# (mm) D3 radius of wheel

TIME_SCALE = 0.100# (s)

################
# System matrices
# Covariance matrices of time constant additive white gaussian noise w, v

#	Dynamics propogation covariance
Q_COVAR = np.array([[10,	0,		0,					0],
			  		[0,		10,		0,					0],
				 	[0,		0,		0.154*TIME_SCALE**2,0],
				  	[0,		0,		0,					0.154]])


#	Measurement model covariance
R_COVAR = np.array([[2.56,	0,		0,		0],
			  		[0,		2.56,	0,		0],
			  		[0,		0,		1.52,	0],
			  		[0,		0,		0,		1.75]])

# F, Jacobian of dynamics propogation
F_JACOBIAN = np.array([[	1,	0,	0,	0],
					 [	0,	1,	0,	0],
					 [	0,	0,	1,	0],
					 [	0,	0,	0,	0]])

# C, Jacobian of dynamics propogation for input V(u_t) and phi(u_t),
# 	see equation (13) in lab 2 report
def get_C_jacobian(th):
	return np.array  ([[TIME_SCALE*np.cos(th),	0],
				  	   [TIME_SCALE*np.sin(th),	0],
				  	   [0,						TIME_SCALE],
				  	   [0,						1]])


################
# State class,
# 	contains the state information and transformation functions
class State:
	def __init__(self, xp, yp, th, dth):
		self.xp = xp	# x position
		self.yp = yp	# y position
		self.th = th	# compass heading, theta
		self.dth = dth	# rotational velocity, theta^dot

	# Update the state
	def update_state(self, args):
		self.update_state_s(*args)

	def update_state_s(self, new_xp, new_yp, new_th, new_dth):
		# check if new position is out of bounds,
		# 	raise exception if found
		if ((new_xp < 0) or (new_yp < 0) or
			(new_xp > L1_BOX_WIDTH) or (new_yp > L2_BOX_LENGTH)):
			raise Exception("Robot position out of box")

		# check if new heading is out of bounds,
		# 	correct if so
		if new_th >= 2*np.pi:
			new_th -= 2*np.pi
		if new_th < 0:
			new_th += 2*np.pi

		# update state
		self.xp = new_xp
		self.yp = new_yp
		self.th = new_th
		self.dth = new_dth


################
# StateEstimator class,
# 	calculates EKF update of some initial distirbution, provided in declaration
class StateEstimator:
	# Initialize state estimator with expected value of state
	def __init__(self, xp = L1_BOX_WIDTH/2,
								   yp = L2_BOX_LENGTH/2,
								   th = 0,
								   dth = 0 ):
		# Initialize with initial condition
		self.x_state_estimate = State(xp, yp, th, dth)

		# Initial covariance estimate, E[x0*x0^T],
		# We will use the assumption that the state estimate has:
		# 		1. Uncorrelated initial conditions
		# 		2. High uncertainty
		self.P_covar_estimate = np.eye(4)*9999

		# Keeps track of whether the state is a priori or a posteriori,
		# in case the Kalman filtering steps are taken out of order
		self.SE_is_a_priori = False

	# Get array of state values
	def get_state_val(self):
		return [self.x_state_estimate.xp, self.x_state_estimate.yp, self.x_state_estimate.th, self.x_state_estimate.dth]


	# For some a posteriori state estimate and input,
	# 	compute the next a priori state estimate
	# 	input to right, left motors
	def dynamics_propogation(self, input):

		if self.SE_is_a_priori:
			raise Exception("Steps computed out of order: dynamics_propogation was called when measurement_update was expected.")


		# Find linear and rotational velocities using the actuation model,
		#	depedent on the input to the motors
		lin_velocity = (MOTOR_ACTUATION[0, input[0]] + MOTOR_ACTUATION[0, input[1]]) * D3_RADIUS / 2 # (wr + wl)*R3
		rot_velocity = (MOTOR_ACTUATION[0, input[0]] - MOTOR_ACTUATION[0, input[1]]) / D1_WIDTH

		# Compute the next state estimate
		old_state_vals = np.array(self.get_state_val())
		#print(old_state_vals) # DEBUG

		next_state_vals = ( np.dot(F_JACOBIAN, old_state_vals)
							+ np.dot(get_C_jacobian(old_state_vals[2]), [lin_velocity, rot_velocity]) )

		#print(next_state_vals) # DEBUG
		# 	update the SE's state estimate
		self.x_state_estimate.update_state(next_state_vals)

		# Compute and update the next covariance estimate
		old_P_covar = self.P_covar_estimate
		self.P_covar_estimate = F_JACOBIAN @ old_P_covar @ F_JACOBIAN.transpose() + Q_COVAR

		# FLip to a priori
		self.SE_is_a_priori = True

		#print("a priori state estimate:")
		#print(self.get_state_val())
		#print(self.P_covar_estimate)

		return self.get_state_val()


	# Predict measurement based on current state TODO FIND BUG
	def measurement_model(self):
		state_vals = self.get_state_val()
		pos = np.array(state_vals[0:2]) 	# position

		#print(pos)

		# probe tips
		b_forward = np.array([np.cos(state_vals[2]), np.sin(state_vals[2])])
		b_right = np.array([np.sin(state_vals[2]), -np.cos(state_vals[2])])

		probe_forward = pos + L3_PROBE_LENGTH * b_forward
		probe_right = pos + L3_PROBE_LENGTH * b_right

		sensor_to_wall = [-1,-1] # [forward sensor, right sensor]

		# find out which wall each sensor is facing
		for i in range(0,4):
			wall_pts = WALL_CORNERS[i]

			# forward sensor
			if intersect(pos, probe_forward, wall_pts[0], wall_pts[1]):
				sensor_to_wall[0] = i
			# right sesnor
			if intersect(pos, probe_right, wall_pts[0], wall_pts[1]):
				sensor_to_wall[1] = i

		# an error has been encountered
		if (sensor_to_wall[0] == -1) or (sensor_to_wall[1] == -1):
			raise Exception("No wall was expected to be faced by a distance sensor")

		# calculate the distances that should be measured
		wall_line_forward = line(WALL_CORNERS[sensor_to_wall[0]][0], WALL_CORNERS[sensor_to_wall[0]][1])
		wall_line_right = line(WALL_CORNERS[sensor_to_wall[1]][0], WALL_CORNERS[sensor_to_wall[1]][1])

		wall_pt_forward = intersection_point(line(pos, probe_forward), wall_line_forward)
		wall_pt_right = intersection_point(line(pos, probe_right), wall_line_right)

		z_1 = np.linalg.norm(wall_pt_forward - pos) - D3_RADIUS/2
		z_2 = np.linalg.norm(wall_pt_right - pos) - D1_WIDTH/2

		return [z_1, z_2, state_vals[2], state_vals[3], sensor_to_wall, [wall_pt_forward, wall_pt_right]]


	# Calcualte the H jacobian for the given state
	def get_H_jacobian(self, predicted_measurement):
		SVs = self.get_state_val()

		h1 = np.zeros(4)
		h2 = np.zeros(4)

		sensor_to_wall = predicted_measurement[4]
		wall_pt_forward = predicted_measurement[5][0]
		wall_pt_right = predicted_measurement[5][1]

		# Forward sensor, first column of calibration
		h1 = [(SVs[0]-wall_pt_forward[0])/predicted_measurement[0],
			  (SVs[1]-wall_pt_forward[1])/predicted_measurement[0],
			  0,
			  0]


		# Right sensor, second columns of calirbation
		h2 = [(SVs[0]-wall_pt_right[0])/predicted_measurement[1],
			  (SVs[1]-wall_pt_right[1])/predicted_measurement[1],
			  0,
			  0]


		h3 = [0,0,1,0]
		h4 = [0,0,0,1]

		return np.stack((h1,h2,h3,h4),axis = -1)



	# For some a priori state estimate and measurement,
	# 	compute a posteriori state estimate
	def measurement_update(self, measurement):
		if not self.SE_is_a_priori:
			raise Exception("Steps computed out of order: measurement_update was called when dynamics_propogation was expected.")

		# compute residuals
		predicted_measurement = self.measurement_model()
		residuals = np.array(measurement) - np.array(predicted_measurement[0:4])
		print(predicted_measurement)
		#print(residuals)

		# compute H jacobian
		H_jacobian = self.get_H_jacobian(predicted_measurement)

		# compute residual covariance
		S_inv = np.linalg.inv(H_jacobian @ self.P_covar_estimate @ H_jacobian.transpose() + R_COVAR)

		# compute Kalman gain
		kalman_gain = self.P_covar_estimate @ H_jacobian.transpose() @ S_inv
		#print(kalman_gain)
		# update Kalman estimate and covariance estimate
		new_state_estimate = self.get_state_val() + np.dot(kalman_gain, residuals)
		#print(new_state_estimate)
		self.x_state_estimate.update_state(new_state_estimate)
		self.P_covar_estimate = (np.eye(4) - kalman_gain @ H_jacobian) @ self.P_covar_estimate

		# FLip to a posteriori
		self.SE_is_a_priori = False

		#print("a posteriori state estimate:")
		#print(self.get_state_val())
		#print(self.P_covar_estimate)

		return self.get_state_val()


################
# Line Segment Intersection Algorithm, credit to Byrce Boe (2006)
# https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
######
# returns 1:	if line segment AB intersects CD
# 		  0:	if line segment AB does not intersect CD
def ccw(A,B,C):
	return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])

def intersect(A,B,C,D):
	return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

# Given two points of a line segment,
# 	returns the coefficients of Ax+By=C
def line(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return A, B, -C

def intersection_point(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return np.array([x,y])
    else:
        return False


# Run the code

################
# Run the code.

# import data
data = np.loadtxt('\test_data\test_data_0.csv', delimiter=',', skiprows = 1, max_rows = 50) # TODO import measurement data

estimations_prior = np.zeros([30,4])
estimations_post = np.zeros([30,4])

test_model_input = data[:,1:3].astype(int)
test_measurements = data[:,3:]

# Run estimator with default initial conditions
SE = StateEstimator()
for t in range(0,30):
	estimations_prior[t,:] = SE.dynamics_propogation(test_model_input[t,:])[0:4]
	estimations_post[t,:] = SE.measurement_update(test_measurements[t,:])


np.savetxt("\results\prior_0.csv",estimations_prior, delimiter=',')
np.savetxt("\resultspost_0.csv",estimations_post, delimiter=',')
