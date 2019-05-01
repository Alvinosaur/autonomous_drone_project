import numpy as np
from matplotlib import pyplot as plt
import random


def predict_state(X, A, G, a, w):
	X_pred = A.dot(X) + G.dot(a) + w
	return X_pred


def predict_cov(P, A, Q):
	P_pred = A.dot(P.dot(A.T)) + Q
	return P_pred


def update(new_meas, x_pred, P_pred, H, R):
	# print(P_pred, type(P_pred))
	# print(x_pred)
	S = (H.dot(P_pred.dot(H.T)) + R)  # 1 x 1  

	K_gain = (P_pred.dot(H.T)).dot(1/S)

	innov = new_meas - H.dot(x_pred)  # measurement is equal to state, so no no conversion

	x_new = x_pred + K_gain*innov

	P_new = P_pred - K_gain.dot(H.dot(P_pred))

	return x_new, P_new


def get_cov(sigma1, sigma2):
	return np.array([[sigma1**2, sigma1*sigma2], 
					 [sigma2*sigma1, sigma2**2]])


def init_P(use_initial_error=False, x_obs_error=0, v_obs_error=0):
	if (use_initial_error):
		return get_cov(x_obs_error, v_obs_error)
	else:
		return np.eye(2)


def generate_noisy_data(x_0, v_0, a, time_steps, position_noise, velocity_noise):
	x_obs = [0 for i in range(time_steps)]
	v_obs = [0 for i in range(time_steps)]

	x_actual =  [0 for i in range(time_steps)]
	v_actual = [0 for i in range(time_steps)]

	x_actual[0], v_actual[0] = x_0, v_0
	x_obs[0], v_obs[0] = x_0, v_0
	t = 0.5

	for i in range(1, time_steps):

		x_actual[i] = x_actual[i-1] + v_actual[i-1] * t
		v_actual[i] = v_actual[i-1] + a * t

		x_obs[i] = x_actual[i] + (random.random() - 0.5) * position_noise
		v_obs[i] = v_actual[i] + (random.random() - 0.5) * velocity_noise


		# x_obs[i] = x_obs[i-1] + v_obs[i-1] * t + (random.random() - 0.5) * noise_scale
		# v_obs[i] = v_obs[i-1] + a * t + (random.random() - 0.5) * noise_scale

	return x_actual, v_actual, x_obs, v_obs

def plot_graphs(time, x_actual, v_actual, x_obs, v_obs):
	plt.subplot(2, 1, 1)
	plt.plot(time, x_actual, label='Actual', color='r')
	plt.plot(time, x_obs, label='Measured', color='g')
	plt.ylabel('Position (m)')
	plt.title('Measured(GREEN) and Actual(RED) Position')
	

	plt.subplot(2, 1, 2)
	plt.plot(time, v_actual, label='Actual', color='r')
	plt.plot(time, v_obs, label='Measured', color='g')
	plt.ylabel('Velocity (m/s)')
	plt.title('Measured(GREEN) and Actual(RED) Velocity')

	plt.show()


def main(x_obs, v, a):
	time_axis = [i for i in range(len(x_obs))]

	# Assume zero-mean white Gaussian process and measurement noise
	w, z = 0, 0

	# Constants
	t = 0.5  # Discrete time intervals of 1 second

	# Linear motion model
	A = np.array([[1, t], 
				  [0, 1]])

	# Control transformation matrix: acceleration 
	G = np.array([[.5*t**2],  # acc = Force/mass
				  [t]])

	# Error covariance matrices
	Q = np.array([[1, 0], 
				  [0, 1]])  # Process noise cov matrix
	R = np.array([[1]])  # Measurement noise cov matrix
	H = np.array([[1, 0]])  # State -> Measurement transformation matrix, here measurement = state

	# Initialize State vector, no information on velocity
	X = np.array([[x_obs[0]], [0]])

	# Initialize Error Covariance matrix
	# P = init_P(true, 5, 5)
	P = init_P()  # eye(2)

	position, pred_position = [x_obs[0]], [x_obs[0]]
	velocity, pred_velocity = [0], [0]
	P_pred_list, P_list = [P], [P]

	for obs in x_obs[1:]:
		# Predict state
		x_pred = predict_state(X, A, G, a, w)
		P_pred = predict_cov(P, A, Q)
		# print('Init: ', P_pred)

		# Store predictions for visualization
		pred_position.append(x_pred[0])
		pred_velocity.append(x_pred[1])
		P_pred_list.append(P_pred)

		# Update state with feedback measured state 
		# Overwrite the existing state and error covariance
		new_meas = np.array([obs])  # only observing position data, velocity learned over time
		X, P = update(new_meas, x_pred, P_pred, H, R)

		# Store final values for visualization
		position.append(X[0])
		velocity.append(X[1])
		P_list.append(P)


	plt.subplot(2, 1, 1)
	plt.plot(time_axis, position, label='Estimated', color='g')
	plt.plot(time_axis, x_obs, label='Measured', color='r')
	plt.ylabel('Position (mm)')
	plt.title('Measured(RED) and Estimated(GREEN) Position')
	

	plt.subplot(2, 1, 2)
	plt.plot(time_axis, velocity, label='Estimated', color='g')
	plt.plot(time_axis, v_obs, label='Measured', color='r')
	plt.xlabel('time (s)')
	plt.ylabel('Velocity (mm/s)')
	plt.title('Measured(RED) and Estimated(GREEN) Velocity')

	plt.show()

	return P_list


x = 5
v = 1
a = 2
time_steps = 50
time_axis = [i for i in range(time_steps)]
position_noise, velocity_noise = 50, 5
x_actual, v_actual, x_obs, v_obs = generate_noisy_data(x, v, a, time_steps, position_noise, velocity_noise)

plot_graphs(time_axis, x_actual, v_actual, x_obs, v_obs)

main(x_obs, v_obs, a)


