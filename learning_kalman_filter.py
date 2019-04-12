import numpy as np
from numpy.linalg import inv

x_obs = np.array([4000, 4260, 4550, 4860, 5110])
v_obs = np.array([280, 282, 285, 286, 290])
z = np.c_[x_obs, v_obs]

# static conditions
acc = 2  # acceleration
t = 1  # discrete difference in time

# process / estimate errors
error_est_x = 20
error_est_v = 5

# observation errors
error_obs_x = 25
error_obs_v = 6

# Predicted model for motion
A = np.array([[1, t],
              [0, 1]])
B = np.array([[0.5 * t**2],
              [t]])



def update_state(x, v, t, acc):
    X = np.array([[x],
                  [v]])
    new_state = A.dot(X) + B.dot(acc)
    return new_state

def covariance(sigma1, sigma2):
    cov = sigma1 * sigma2

    P = np.array([[sigma1**2, cov],
                  [cov, sigma2**2]])
    return np.diag(np.diag(P))  # set off-diagonals to 0
    # return P

# Initial Estimation Covariance Matrix
P = covariance(error_est_x, error_est_v)
X = np.array([[z[0][0]],
              [z[0][1]]])
N = len(z[0])

for data in z[1:]:  # already have initial state
    # always use initial measurement as
    X = update_state(X[0][0], X[1][0], t, acc)
    print(X, X[0], X[1])
    # P = A.dot(P).dot(A.T)
    P = np.diag(np.diag(A.dot(P).dot(A.T)))

    # Calculating Kalman Gain
    H = np.identity(N)
    R = covariance(error_obs_x, error_obs_v)
    S = H.dot(P).dot(H.T) + R
    K = P.dot(H).dot(inv(S))

    # Reshape new data into measurement space
    Y = H.dot(data).reshape(N, -1)

    # Update state matrix
    X = X + K.dot(Y - H.dot(X))

    # Update Process Covariance Matrix
    P = (np.identity(len(K)) - K.dot(H)).dot(P)

    print("State: \n", X)

