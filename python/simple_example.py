#!/usr/bin/python3

import matplotlib.pyplot as plt
import numpy as np
from numpy.random import normal as N
plt.style.use('ggplot')

def state_propagation(x, phi):
    return phi @ x

def covariance_propagation(P, phi, G, Q):
    return phi @ P @ phi.T + G @ Q @ G.T

def predict(x, phi, P, G, Q):
    new_x = state_propagation(x, phi)
    new_p = covariance_propagation(P, phi, G, Q)
    return new_x, new_p

def update(x, P, dy, R, H):
    K = P @ H.T @ np.linalg.inv(R + H @ P @ H.T)
    new_x = x + K @ dy
    I = np.eye(3) 
    J = (I - K @ H)
    new_p = J @ P @ J.T + K @ R @ K.T
    return new_x, new_p

def plot(t_, bias, sx, sv, sb):
    plt.plot(t_, bias, label='${\hat b}_k$')
    plt.plot(t_, sv, c='k', label='$\sigma_{\dot x_k}$')
    plt.plot(t_, sx, c='g', label='$\sigma_{x_k}$')
    plt.plot(t_, sb, c='b', label='$\sigma_{b_k}$')
    plt.legend()
    plt.xlabel('t')
    plt.show()


def test():
    dt = 1.0
    dt2 = dt*dt
    phi = np.array([[1., dt, -dt2/2],
                    [0., 1., -dt],
                    [0., 0.,  1.]])
    P = np.diag([1., 1., 1.])
    G = np.array([[0., 0.],
                  [1., 0.],
                  [0., 1.]])
    Q = np.array([[0.01 * dt2, 0.],
                  [0.        , 0.01 * dt]])
    H = np.array([[1., 0., 0.]])
    R = np.array([[0.09]])

    # initial variables
    x = np.array([[0.0, 0.0, 0.0]]).T
    t = 0.0

    # acceleration, bias
    a = 2.0
    b = 1.0 # this is estimated as part of state. initialized to zero

    # simulated position and velocity [only used for generating observations]
    x_k = 0.0
    v_k = 0.0

    # tracked position and velocity (actual nominal state of the system
    e_x_k = 0.0
    e_v_k = 0.0
    e_b_k = 0.0

    # for logging
    bias = [0.]
    t_ = [0.]
    sx, sv, sb = [1.], [1.], [1.]

    imax = 30
    for i in range(1, imax):
        t = t + dt

        # simulate
        x_k = x_k + v_k * dt + 0.5 * (a + b) * dt2
        v_k = v_k + (a + b) * dt

        # nominal state prediction
        e_x_k = e_x_k + e_v_k * dt + 0.5 * (a - e_b_k) * dt2
        e_v_k = e_v_k + (a - e_b_k) * dt
        e_b_k = e_b_k

        # error state prediction predict
        x, P = predict(x, phi, P, G, Q)

        # update every 3s
        if i % 3 == 0:
            # 1. error in observation 
            e = np.array([[x_k - e_x_k]])

            # 2. update the system 
            x, P = update(x, P, e, R, H)

            # 3. close the loop
            e_x_k = e_x_k + x[0, 0]
            e_v_k = e_v_k + x[1, 0]
            e_b_k = e_b_k + x[2, 0]

            # 4. reset the error state to zero
            x = np.zeros_like(x)

        bias.append(e_b_k)
        sx.append(np.sqrt(P[0, 0]))
        sv.append(np.sqrt(P[1, 1]))
        sb.append(np.sqrt(P[2, 2]))
        t_.append(t)

        print('Step = {:2d}/{:2d}:\t bias = '.format(i, imax-1), e_b_k)
        print('-'*50)
    plot(t_, bias, sx, sv, sb)

if __name__ == '__main__':
    test()

