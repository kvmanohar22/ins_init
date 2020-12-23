#! /usr/bin/python3

from matplotlib import rc
import matplotlib.pyplot as plt
import numpy as np
plt.style.use('ggplot')

rc('font', **{'family':'serif', 'serif':['Cardo']})
rc('text', usetex=True)

def save_plot(xlabel, ylabel):
    plt.legend(loc=1)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.show()

def plot_covariance(t, data):
    plt.plot(t, data[:, 1], label='$\sigma_{\psi_{N}}$')
    plt.plot(t, data[:, 2], label='$\sigma_{\psi_{E}}$')
    plt.plot(t, data[:, 3], label='$\sigma_{\psi_{D}}$')

def plot_attitude(t, data):
    plt.plot(t, data[:, 4], label='roll')
    plt.plot(t, data[:, 5], label='pitch')

def plot_yaw(t, data):
    plt.plot(t, data[:, 6], label='yaw')

def plot(path):
    data = np.loadtxt(open(path), delimiter=",", skiprows=1)
    t = data[:, 0]

    # covariance plot
    fig = plt.figure()
    plot_covariance(t, data)
    save_plot('time (sec)', 'degrees')

    # attitude plot
    fig = plt.figure()
    plot_attitude(t, data)
    save_plot('time (sec)', 'degrees')

    # yaw plot
    fig = plt.figure()
    plot_yaw(t, data)
    save_plot('time (sec)', 'degrees')

if __name__ == '__main__':
    plot("../assets/imu_test_00.csv")

