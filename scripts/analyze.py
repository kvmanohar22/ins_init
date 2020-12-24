#! /usr/bin/python3

from matplotlib import rc
import matplotlib.pyplot as plt
import numpy as np

rc('font', **{'family':'serif', 'serif':['Cardo']})
rc('text', usetex=True)

def save_plot(xlabel, ylabel, path):
    plt.legend(loc=1)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.savefig(path)

def plot_covariance(t, data):
    plt.plot(t, data[:, 1], label='$\sigma_{\psi_{N}}$')
    plt.plot(t, data[:, 2], label='$\sigma_{\psi_{E}}$')
    plt.plot(t, data[:, 3], label='$\sigma_{\psi_{D}}$')

def plot_roll(t, data):
    plt.plot(t, data[:, 4], label='roll')

def plot_pitch(t, data):
    plt.plot(t, data[:, 5], label='pitch')

def plot_yaw(t, data):
    plt.plot(t, data[:, 6], label='yaw')

def plot(path):
    data = np.loadtxt(open(path), delimiter=",", skiprows=1)
    t = data[:, 0]

    # covariance plot
    plt.figure(figsize=(15, 4))
    plot_covariance(t, data)
    save_plot('time (sec)', 'degrees', '../assets/imu_test_00_covariance2.png')

    # attitude plot
    plt.figure(figsize=(15, 4))
    plot_roll(t, data)
    save_plot('time (sec)', 'degrees', '../assets/imu_test_00_roll.png')
    plt.figure(figsize=(15, 4))
    plot_pitch(t, data)
    save_plot('time (sec)', 'degrees', '../assets/imu_test_00_pitch.png')

    # yaw plot
    # plt.figure(figsize=(15, 4))
    # plot_yaw(t, data)
    # save_plot('time (sec)', 'degrees')

if __name__ == '__main__':
    plot("../assets/imu_test_00.csv")

