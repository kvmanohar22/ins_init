#! /usr/bin/python3

from matplotlib import rc
import matplotlib.pyplot as plt
import numpy as np

rc('font', **{'family':'serif', 'serif':['Cardo']})
rc('text', usetex=True)

def save_plot(xlabel, ylabel):
    plt.legend()
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.show()

def plot(path):
    data = np.loadtxt(open(path), delimiter=",", skiprows=1)
    t = data[:, 0]
    plt.plot(t, data[:, 1], label='$\sigma_{\psi_{N}}$')
    plt.plot(t, data[:, 2], label='$\sigma_{\psi_{E}}$')
    plt.plot(t, data[:, 3], label='$\sigma_{\psi_{D}}$')
    save_plot('time (sec)', 'degrees')

if __name__ == '__main__':
    plot("../assets/ins_init.csv")

