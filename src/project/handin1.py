import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def plot(path, title):
    data = pd.read_csv(path).to_numpy()

    # Position
    # t = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    z = data[:, 3]

    # Velocity
    dx = np.gradient(x)
    dy = np.gradient(y)
    dz = np.gradient(z)

    # Acceleration
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    ddz = np.gradient(dz)

    fig, (ax1, ax2, ax3) = plt.subplots(3)
    fig.suptitle(title)
    ax1.plot(x)
    ax2.plot(y)
    ax3.plot(z)

    fig, (ax1, ax2, ax3) = plt.subplots(3)
    fig.suptitle(title)
    ax1.plot(dx)
    ax2.plot(dy)
    ax3.plot(dz)

    fig, (ax1, ax2, ax3) = plt.subplots(3)
    fig.suptitle(title)
    ax1.plot(ddx)
    ax2.plot(ddy)
    ax3.plot(ddz)


plot(path='./data/linear.csv', title='Linear interpolation')
plot(path='./data/parabolic.csv', title='Linear interpolation with parabolic blend')
plt.show()
