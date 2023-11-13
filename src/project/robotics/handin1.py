import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def plot():
    path = 'data/'
    linear_side = pd.read_csv(path + 'linear_from_side2.csv').to_numpy()
    parabolic_side = pd.read_csv(path + 'parabolic_from_side2.csv').to_numpy()
    linear_top = pd.read_csv(path + 'linear_from_above2.csv').to_numpy()
    parabolic_top = pd.read_csv(path + 'parabolic_from_above2.csv').to_numpy()

    figsize = (8, 6)
    blue = '#ff7f0e'
    orange = '#1f77b4'
    joint_names = ['base', 'shoulder', 'elbow', 'wrist 1', 'wrist 2', 'wrist 3']
    tcp_coords = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']

    # Combine the plots of both methods on a single plot,
    # which shows the trajectories by DOF.
    q_lin = linear_side[:, 1:7]
    q_blend = parabolic_side[:, 1:7]
    q_lin_ = linear_top[:, 1:7]
    q_blend_ = parabolic_top[:, 1:7]

    fig, ax = plt.subplots(6, sharex=True, figsize=figsize)
    fig.suptitle('Joint angles')
    fig.subplots_adjust(left=0.1, right=0.9, top=0.83, bottom=0.1)

    for i in range(6):
        ax[i].plot(q_lin[:, i], color=blue, linestyle='dotted', label='linear, side grasp' if i == 0 else None)
        ax[i].plot(q_blend[:, i], color=orange, linestyle='dotted', label='parabolic, side grasp' if i == 0 else None)
        ax[i].plot(q_lin_[:, i], color=blue, label='linear, top grasp' if i == 0 else None)
        ax[i].plot(q_blend_[:, i], color=orange, label='parabolic, top grasp' if i == 0 else None)

        ax[i].set_ylabel(joint_names[i])
    ax[5].set_xlabel('time')
    fig.legend()
    fig.savefig('images/joints.png')

    # Calculate the DK for the two executions
    # and plot them as a separate plot by DOF.
    # Position
    pos_lin = linear_side[:, 7:10]
    pos_blend = parabolic_side[:, 7:10]
    pos_lin_ = linear_top[:, 7:10]
    pos_blend_ = parabolic_top[:, 7:10]
    # Orientation
    orn_lin = linear_side[:, 10:13]
    orn_blend = parabolic_side[:, 10:13]
    orn_lin_ = linear_top[:, 10:13]
    orn_blend_ = parabolic_top[:, 10:13]

    fig, ax = plt.subplots(6, sharex=True, figsize=figsize)
    fig.suptitle('TCP trajectory')
    fig.subplots_adjust(left=0.1, right=0.9, top=0.83, bottom=0.1)

    for i in range(3):
        ax[i].plot(pos_lin[:, i], color=blue, linestyle='dotted', label='linear, side grasp' if i == 0 else None)
        ax[i].plot(pos_blend[:, i], color=orange, linestyle='dotted', label='parabolic, side grasp' if i == 0 else None)
        ax[i].plot(pos_lin_[:, i], color=blue, label='linear, top grasp' if i == 0 else None)
        ax[i].plot(pos_blend_[:, i], color=orange, label='parabolic, top grasp' if i == 0 else None)

        ax[i + 3].plot(orn_lin[:, i], color=blue, linestyle='dotted')
        ax[i + 3].plot(orn_blend[:, i], color=orange, linestyle='dotted')
        ax[i + 3].plot(orn_lin_[:, i], color=blue)
        ax[i + 3].plot(orn_blend_[:, i], color=orange)
        ax[i].set_ylabel(tcp_coords[i])

        ax[i + 3].set_ylabel(tcp_coords[i + 3])
    ax[5].set_xlabel('time')
    fig.legend()
    fig.savefig('images/pos_orn.png')

    # Calculate the velocity and acceleration profiles for both cases
    # and plot them on a single plot by DOF.
    # Velocity
    dq_lin = np.gradient(q_lin, axis=0)
    dq_blend = np.gradient(q_blend, axis=0)
    dq_lin_ = np.gradient(q_lin_, axis=0)
    dq_blend_ = np.gradient(q_blend_, axis=0)
    # Acceleration
    ddq_lin = np.gradient(dq_lin, axis=0)
    ddq_blend = np.gradient(dq_blend, axis=0)
    ddq_lin_ = np.gradient(dq_lin_, axis=0)
    ddq_blend_ = np.gradient(dq_blend_, axis=0)

    fig, ax = plt.subplots(6, sharex=True, figsize=figsize)
    fig.suptitle('Joint velocities')
    fig.subplots_adjust(left=0.1, right=0.9, top=0.83, bottom=0.1)

    for i in range(6):
        ax[i].plot(dq_lin[:, i], color=blue, linestyle='dotted', label='linear, side grasp' if i == 0 else None)
        ax[i].plot(dq_blend[:, i], color=orange, linestyle='dotted', label='parabolic, side grasp' if i == 0 else None)
        ax[i].plot(dq_lin_[:, i], color=blue, label='linear, top grasp' if i == 0 else None)
        ax[i].plot(dq_blend_[:, i], color=orange, label='parabolic, top grasp' if i == 0 else None)

        ax[i].set_ylabel(joint_names[i])
    ax[5].set_xlabel('time')
    fig.legend()
    fig.savefig('images/vel.png')

    fig, ax = plt.subplots(6, sharex=True, figsize=figsize)
    fig.suptitle('Joint accelerations')
    fig.subplots_adjust(left=0.1, right=0.9, top=0.83, bottom=0.1)

    for i in range(6):
        ax[i].plot(ddq_lin[:, i], color=blue, linestyle='dotted', label='linear, side grasp' if i == 0 else None)
        ax[i].plot(ddq_blend[:, i], color=orange, linestyle='dotted', label='parabolic, side grasp' if i == 0 else None)
        ax[i].plot(ddq_lin_[:, i], color=blue, label='linear, top grasp' if i == 0 else None)
        ax[i].plot(ddq_blend_[:, i], color=orange, label='parabolic, top grasp' if i == 0 else None)

        ax[i].set_ylabel(joint_names[i])
    ax[5].set_xlabel('time')
    fig.legend()
    fig.savefig('images/acc.png')

    plt.show()


if __name__ == '__main__':
    plot()
