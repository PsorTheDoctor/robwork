import sdurw as rw
import matplotlib.pyplot as plt
import numpy as np


def cubic_spline(t: float, ts: float, tf: float,
                 Ps: rw.Vector2D, Pf: rw.Vector2D,
                 Vs: rw.Vector2D, Vf: rw.Vector2D) -> rw.Vector2D:

    p1 = (Pf - Ps) * (-2.0) + (Vs + Vf) * (tf - ts)
    p2 = (Pf - Ps) * 3.0 - (Vs * 2.0 + Vf) * (tf - ts)
    p3 = Vs * (t - ts) + Ps

    T = (t - ts) / (tf - ts)
    return p1 * T**3 + p2 * T**2 + p3


if __name__ == "__main__":
    Vs = rw.Vector2D(0, 1)
    Vf = rw.Vector2D(1, 0)
    Ps = rw.Vector2D(0, 0)
    Pf = rw.Vector2D(1, 2)

    x1 = []
    y1 = []
    x2 = []
    y2 = []
    x3 = []
    y3 = []

    for t in np.arange(0, 1, 0.001):
        V = cubic_spline(t, 0.0, 1.0, Ps, Pf, Vs, Vf)
        x1.append(V[0])
        y1.append(V[1])

        V = cubic_spline(t, 0.0, 1.0, Ps, Pf, Vs * 2, Vf * 2)
        x2.append(V[0])
        y2.append(V[1])

        V = cubic_spline(t, 0.0, 1.0, Ps, Pf, Vs * 6, Vf * 6)
        x3.append(V[0])
        y3.append(V[1])

    fig, ax = plt.subplots()
    ax.plot(x1, y1, linewidth=2.0)
    ax.plot(x2, y2, linewidth=2.0)
    ax.plot(x3, y3, linewidth=2.0)
    ax.set(xlim=(-0.5, 1.5), xticks=np.arange(-0.5, 1.5, 0.25),
           ylim=(-0.5, 2.5), yticks=np.arange(-0.5, 2.5, 0.25))
    ax.grid()
    plt.show()
