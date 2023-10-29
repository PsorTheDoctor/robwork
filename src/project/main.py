import pandas as pd
import matplotlib.pyplot as plt


def main(path):
    data = pd.read_csv(path).to_numpy()

    t = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    z = data[:, 3]

    fig, (ax1, ax2, ax3) = plt.subplots(3)
    ax1.plot(x)
    ax2.plot(y)
    ax3.plot(z)
    plt.show()


main('./data/linear.csv')
main('./data/parabolic.csv')
