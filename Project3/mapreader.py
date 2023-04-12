import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    map = np.loadtxt('map_right.csv', delimiter=',', dtype=int)

    obs_x = []
    obs_y = []

    for r in range(len(map)):
        for c in range(len(map[r])):
            if map[r][c] > 0:
                obs_x.append(c)
                obs_y.append(-r)
            pass

    plt.scatter(obs_x, obs_y)
    plt.xlim(0, 31)
    plt.ylim(-27, 0)
    plt.show()