import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d
from astar import astar
import utils

def interpolate(path):
    interpolation = []
    for i in range(len(path) - 1):
        x_start, y_start = path[i]
        x_end, y_end = path[i + 1]
        x_end += 0.001 # Prevent x_start and x_end from being duplicates
        x_new = np.linspace(x_start, x_end, num=10, endpoint=True)
        y_new = np.linspace(y_start, y_end, num=10, endpoint=True)
        f = interp1d(x_new, y_new, kind='cubic')
        y_interp = list(f(x_new))
        interpolation += [(x, y) for (x, y) in zip(list(x_new), y_interp)]
    return interpolation

if __name__ == '__main__':
    map_path = input('Enter path to map CSV: ')
    map_ = utils.load_map(map_path)
    graph, start, end = utils.construct_graph(map_)
    pr = astar(graph, start, end)
    path = utils.pr_to_path(pr, start, end)
    start = np.where(map_ == 2)
    end = np.where(map_ == 3)
    walls = np.where(map_ == 1)
    visited_points = list(pr.keys())
    visited_rows = [point[0] for point in visited_points]
    visited_cols = [point[1] for point in visited_points]
    path_rows = [point[0] for point in path]
    path_cols = [point[1] for point in path]
    plt.scatter(*start, c='green', marker='x')
    plt.scatter(*end, c='red', marker='x')
    plt.scatter(*walls, c='black', marker='x')
    plt.scatter(path_rows, path_cols, s=8, c='purple', marker='x')
    interpolation = interpolate(path)
    x = [point[0] for point in interpolation]
    y = [point[1] for point in interpolation]
    plt.plot(x, y, '-')
    plt.show()
