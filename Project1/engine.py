"""
Given a map with known start and end.
Plan global path of distinct waypoints.
Find AprilTag in frame to locate robot in world frame.
Repeat:
    Reference global path to get path to next waypoint.
    Select AprilTag to look at for this next path.
    Interpolate path to next waypoint.
    Use velocity controller to reach next waypoint.
    Use heading controller to keep AprilTag in view.
Until robot reaches goal.
"""
import matplotlib.pyplot as plt
import numpy as np
from astar import astar
import utils

if __name__ == '__main__':
    map_path = input('Enter path to map CSV: ')
    map_ = utils.load_map(map_path)
    graph, start, end = utils.construct_graph(map_)
    pr = astar(graph, start, end)
    path = utils.pr_to_path(pr, start, end)
    start = np.where(map_ == 2)
    end = np.where(map_ == 3)
    walls = np.where(map_ == 1)

    while True:
        curr_pos = None
        next_waypoint = None
        interp_path = None
        velocity = None

    plt.scatter(*start, c='green', marker='x')
    plt.scatter(*end, c='red', marker='x')
    plt.scatter(*walls, c='black', marker='x')
    plt.scatter(path_rows, path_cols, s=8, c='purple', marker='x')
    plt.show()
