import numpy as np
import utils
from astar import astar
from interpolate import interpolate

def path_to_next_waypoint(path, robot_world_pos):
    min_dist = float('inf')
    closest_idx = None
    robot_world_pos = np.array(robot_world_pos)
    for i, point in enumerate(path):
        dist = np.linalg.norm(np.array(point) - robot_world_pos)
        if dist < min_dist:
            min_dist = dist
            closest_idx = i
    if closest_idx == len(path) - 1: # End of path
        res = interpolate([robot_world_pos, path[-1]])
    else:
        res = interpolate([robot_world_pos, path[closest_idx + 1]])
    return res

if __name__ == '__main__':
    path = utils.get_path('Map.csv')
    print(path)
    robot_world_pos = (-9.2, 1.9)
    print(path_to_next_waypoint(path, robot_world_pos))
