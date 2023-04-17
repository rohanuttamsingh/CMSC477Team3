import numpy as np

def load_map(path):
    return np.loadtxt(path, delimiter=',')

def create_graph(path):
    wall = 1
    map_ = load_map(path)
    graph = {}
    goal = 2
    for row in range(map_.shape[0]):
        for col in range(map_.shape[1]):
            if map_[row, col] != wall:
                if map_[row, col] == goal:
                    goal_position = (row, col)
                neighbors = []
                dirs = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
                for row_diff, col_diff in dirs:
                    new_row, new_col = row + row_diff, col + col_diff
                    if 0 <= new_row < map_.shape[0] and 0 <= new_col < map_.shape[1]:
                        if map_[new_row, new_col] != wall:
                            neighbors.append((new_row, new_col))
                graph[(row, col)] = neighbors
    return graph, goal_position

def bfs_reverse(graph, goal_position):
    """BFS from goal location to every other node."""
    goal_position = (goal_position[0], goal_position[1])
    visited = set()
    pr = {}
    q = [(goal_position, 0)]
    ds = {goal_position: 0}
    while len(q) > 0:
        curr, d = q.pop(0)
        if curr not in visited:
            visited.add(curr)
            for neighbor in graph[curr]:
                if neighbor not in visited and (neighbor not in ds or d + 1 < ds[neighbor]):
                    pr[neighbor] = curr
                    ds[neighbor] = d + 1
                    q.append((neighbor, d + 1))
    return pr

def pr_to_path(start, pr):
    """Given start location and pr from goal location to every other node,
    returns path to the goal node."""
    path = []
    curr = pr[start]
    while True:
        path.append(curr)
        curr = pr[curr]
        if curr not in pr:
            break
    path.append(curr)
    return path

def scale_path(path):
    """Given a path, scales the points to feet scale. Assumes that the starting
    point is at (0, 0)."""
    unit = 0.5 # 0.5 feet
    start_x, start_y = path[0]
    zero_start_path = [(x - start_x, y - start_y) for x, y in path]
    scaled_path = [(x * unit, y * unit) for x, y in zero_start_path]
    return scaled_path

def m_to_feet(position):
    """Given an (x, y) tuple in meters, returns it in feet. Useful for
    converting wheel odometry to our map scale."""
    scale = 3.28084
    return (position[0] * scale, position[1] * scale)

def velocity(position, next_point):
    """Given robot current position and next point in path, uses a proportional
    controller to determine the x and y velocity of the robot."""
    K = 1.5
    return (K * (next_point[0] - position[0]), K * (next_point[1] - position[1]))
