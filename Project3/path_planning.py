import numpy as np

def load_map(path):
    return np.loadtxt(path, delimiter=',')

def create_graph(map_):
    wall = 1
    graph = {}
    goal = 2
    for row in range(map_.shape[0]):
        for col in range(map_.shape[1]):
            if map_[row, col] != wall:
                if map_[row, col] == goal:
                    goal_position = (row, col)
                neighbors = []
                dirs = [(1, 0), (0, 1), (-1, 0), (0, -1)]
                for row_diff, col_diff in dirs:
                    new_row, new_col = row + row_diff, col + col_diff
                    if 0 <= new_row < map_.shape[0] and 0 <= new_col < map_.shape[1]:
                        if map_[new_row, new_col] != wall:
                            neighbors.append(((new_row, new_col), 1))
                dirs = [(1, 1), (-1, 1), (-1, -1), (1, -1)]
                for row_diff, col_diff in dirs:
                    new_row, new_col = row + row_diff, col + col_diff
                    if 0 <= new_row < map_.shape[0] and 0 <= new_col < map_.shape[1]:
                        if map_[new_row, new_col] != wall:
                            neighbors.append(((new_row, new_col), np.sqrt(2)))
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
            for neighbor, next_d in graph[curr]:
                new_d = d + next_d
                if neighbor not in visited and (neighbor not in ds or new_d < ds[neighbor]):
                    pr[neighbor] = curr
                    ds[neighbor] = new_d
                    q.append((neighbor, new_d))
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
    """Given a path, scales the points to feet scale and changes the starting
    point to (0, 0)."""
    unit = 0.5 # 0.5 feet
    # swapped_path = [(y, x) for x, y in path]
    # start_x, start_y = swapped_path[0]
    start_x, start_y = path[0]
    zero_start_path = [(x - start_x, y - start_y) for x, y in path]
    scaled_path = [(x * unit, y * unit) for x, y in zero_start_path]
    return scaled_path

def get_start_position(map_):
    """Given a numpy array representing a map, gets the starting position of
    the robot (represented by a 3)."""
    raw_start_position = np.where(map_ == 3)
    start_position = (raw_start_position[0][0], raw_start_position[1][0])
    return start_position
