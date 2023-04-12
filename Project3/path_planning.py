import numpy as np

def load_map(path):
    return np.loadtxt(path, delimiter=',')

def create_graph(path):
    wall = 1
    map_ = load_map(path)
    graph = {}
    for row in range(map_.shape[0]):
        for col in range(map_.shape[1]):
            if map_[row, col] != wall:
                neighbors = []
                dirs = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
                for row_diff, col_diff in dirs:
                    new_row, new_col = row + row_diff, col + col_diff
                    if 0 <= new_row < map_.shape[0] and 0 <= new_col < map_.shape[1]:
                        if map_[new_row, new_col] != wall:
                            neighbors.append((new_row, new_col))
                graph[(row, col)] = neighbors
    return graph

def bfs_reverse(graph):
    """BFS from goal location to every other node."""
    goal = 2
    goal_position = np.where(graph == goal)
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
