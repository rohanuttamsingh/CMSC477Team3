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

def subtract_start_position(start_position, coords):
    return (coords[0] - start_position[0], coords[1] - start_position[1])

def graph_to_real_coords(coords):
    real_y, real_x = coords
    conversion = 0.1524 # 1/2 ft in m
    real_x *= conversion
    real_y *= conversion
    return (real_x, real_y)

def process_path(path, start_position_graph):
    path = [subtract_start_position(start_position_graph, graph_coords) for graph_coords in path]
    path = [graph_to_real_coords(graph_coords) for graph_coords in path]
    return path

def real_to_graph_coords(coords):
    graph_y, graph_x = coords
    conversion = 1 / 0.1524
    graph_x *= conversion
    graph_y *= conversion
    return (round(graph_x), round(graph_y))