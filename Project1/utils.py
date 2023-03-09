import os
import imageio
import numpy as np
import matplotlib.pyplot as plt
from astar import astar
from tagmap import unit

def load_map(path):
    return np.loadtxt(path, delimiter=',', dtype=int)

def construct_graph(map):
    graph = {}
    for row in range(map.shape[0]):
        for col in range(map.shape[1]):
            if map[row, col] != 1:
                neighbors = []
                dirs = [(1, 0), (0, 1), (-1, 0), (0, -1)]
                for row_diff, col_diff in dirs:
                    new_row, new_col = row + row_diff, col + col_diff
                    if new_row >= 0 and new_row < map.shape[0] and new_col >= 0 and new_col < map.shape[1]:
                        if map[new_row, new_col] != 1:
                            neighbors.append((new_row, new_col))
                graph[(row, col)] = neighbors
            if map[row, col] == 2:
                start = (row, col)
            elif map[row, col] == 3:
                end = (row, col)
    return graph, start, end

def pr_to_path(pr, start, end):
    path = []
    curr = end
    while True:
        path.append(curr)
        if curr == start:
            break
        curr = pr[curr]
    return path[::-1]

def change_origin(path, origin):
    origin_row = origin[0]
    origin_col = origin[1]
    return [(row - origin_row, col - origin_col) for row, col in path]

def swap_xy(path):
    return [(col, -row) for row, col in path]

def change_unit(path):
    return [(unit / 2 * x, unit / 2 * y) for x, y in path]

def get_path(map_path):
    map_ = load_map(map_path)
    graph, start, end = construct_graph(map_)
    pr = astar(graph, start, end)
    path = pr_to_path(pr, start, end)
    origin = np.where(map_ == 3)
    origin = (origin[0][0], origin[1][0])
    path = change_origin(path, origin)
    path = swap_xy(path)
    path = change_unit(path)
    return path
