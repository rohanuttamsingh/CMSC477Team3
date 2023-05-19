import cv2
import numpy as np
import time
from robomaster import robot

import sns
import path_planning

pos = np.zeros((3,))
def sub_position_handler(p):
    pos[0], pos[1], pos[2] = p[1], -p[0], p[2]

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

def controller(next_position):
    K = 1
    diff = np.array(next_position) - pos[:2]
    return K * diff

def distance(position):
    return np.linalg.norm(np.array(position) - pos[:2])

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='sta', sn=sns.ROBOT6_SN)

    ep_chassis = ep_robot.chassis
    ep_chassis.sub_position(cs=0, freq=50, callback=sub_position_handler)

    trajectory_plot = np.zeros((480, 640, 3), dtype=np.uint8)
    plot_off_x = int(trajectory_plot.shape[1]/2)
    plot_off_y = int(trajectory_plot.shape[0]/2)
    # Each pixel in the plot is 1 ft / plot_scale
    plot_scale = 100

    map_ = path_planning.load_map('map_left.csv')
    graph, _ = path_planning.create_graph(map_)
    # Graph coords
    start_position_graph = (1, 1) # Left corner
    goal_position_graph = (5, 5) # River
    pr = path_planning.bfs_reverse(graph, goal_position_graph)
    path = path_planning.pr_to_path(start_position_graph, pr)
    path = process_path(path, start_position_graph)
    print(path)
    # time.sleep(10)

    i = 0
    idx = 0
    threshold = 0.1 # 10cm
    # while idx < len(path):
    #     velocities = controller(path[idx])
    #     ep_chassis.drive_speed(x=velocities[0], y=velocities[1], z=0, timeout=0.1)
    #     if i == 0:
    #         print(f'position: {pos}')
    #         print(f'next point: {path[idx]}')
    #         print(f'velocity: {velocities}')
    #     i = (i + 1) % 30
    #     if distance(path[idx]) < threshold:
    #         print(f'')
    #         idx += 1
    #     cv2.circle(trajectory_plot,
    #                 (int(plot_scale * pos[0] + plot_off_x),
    #                 int(plot_scale * pos[1] + plot_off_y)),
    #                 1, (0,0,255), 1)
    #     cv2.imshow('Trajectory plot', trajectory_plot)
    #     cv2.waitKey(1)

    while True:
        ep_chassis.drive_speed(x=-0.1, y=0.0, z=0, timeout=0.1)
        if i == 0:
            print(f'position: {pos}')
        i = (i + 1) % 30
        cv2.circle(trajectory_plot,
                    (int(plot_scale * pos[0] + plot_off_x),
                    int(plot_scale * pos[1] + plot_off_y)),
                    1, (0,0,255), 1)
        cv2.imshow('Trajectory plot', trajectory_plot)
        cv2.waitKey(1)
