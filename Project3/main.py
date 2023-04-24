import numpy as np
from robomaster import robot, camera

import sns
import path_planning

K = 1.5
threshold = 0.2 # feet

x_old = np.zeros((3,))
x_new = np.zeros((3,))
def sub_position_handler(p, x_new):
    x_new[0], x_new[1], x_new[2] = p[0], p[1], p[2]

def m_to_scale():
    """Converts x_new to our map scale (0.5ft/unit)"""
    return x_new[:2] / (3.28084 / 2)

def distance(idx):
    """Distance to point at index idx in path."""
    diff = m_to_scale() - np.array(path[idx])
    return np.linalg.norm(diff)

def get_xy_velocity(idx):
    """Velocity to get to point at index idx in path."""
    return (K * (path[idx][0] - x_new[0]), K * (path[idx][1] - x_new[1]))

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='sta', sn=sns.ROBOT6_SN)
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_chassis = ep_robot.chassis
    ep_chassis.sub_position(freq=50, callback=lambda p: sub_position_handler(p, x_new))
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper

    graph, goal_position = path_planning.create_graph('map_left.csv')
    pr = path_planning.bfs_reverse(graph, goal_position)
    path = path_planning.scale_path(path_planning.pr_to_path((0, 0), pr))
    idx = 1
    
    while True:
        x_old = np.copy(x_new)
        if idx == len(path) - 1:
            print('Made it')
            break
        if distance(idx) <= threshold:
            idx += 1
        xy_velocity = get_xy_velocity(idx)
        ep_chassis.drive_speed(x=xy_velocity[0], y=xy_velocity[1], z=0, timeout=0.5)
