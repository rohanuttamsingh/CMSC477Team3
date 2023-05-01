import numpy as np
from robomaster import robot, camera
from time import sleep

import sns
import path_planning

K = 0.1
threshold = 0.2 # feet

pos = np.zeros((3,))
def sub_position_handler(p):
    # Mapping from robot coordinate system to map coordinate system
    m_to_ft = 3.28084
    # x and y are swapped and x is negative
    # TODO: This isn't always the case, depends on robot startup
    # Test on every boot
    pos[0], pos[1], pos[2] = -p[1] * m_to_ft, p[0] * m_to_ft, p[2]

def distance(idx):
    """Distance to point at index idx in path."""
    diff = np.array(path[idx]) - pos[:2]
    return np.linalg.norm(diff)

def get_xy_velocity(idx):
    """Velocity to get to point at index idx in path."""
    Kx = K
    # Ky = 1.8 * Kx
    Ky = Kx
    # Same velocity on x and y moves y half as fast as x
    velocity = np.array(path[idx]) - pos[:2]
    velocity[0] *= Kx
    velocity[1] *= Ky
    return velocity

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='sta', sn=sns.ROBOT6_SN)
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_chassis = ep_robot.chassis
    ep_chassis.sub_position(cs=0, freq=50, callback=sub_position_handler)
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper

    map_ = path_planning.load_map('map_left.csv')
    graph, goal_position = path_planning.create_graph(map_)
    pr = path_planning.bfs_reverse(graph, goal_position)
    start_position = (0, 0)
    # start_position = path_planning.get_start_position(map_)
    path = path_planning.scale_path(path_planning.pr_to_path(start_position, pr))
    idx = 1

    print(path)
    sleep(3)

    i = 0
    
    # while True:
    #     if idx == len(path) - 1:
    #         print('Made it')
    #         break
    #     if distance(idx) <= threshold:
    #         print(f'Passed point {idx}')
    #         idx += 1
    #     xy_velocity = get_xy_velocity(idx)
    #     if i == 0:
    #         print('position:', pos)
    #         print('next point:', path[idx])
    #         print('velocity:', xy_velocity)
    #     i = (i + 1) % 30
    #     ep_chassis.drive_speed(x=xy_velocity[0], y=xy_velocity[1], z=0, timeout=0.1)

    while True:
        if i == 0:
            print('position:', pos)
        i = (i + 1) % 30
        ep_chassis.drive_speed(x=0.1, y=0.1, z=0, timeout=0.1)
