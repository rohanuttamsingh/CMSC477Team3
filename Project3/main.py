import numpy as np
from robomaster import robot, camera

import sns

x_old = np.zeros((3,))
x_new = np.zeros((3,))
def sub_position_handler(p, x_new):
    x_new[0], x_new[1], x_new[2] = p[0], p[1], p[2]

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='sta', sn=sns.ROBOT6_SN)
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_chassis = ep_robot.chassis
    ep_chassis.sub_position(freq=50, callback=lambda p: sub_position_handler(p, x_new))
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper

    while True:
        x_old = np.copy(x_new)
