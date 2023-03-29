import time
import cv2
from robomaster import robot
from robomaster import camera
from roboflow import Roboflow
import sns
import utils

goal_x = utils.image_width // 2
goal_y = 240

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='sta', sn=sns.ROBOT5_SN)
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper

    rf = Roboflow(api_key='kKusTXhj0ObVGmi9slHp')
    project = rf.workspace().project('project2-l7rdy')
    model = project.version(4).model

    i = 0
    found_robot = False
    in_line = False
    gripping_lego = False
    found_river = False
    while True:
        try:
            image = ep_camera.read_cv2_image(strategy='newest', timeout=0.5)
            if i == 0:
                predictions = utils.detect(model, image)

                # Spin to find robot
                if not found_robot:
                    found_robot = utils.can_see_robot(predictions)
                    ep_chassis.drive_speed(x=0, y=0, z=-20, timeout=0.5)

                # Proportional controller to align robot parallel with river and in line with other robot
                elif not centered_with_robot:
                    river_angle = None # TODO
                    coords = utils.get_robot_coords(predictions)
                    if coords is not None:
                        robot_x, _ = coords
                        centered_with_robot = (goal_x - utils.threshold <= robot_x <= goal_x + utils.threshold)
                        y_speed = (robot_x - goal_x) / 200
                        z_speed = -river_angle * 3
                        ep_chassis.drive_speed(x=0, y=y_speed, z=z_speed, timeout=0.5)

                # Move forward to robot
                elif not in_front_of_robot:
                    coords = utils.get_robot_coords(predictions)
                    if coords is not None:
                        _, robot_y = coords
                        print(robot_y)
                        in_front_of_robot = goal_y - utils.threshold <= robot_y <= goal_y + utils.threshold
                        x_speed = (goal_y - robot_y) / 200
                        ep_chassis.drive_speed(x=x_speed, y=0, z=0, timeout=0.1)

                # Squeeze the gripper
                elif not gripping_lego:
                    ep_gripper.close(power=50)
                    time.sleep(2)
                    ep_gripper.pause()
                    gripping_robot = True


                print(f'found_robot: {found_robot}')
                print(f'centered_with_robot: {centered_with_robot}')
                print(f'in_front_of_robot: {in_front_of_robot}')
                print(f'gripping_lego: {gripping_lego}')
            i = (i + 1) % utils.detect_every_n_frames

            cv2.imshow('image', image)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)

