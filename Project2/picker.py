import time
import cv2
from robomaster import robot
from robomaster import camera
from roboflow import Roboflow
import sns
import utils

goal_x = utils.image_width // 2
goal_y = 285

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='sta', sn=sns.ROBOT6_SN)
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper

    rf = Roboflow(api_key='kKusTXhj0ObVGmi9slHp')
    project = rf.workspace().project('project2-l7rdy')
    model = project.version(4).model

    i = 0
    found_lego = False
    centered_with_lego = False
    in_front_of_lego = False
    gripping_lego = False
    found_river = False
    while True:
        try:
            image = ep_camera.read_cv2_image(strategy='newest', timeout=0.5)
            if i == 0:
                predictions = utils.detect(model, image)

                # Spin to find lego
                if not found_lego:
                    found_lego = utils.can_see_lego(predictions)
                    ep_chassis.drive_speed(x=0, y=0, z=-20, timeout=0.5)

                # Spin to center lego
                elif not centered_with_lego:
                    lego_x, _ = utils.get_lego_coords(predictions)
                    centered_with_lego = goal_x - utils.threshold <= lego_x <= goal_x + utils.threshold
                    z_speed = (lego_x - goal_x) / 10
                    ep_chassis.drive_speed(x=0, y=0, z=z_speed, timeout=0.5)

                # Move forward to lego
                elif not in_front_of_lego:
                    _, lego_y = utils.get_lego_coords(predictions)
                    in_front_of_lego = goal_y - utils.threshold <= lego_y <= goal_y + utils.threshold
                    x_speed = (goal_y - lego_y) / 200
                    ep_chassis.drive_speed(x=x_speed, y=0, z=0, timeout=0.1)

                # Squeeze the gripper
                elif not gripping_lego:
                    ep_gripper.close(power=50)
                    time.sleep(2)
                    ep_gripper.pause()
                    ep_arm.move(x=0, y=60).wait_for_completed()
                    gripping_lego = True


                print(f'found_lego: {found_lego}')
                print(f'centered_with_lego: {centered_with_lego}')
                print(f'in_front_of_lego: {in_front_of_lego}')
                print(f'gripping_lego: {gripping_lego}')
            i = (i + 1) % utils.detect_every_n_frames

            cv2.imshow('image', image)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)

