import time
import cv2
import numpy as np
from robomaster import robot
from robomaster import camera
from roboflow import Roboflow
import sns
import utils
from river import angle_to_river
from cone_detection import detect_dropoff

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
    # found_robot = False
    # centered_with_robot = False
    # in_front_of_river = False
    # gripping_lego = False
    found_robot = True
    centered_with_robot = True
    in_front_of_river = True
    gripping_lego = True
    found_dropoff = False
    at_dropoff = False

    # time.sleep(30)
    while True:
        try:
            image = ep_camera.read_cv2_image(strategy='newest', timeout=0.5)
            if i == 0:
                predictions = utils.detect(model, image)

                # Spin to find lego
                if not found_robot:
                    found_robot = utils.can_see_robot(predictions)
                    ep_chassis.drive_speed(x=0, y=0, z=-20, timeout=0.5)

                # Proportional controller to align robot parallel to river and in line with other robot
                elif not centered_with_robot:
                    retval = angle_to_river(image)
                    y_speed = 0
                    z_speed = 0
                    if retval is not None:
                        riverline = retval['riverline']
                        if len(riverline) >= 100:
                            ang_disp = retval["ang_disp"]
                            if ang_disp > 2:
                                print(f"ang_disp = {ang_disp} ==> turn to left")
                                z_speed = -20
                            elif ang_disp < -2:
                                print(f"ang_disp = {ang_disp} ==> turn to right")
                                z_speed = 20
                            coords = utils.get_robot_coords(predictions)
                            if coords is not None:
                                robot_x, _ = coords
                                centered_with_robot = (goal_x - utils.threshold <= robot_x <= goal_x + utils.threshold)
                                y_speed = (robot_x - goal_x) / 300
                    ep_chassis.drive_speed(x=0, y=y_speed, z=z_speed, timeout=0.5)

                # Move forward to lego
                # elif not in_front_of_robot:
                #     coords = utils.get_robot_coords(predictions)
                #     if coords is not None:
                #         _, robot_y = coords
                #         print(robot_y)
                #         in_front_of_robot = goal_y - utils.threshold <= robot_y <= goal_y + utils.threshold
                #         x_speed = (goal_y - robot_y) / 200
                #         ep_chassis.drive_speed(x=x_speed, y=0, z=0, timeout=0.1)
                elif not in_front_of_river:
                    retval = angle_to_river(image)
                    river_y_prop = np.median(retval["riverline"]) / retval["ylim"]
                    print(f"river y = {river_y_prop}")
                    z_speed = 0
                    lego_coords = utils.get_lego_coords(predictions)
                    if lego_coords is not None:
                        lego_x, _ = utils.get_lego_coords(predictions)
                        z_speed = (lego_x - goal_x) / 10
                    if river_y_prop < 0.835:  # pretty arbitrary cutoff for now - adjust as needed
                        ep_chassis.drive_speed(x=0.2, y=0, z=z_speed, timeout=0.1)
                    else:
                        in_front_of_river = True

                # Squeeze the gripper
                elif not gripping_lego:
                    ep_gripper.close(power=50)
                    time.sleep(2.5)
                    ep_gripper.pause()
                    gripping_lego = True
                    time.sleep(30)
                
                elif not found_dropoff:
                    ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
                    found_dropoff = True

                elif not at_dropoff:
                    dropoff_coords = detect_dropoff(image)
                    if dropoff_coords is not None:
                        dropoff_x, dropoff_y = dropoff_coords
                        print(dropoff_y)
                        at_dropoff = dropoff_y - utils.threshold <= goal_y <= dropoff_y + utils.threshold
                        x_speed = (goal_y - dropoff_y) / 300
                        z_speed = (dropoff_x - goal_x) / 10
                        ep_chassis.drive_speed(x=x_speed, y=0, z=z_speed, timeout=0.1)
                    else:
                        ep_chassis.drive_speed(x=0, y=0, z=-20, timeout=0.5)

                else:
                    ep_gripper.open(power=50)
                    time.sleep(2.5)
                    ep_gripper.pause()


                print(f'found_robot: {found_robot}')
                print(f'centered_with_robot: {centered_with_robot}')
                print(f'in_front_of_river: {in_front_of_river}')
                print(f'gripping_lego: {gripping_lego}')
                print(f'found_dropoff: {found_dropoff}')
                print(f'at_dropoff: {at_dropoff}')
            i = (i + 1) % utils.detect_every_n_frames

            cv2.imshow('image', image)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)

