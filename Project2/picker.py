import time
import cv2
import numpy as np
from socket import *
from robomaster import robot
from robomaster import camera
from roboflow import Roboflow
import sns
import utils
from river import angle_to_river

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
    angled = True
    at_river = False
    sent_at_river = False
    placer_gripping_lego = False

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
                    lego_x, lego_y = utils.get_lego_coords(predictions)
                    in_front_of_lego = goal_y - utils.threshold <= lego_y <= goal_y + utils.threshold
                    x_speed = (goal_y - lego_y) / 200
                    z_speed = (lego_x - goal_x) / 10
                    ep_chassis.drive_speed(x=x_speed, y=0, z=z_speed, timeout=0.1)

                # Squeeze the gripper
                elif not gripping_lego:
                    ep_gripper.close(power=50)
                    time.sleep(3)
                    ep_gripper.pause()
                    ep_arm.move(x=0, y=60).wait_for_completed()
                    gripping_lego = True
                
                elif angled:
                    retval = angle_to_river(image)
                    if retval is not None:
                        riverline = retval['riverline']
                        if len(riverline) >= 100:
                            ang_disp = retval["ang_disp"]
                            if ang_disp > 2:
                                print(f"ang_disp = {ang_disp} ==> turn to left")
                                ep_chassis.drive_speed(x=0, y=0, z=-20, timeout=0.1)
                            elif ang_disp < -2:
                                print(f"ang_disp = {ang_disp} ==> turn to right")
                                ep_chassis.drive_speed(x=0, y=0, z=20, timeout=0.1)
                            else:
                                river_y_avg = np.average(riverline)
                                river_y_med = np.median(riverline)
                                print(f"ang_disp = {ang_disp} ==> do not turn; riverline at {river_y_avg} (avg) or {river_y_med} (median) out of {retval['ylim']}")
                                angled = False
                        else:
                            print("ang_disp = None ==> no river detected!")
                            ep_chassis.drive_speed(x=0, y=0, z=-20, timeout=0.5)
                    else:
                        print("ang_disp = None ==> no river detected!")
                        ep_chassis.drive_speed(x=0, y=0, z=-20, timeout=0.5)

                elif not at_river:
                    retval = angle_to_river(image)
                    river_y_prop = np.median(retval["riverline"]) / retval["ylim"]
                    print(f"river y = {river_y_prop}")
                    if river_y_prop < 0.935:  # pretty arbitrary cutoff for now - adjust as needed
                        ep_chassis.drive_speed(x=0.2, y=0, z=0, timeout=0.1)
                    # elif river_y_prop > 0.94:
                    #     ep_chassis.drive_speed(x=-0.05, y=0, z=0, timeout=0.1)
                    else:
                        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
                        at_river = True

                elif not sent_at_river:
                    host = "192.168.50.4" # set to IP address of target computer 
                    port = 13000 
                    addr = (host, port) 
                    UDPSock = socket(AF_INET, SOCK_DGRAM) 
                    data = 'atriver'
                    UDPSock.sendto(data.encode(), addr) 
                    UDPSock.close() 
                    sent_at_river = True
                
                elif not placer_gripping_lego:
                    host = ''
                    port = 13000 
                    buf = 1024 
                    addr = (host, port) 
                    UDPSock = socket(AF_INET, SOCK_DGRAM) 
                    UDPSock.bind(addr) 
                    print ("Waiting to receive messages...")
                    while not placer_gripping_lego: 
                        (data, addr) = UDPSock.recvfrom(buf) 
                        print ("Received message: " + data.decode() )
                        if data.decode() == "gripping_lego": 
                            placer_gripping_lego = True
                            UDPSock.close() 

                else:
                    ep_gripper.open(power=50)
                    time.sleep(2.5)
                    ep_gripper.pause()
                    # ep_arm.move(x=0, y=-60).wait_for_completed()
                    ep_chassis.move(x=-0.1, y=0, z=0).wait_for_completed()
                    break

                print(f'found_lego: {found_lego}')
                print(f'centered_with_lego: {centered_with_lego}')
                print(f'in_front_of_lego: {in_front_of_lego}')
                print(f'gripping_lego: {gripping_lego}')
                print(f'angled: {angled}')
                print(f'at_river: {at_river}')
            i = (i + 1) % utils.detect_every_n_frames

            cv2.imshow('image', image)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)

