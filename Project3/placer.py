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
import threading

"""

- Start near dropoff point                      handle in main
- Path planning takes robot to near river       DONE
- Wait for "about to drop off LEGO" signal      DONE
- Uses NN to detect LEGO                        DONE
- Picks up LEGO                                 DONE
- Path planning takes robot to dropoff
- Orient robot to directly face dropoff
- Extend arm
- Release LEGO
- Wait for next signal and loop                 DONE


NOTE: keep running counter of how many signals received = how many LEGOs dropped off by picker
=> also track how many LEGOs placed in dropzone => # LEGOs waiting for pickup (assuming no stealing)
= (# picker dropoffs) - (# in dropzone)

NOTE 2: in map_right.csv -- 5 = dropoff zone location, 3 = robot starting location, 2 = river waypoint

NOTE 3: cannot use april tags to check robot orientation - must keep track of it using odometry

"""

goal_x = utils.image_width // 2
goal_y = 285

legos_waiting = 0    # RUNNING COUNTER OF HOW MANY LEGOS ARE WAITING FOR PICKUP


def grab_lego():
    i = 0
    found_lego = False
    centered_with_lego = False
    in_front_of_lego = False
    gripping_lego = False

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
                
                # Grabbed lego => return to main loop
                else:
                    return
            i = (i + 1) % utils.detect_every_n_frames
        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)


# Listens for signals from the picker that a LEGO has been dropped off, and
# increments the value of legos_waiting correspondingly
def signalListener():
    while 1: 
        (data, addr) = UDPSock.recvfrom(buf) 
        print ("Received message: " + data.decode() )
        if data.decode() == "lego_dropped": 
            legos_waiting = legos_waiting + 1


# TODO
def obstacleDetection():
    pass


# Main control loop
def mainLoop():
    while 1:
        # Path planning to go near river

        # Check if any LEGOs waiting for pickup
        while legos_waiting == 0:
            # none dropped off atm - idle while waiting for signal
            pass
        # NN picks up LEGO
        grab_lego()
        # Path planning to dropoff point

        # Orient to face dropzone and move forward if necessary

        # Extend arm and release
        ep_arm.moveto(x=91, y=-32).wait_for_completed() # move arm to transit position
        ep_gripper.open(power=50)
        time.sleep(3)
        ep_gripper.pause()
        legos_waiting = legos_waiting - 1
        # Retract arm, return to starting position/orientation and loop
        ep_arm.moveto(x=91, y=-32).wait_for_completed() # move arm to transit position
        # Loop!


if __name__ == "__main__":
    # --- PROGRAM STARTUP ---
    # initialization stuff goes here
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
    
    host = ''
    port = 13000 
    buf = 1024 
    addr = (host, port) 
    UDPSock = socket(AF_INET, SOCK_DGRAM) 
    UDPSock.bind(addr) 
                    
    tMain = threading.Thread(target=mainLoop)
    tListener = threading.Thread(target=signalListener)
    tObstacles = threading.Thread(target=obstacleDetection)
    # add third thread for obstacle detector
    tMain.start()
    tListener.start()
    tObstacles.start()