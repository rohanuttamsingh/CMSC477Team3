import argparse
import time
import numpy as np
import cv2
from socket import *
from robomaster import robot
from robomaster import camera
import sns
import threading
import detector
import path_planning

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

goal_x = 1280 // 2
goal_y = 350
threshold = 5

legos_waiting = 0    # RUNNING COUNTER OF HOW MANY LEGOS ARE WAITING FOR PICKUP

map = np.loadtxt('map_right.csv', delimiter=',', dtype=int)
obstacleList = []  # array of tuples denoting the center of all obstacles found

pos = np.zeros((3,))
def sub_position_handler(p):
    pos[0], pos[1], pos[2] = p[0], p[1], p[2]

att = np.zeros((3,))
start_att = np.zeros((3,))
def sub_attitude_handler(p):
    att[0], att[1], att[2] = p[0], p[1], p[2]
    att[0] -= start_att[0]
    att[1] -= start_att[1]
    att[2] -= start_att[2]

def controller(next_position):
    K = [1, 1.2]
    diff = np.array(next_position) - pos[:2]
    return K * diff

def rotate_to(yaw_des):
    K = 1.5
    diff = yaw_des - att[0]
    while abs(diff) > 3:
        ep_chassis.drive_speed(x=0, y=0, z=K*diff, timeout=0.1)
        diff = yaw_des - att[0]
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
    print("Done rotating")


def distance(position):
    return np.linalg.norm(np.array(position) - pos[:2])

def velocity_to_avoid_robots(image):
    velocity = np.zeros(2)
    robot_x_centers = detector.get_close_robot_x_centers(image)
    amount = 0.2
    for x in robot_x_centers:
        if x < detector.COLS // 3:
            # Obstacle robot is to the left of this robot
            # Back and to the right velocity
            velocity[0] -= amount
            velocity[1] += amount
        elif x > 2 * detector.COLS // 3:
            # Obstacle robot is to the right of this robot
            # Back and to the left velocity
            velocity[0] -= amount
            velocity[1] -= amount
        else:
            # Obstacle robot is in front of our robot
            # Backwards velocity
            velocity[0] -= amount

def grab_lego():
    i = 0
    found_lego = False
    centered_with_lego = False
    in_front_of_lego = False
    gripping_lego = False
    ep_chassis._action_dispatcher._in_progress = {}
    ep_arm.moveto(x=183, y=-71).wait_for_completed()

    while True:
        try:
            image = ep_camera.read_cv2_image(strategy='newest', timeout=5.0)
            if i == 0:
                # Spin to find lego
                if not found_lego:
                    found_lego = detector.can_see_lego(image)
                    ep_chassis.drive_speed(x=0, y=0, z=-20, timeout=0.1)

                # Spin to center lego
                elif not centered_with_lego:
                    lego_x, _ =  detector.get_closest_lego_coords(image)
                    centered_with_lego = goal_x - threshold <= lego_x <= goal_x + threshold
                    z_speed = (lego_x - goal_x) / 10
                    ep_chassis.drive_speed(x=0, y=0, z=z_speed, timeout=0.1)

                # Move forward to lego
                elif not in_front_of_lego:
                    lego_x, lego_y = detector.get_closest_lego_coords(image)
                    in_front_of_lego = goal_y - threshold <= lego_y <= goal_y + threshold
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
            i = (i + 1) % 5  # only run on every n = 5 frames
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


# Removes an old obstacle entry from the map if it's close enough to new one
# => assuming that means the previous obstacle was somehow moved to this
# current position
def clearObstacle(r, c):
    for (ro, co) in obstacleList:
        if abs(ro-r) <= 2 and abs(co-c) <= 2:
            print(f"Removing {ro, co} from obstacle list")
            obstacleList.remove((ro, co))
            for i in range(-1,2):
                for j in range(-1,2):
                    map[ro+i][co+j] = 0


# TODO
def obstacleDetection():
    while True:
        f = 184.752*1.7                                             # focal length IN PIXELS
        theta = np.radians(61.45)
        image = ep_camera.read_cv2_image(strategy='newest', timeout=5.0)
        obstacles = detector.get_obstacles(image)
        print(f"obstacles length = {len(obstacles)}")
        for obstacle in obstacles:
            # do all of the below for each obstacle
            (x,y) = detector.get_obstacle_offset_from_center(obstacle)
            phi = -np.arctan(y / f) # inverting because y > 0 means edge is BELOW center in img frame
            d = 5 + 32.5 * np.tan(theta + phi)                      # cm
            psi = np.arctan(x / f)
            a = d * np.tan(psi)                                     # cm
            # convert (d,a) from robot frame to world frame,
            Rrw = np.array([[np.cos(np.radians(pos[2])), -np.sin(np.radians(pos[2]))],
                            [np.sin(np.radians(pos[2])),  np.cos(np.radians(pos[2]))]])
            p_robot = np.array([[pos[0], pos[1]]])
            da_vec = np.array([[d], [a]]) / 100
            p_obs = (Rrw@da_vec) + p_robot                          # world coords of obstacle
            print(f"p_obs = {p_obs}")
            map_obs = (start_position_graph[0] + round(p_obs[1][0] / 0.1524), start_position_graph[1] + round(p_obs[0][0] / 0.1524))        # gets nearest map index to obstacle
            print(f"map_obs = {map_obs}")
            # overcompensate by making obstacle occupy 3x3 space in map
            clearObstacle(map_obs[0], map_obs[1])
            if 1 <= map_obs[0] < len(map) - 1 and 1 <= map_obs[1] < len(map[0]) - 1:
                print(f"Adding {map_obs} to map!")
                obstacleList.append((map_obs[0], map_obs[1]))
                for i in range(-1,2):
                    for j in range(-1,2):
                        map[map_obs[0]+i][map_obs[1]+j] = 7
            # then add to map!

    # ML gives position of box -> find center point of its bottom edge
    # and designate that point's location as (x,y), where:
    #   - x = horizontal location in image
    #   - y = vertical location in image
    # We have focal length in meters => from that, we can determine the
    # height of the image plane in meters using the relationship
    # h_ip = 2*f*tan(23.89)
    # Calculate proportional displacement of bottom edge, i.e. y/y_center
    # => this will be a constant ratio independent of units, call it Y
    # Thus, the metric displacement along the image plane becomes Y*h_ip
    # Have focal length in m and now image "height" in m as well -->
    # dimensions now agree and so can safely take arctangent
    # Thus, phi = arctan(Y*h_ip / f)
    # Already have theta = 61.45 from prior calculations
    # Thus, d' = 32.5tan(theta +- phi) <-- add or subtract depending on
    # whether bottom edge is above or below image's vertical center
    # d' is ground distance along camera axis from camera to bottom edge
    # => to convert to robot frame, just add ~5 cm to d' => d = d' + 5
    # Can do the same process to determine horizontal location a
    # a = d*tan(psi), where psi = arctan(X*w_ip / f), where X = x/x_center
    # and w_ip = 2*f*tan(37.69) = metric width of image plane

    # Thus, have location of obstacle as (d,a) in robot frame
    # => convert to world frame, and add to map!


# Main control loop
def mainLoop():
    global legos_waiting

    trajectory_plot = np.zeros((480, 640, 3), dtype=np.uint8)
    plot_off_x = int(trajectory_plot.shape[1]/2)
    plot_off_y = int(trajectory_plot.shape[0]/2)
    plot_scale = 100

    map_ = path_planning.load_map('map_right.csv')
    graph, _ = path_planning.create_graph(map_)

    # Path planning to go from starting position to river
    # Just do this once, because after this will go from dropoff zone to river
    pr = path_planning.bfs_reverse(graph, river_position_graph)
    path = path_planning.pr_to_path(start_position_graph, pr)
    path = path_planning.process_path(path, start_position_graph)
    threshold = 0.1 # 10cm

    print(path)
    # time.sleep(3)

    i = 0
    idx = 0
    threshold = 0.1 # 10cm
    while idx < len(path):
        velocities = controller(path[idx])
        ep_chassis.drive_speed(x=velocities[0], y=velocities[1], z=0, timeout=0.1)
        if i == 0:
            print(f'position: {pos}')
            print(f'next point: {path[idx]}')
            print(f'velocity: {velocities}')
        i = (i + 1) % 30
        if distance(path[idx]) < threshold:
            print(f'')
            idx += 1
        cv2.circle(trajectory_plot,
                    (int(plot_scale * pos[0] + plot_off_x),
                    int(plot_scale * pos[1] + plot_off_y)),
                    1, (0,0,255), 1)
        cv2.imshow('Trajectory plot', trajectory_plot)
        cv2.waitKey(1)

    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
    # Rotate towards river
    if args.left:
        angle = 60
    else:
        angle = -60
    rotate_to(angle)
    # ep_chassis.drive_speed(x=0, y=0, z=z_speed, timeout=1.2)
    print('*****')
    print('Made it to river')
    print('*****')
    # time.sleep(3)

    while 1:
        print("Start of loop")
        # Check if any LEGOs waiting for pickup
        while legos_waiting == 0:
            # none dropped off atm - idle while waiting for signal
            pass
        print(f"legos_waiting = {legos_waiting} > 0")
        # NN picks up LEGO
        grab_lego()
        ep_arm.moveto(x=86, y=-22).wait_for_completed() # move arm to transit position
        # Rotate back towards original heading
        rotate_to(0)
        # ep_chassis.drive_speed(x=0, y=0, z=z_speed, timeout=1.2)

        # Path planning to go from river to dropoff position
        pr = path_planning.bfs_reverse(graph, dropoff_position_graph)
        path = path_planning.pr_to_path(river_position_graph, pr)
        path = path_planning.process_path(path, start_position_graph)
        threshold = 0.1 # 10cm

        print(path)
        # time.sleep(3)

        i = 0
        idx = 0
        threshold = 0.1 # 10cm
        while idx < len(path):
            velocities = controller(path[idx])
            ep_chassis.drive_speed(x=velocities[0], y=velocities[1], z=0, timeout=0.1)
            if i == 0:
                print(f'position: {pos}')
                print(f'next point: {path[idx]}')
                print(f'velocity: {velocities}')
            i = (i + 1) % 30
            if distance(path[idx]) < threshold:
                print(f'')
                idx += 1
            cv2.circle(trajectory_plot,
                        (int(plot_scale * pos[0] + plot_off_x),
                        int(plot_scale * pos[1] + plot_off_y)),
                        1, (0,0,255), 1)
            cv2.imshow('Trajectory plot', trajectory_plot)
            cv2.waitKey(1)

        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
        print('*****')
        print('Made it to dropoff position')
        print('*****')
        # time.sleep(3)

        # Orient to face dropzone and move forward if necessary
        rotate_to(-180)

        # Extend arm and release
        ep_arm.moveto(x=200, y=50).wait_for_completed() # move arm to transit position
        ep_gripper.open(power=50)
        time.sleep(1)
        ep_gripper.pause()
        legos_waiting = legos_waiting - 1
        # Retract arm, return to starting position/orientation and loop
        ep_arm.moveto(x=86, y=-22).wait_for_completed() # move arm to transit position
        # Back up slightly to not knock over any legos
        ep_chassis.move(x=-0.15, y=0, z=0, xy_speed=0.3).wait_for_completed()
        # Rotate
        rotate_to(0)
        # ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()

        # Path planning to go from dropoff position to river
        pr = path_planning.bfs_reverse(graph, river_position_graph)
        path = path_planning.pr_to_path(dropoff_position_graph, pr)
        path = path_planning.process_path(path, start_position_graph)
        threshold = 0.1 # 10cm

        print(path)
        # time.sleep(3)

        i = 0
        idx = 0
        threshold = 0.1 # 10cm
        while idx < len(path):
            velocities = controller(path[idx])
            ep_chassis.drive_speed(x=velocities[0], y=velocities[1], z=0, timeout=0.1)
            if i == 0:
                print(f'position: {pos}')
                print(f'next point: {path[idx]}')
                print(f'velocity: {velocities}')
            i = (i + 1) % 30
            if distance(path[idx]) < threshold:
                print(f'')
                idx += 1
            cv2.circle(trajectory_plot,
                        (int(plot_scale * pos[0] + plot_off_x),
                        int(plot_scale * pos[1] + plot_off_y)),
                        1, (0,0,255), 1)
            cv2.imshow('Trajectory plot', trajectory_plot)
            cv2.waitKey(1)

        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
        # Rotate towards river
        if args.left:
            angle = 60
        else:
            angle = -60
        rotate_to(angle)
        # ep_chassis.move(x=0, y=0, z=angle, z_speed=45).wait_for_completed()
        print('*****')
        print('Made it to river')
        print('*****')
        # time.sleep(3)
        # Loop!


if __name__ == "__main__":
    # --- PROGRAM STARTUP ---
    parser = argparse.ArgumentParser()
    parser.add_argument('--left', action='store_true')
    parser.add_argument('--right', action='store_true')
    args = parser.parse_args()

    start_position_graph = (2, 3)
    dropoff_position_graph = (2, 2)
    river_position_graph = (8, 11)

    if args.left:
        print('LEFT')
    elif args.right:
        print('RIGHT')
        start_position_graph = (27 - start_position_graph[0], start_position_graph[1])
        dropoff_position_graph = (27 - dropoff_position_graph[0], dropoff_position_graph[1])
        river_position_graph = (27 - river_position_graph[0], river_position_graph[1])
    else:
        print('ERROR: What side are you starting on?')
        exit(1)

    # initialization stuff goes here
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='sta', sn=sns.ROBOT6_SN)
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_chassis = ep_robot.chassis
    ep_chassis.sub_position(cs=0, freq=50, callback=sub_position_handler)
    ep_chassis.sub_attitude(freq=50, callback=sub_attitude_handler)
    time.sleep(1/50)
    start_att[0], start_att[1], start_att[2] = att[0], att[1], att[2]
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    
    host = ''
    port = 13000 
    buf = 1024 
    addr = (host, port) 
    UDPSock = socket(AF_INET, SOCK_DGRAM) 
    UDPSock.bind(addr) 
                    
    # tMain = threading.Thread(target=mainLoop)
    # tListener = threading.Thread(target=signalListener)
    # tObstacles = threading.Thread(target=obstacleDetection)
    # add third thread for obstacle detector
    # tMain.start()
    # tListener.start()
    # tObstacles.start()
    # while True:
    #     print((map == 7).sum())
    mainLoop()