import time
import numpy as np
from socket import *
from robomaster import robot
from robomaster import camera
import sns
import utils
from river import angle_to_river
import threading
import detector
import path_planning

goal_x = utils.image_width // 2
goal_y = 285

pos = np.zeros((3,))
def sub_position_handler(p):
    # Mapping from robot coordinate system to map coordinate system
    m_to_ft = 3.28084
    # x and y are swapped
    # TODO: Sign of movement varies, test this on every boot to detemrine what
    # should be flipped
    pos[0], pos[1], pos[2] = -p[1] * m_to_ft, -p[0] * m_to_ft, p[2]

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
                # Spin to find lego
                if not found_lego:
                    found_lego = detector.can_see_lego(image)
                    ep_chassis.drive_speed(x=0, y=0, z=-20, timeout=0.5)

                # Spin to center lego
                elif not centered_with_lego:
                    lego_x, _ = detector.get_closest_lego_coords(image)
                    centered_with_lego = goal_x - utils.threshold <= lego_x <= goal_x + utils.threshold
                    z_speed = (lego_x - goal_x) / 10
                    ep_chassis.drive_speed(x=0, y=0, z=z_speed, timeout=0.5)

                # Move forward to lego
                elif not in_front_of_lego:
                    lego_x, lego_y = detector.get_closest_lego_coords(image)
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
                    return
                
                # Grabbed lego => return to main loop
                else:
                    return
            i = (i + 1) % utils.detect_every_n_frames
        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)


def drop_at_river():
    i = 0
    angled = True
    at_river = False

    # move up right next to river
    while True:
        try:
            image = ep_camera.read_cv2_image(strategy='newest', timeout=0.5)
            if i == 0:
                if angled:
                    retval = angle_to_river(image)
                    if retval is not None:
                        riverline = retval['riverline']
                        if len(riverline) >= 100:
                            ang_disp = retval["ang_disp"]
                            if ang_disp > 5:
                                print(f"ang_disp = {ang_disp} ==> turn to left")
                                ep_chassis.drive_speed(x=0, y=0, z=-20, timeout=0.1)
                            elif ang_disp < -5:
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
                    if river_y_prop < 0.73:  # pretty arbitrary cutoff for now - adjust as needed
                        ep_chassis.drive_speed(x=0.2, y=0, z=0, timeout=0.1)
                    else:
                        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
                        break
            i = (i + 1) % utils.detect_every_n_frames
        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)
    # angularly aligned at ~10 cm away -> move forward
    print("aligned; moving forward")
    ep_chassis.move(x=0.1, y=0, z=0, xy_speed=0.1).wait_for_completed()
    print("moved forward; ready to drop")
    ep_arm.moveto(x=211, y=-61).wait_for_completed()
    print("arm in dropping position")
    # open gripper to drop LEGO
    ep_gripper.open(power=50)
    time.sleep(3)
    ep_gripper.pause()
    # return arm to transit position and move backward
    ep_arm.moveto(x=91, y=-32).wait_for_completed()
    ep_chassis.move(x=-0.1, y=0, z=0, xy_speed=0.1).wait_for_completed()
    return


# Looks for obstacles and adds them to the map
def obstacleDetection():
    # assume we have some function that returns phi and psi, where
    #   - phi = vertical angular offset from camera axis
    #   - psi = horizontal angular offset from camera axis
    # then, in camera frame, we can calculate:
    #   - depth from robot is d = 32.5tan(phi + theta)
    #   - horizontal displacement is a = d*tan(psi)
    # => box is at (d,a) in camera frame
    # camera is located in front of robot center
    # => to get to robot frame, add 5 cm to d
    # => box is at (d+5,a) in robot frame
    # => convert to world coordinates before adding to map
    pass

def distance(idx, path):
    """Distance to point at index idx in path."""
    diff = np.array(path[idx]) - pos[:2]
    return np.linalg.norm(diff)

def get_xy_velocity(idx, path):
    """Velocity to get to point at index idx in path."""
    K = 0.25
    # Same velocity on x and y moves y half as fast as x
    Kx = K
    Ky = 1.5 * Kx
    # Ky = Kx
    velocity = np.array(path[idx]) - pos[:2]
    velocity[0], velocity[1] = velocity[1], velocity[0]
    velocity[0] *= Kx
    velocity[1] *= Ky
    # Feedforward velocity is velocity from last waypoint to this waypoint
    # feedforward = np.array(path[idx]) - np.array(path[idx - 1])
    # feedforward[0] *= Kx / 2
    # feedforward[1] *= Ky / 2
    # velocity += feedforward
    return velocity

def mainLoop():
    map_ = path_planning.load_map('map_left.csv')
    while True:
        # Path planning to go to lego source
        curr_position = tuple(round(p) for p in pos[:2]) # TODO: Round to nearest 0.5
        source_position = (13, 4) # Lego source
        graph, _ = path_planning.create_graph(map_)
        pr = path_planning.bfs_reverse(graph, source_position)
        path = path_planning.scale_path(path_planning.pr_to_path(curr_position, pr))
        threshold = 0.2 # feet

        print(path)
        time.sleep(3)

        # Move to source
        idx = 1
        i = 0
        while True:
            if idx == len(path) - 1:
                print('Made it to lego source')
                break
            if distance(idx, path) <= threshold:
                print(f'Passed point {idx}')
                idx += 1
            xy_velocity = get_xy_velocity(idx, path)
            if i == 0:
                print('position:', pos)
                print('next point:', path[idx])
                print('velocity:', xy_velocity)
            i = (i + 1) % 30
            ep_chassis.drive_speed(x=xy_velocity[0], y=xy_velocity[1], z=0, timeout=0.1)

        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
        time.sleep(3)

        # Move slightly forward
        # grab_lego() already involves advancing toward LEGO to pick it up
        # => maybe don't need this as an explicit separate step in the loop?
        # # Use NN to pick up LEGO
        # grab_lego()
        # ep_arm.moveto(x=91, y=-32).wait_for_completed() # move arm to transit position

        # Use NN to pick up LEGO
        ep_gripper.open(power=50)
        time.sleep(3)
        ep_gripper.pause()
        ep_arm.moveto(x=208, y=-69).wait_for_completed()
        grab_lego()
        ep_arm.moveto(x=91, y=-32).wait_for_completed() # move arm to transit position
        # Reverse slightly backward
        break
        # # Reverse slightly backward
        # ep_chassis.move(x=-0.5, y=0, z=0, xy_speed=0.3).wait_for_completed()

        # Path planning to go to river
        curr_position = tuple(round(p) for p in pos[:2])
        river_position = (13, 14) # River, may need to change this
        graph, _ = path_planning.create_graph(map_)
        pr = path_planning.bfs_reverse(graph, river_position)
        path = path_planning.scale_path(path_planning.pr_to_path(curr_position, pr))
        threshold = 0.2 # feet

        print(path)
        time.sleep(3)

        # Move to river
        idx = 1
        i = 0
        while True:
            if idx == len(path) - 1:
                print('Made it to river')
                break
            if distance(idx, path) <= threshold:
                print(f'Passed point {idx}')
                idx += 1
            xy_velocity = get_xy_velocity(idx, path)
            if i == 0:
                print('position:', pos)
                print('next point:', path[idx])
                print('velocity:', xy_velocity)
            i = (i + 1) % 30
            ep_chassis.drive_speed(x=xy_velocity[0], y=xy_velocity[1], z=0, timeout=0.1)

        # # Align to river, move forward, and drop LEGO
        # drop_at_river()
        # # Send signal to other robot
        # host = "192.168.50.4" # set to IP address of target computer 
        # port = 13000 
        # addr = (host, port) 
        # UDPSock = socket(AF_INET, SOCK_DGRAM) 
        # data = 'lego_dropped'
        # UDPSock.sendto(data.encode(), addr) 
        # UDPSock.close()
        # # Loop!

    # Loop:
    #   Picker starts at starting location
    #   Move to LEGO source zone
    #       - have from P1 -> Dijkstra's + move
    #   Reach in and pick up LEGO (may involve slight forward movement)
    #       - drop arm to (180-210, -70) at safe distance away
    #       - have from P2 -> NN to detect, then align and grab
    #   Step back if necessary, then travel to river
    #       - have from P1 -> Dijkstra's + move
    #   Position as close as desired to river
    #       - have from P2 -> move_to_river() [adjust thresholds]
    #   Drop the LEGO on the other side
    #       - have from last week
    #   Signal to other robot that LEGO is ready for pickup
    #       - have from P2 -> socket stuff
    #   Return to LEGO source zone and loop
    #       - have from P1 -> Dijkstra's + move

    # NOTE: throughout entire above loop, want to be constantly checking for
    # static (boxes) and dynamic (enemy bot) obstacles and updating our map
    # ==> consider delegating this to a separate thread to run concurrently?

    # NOTE 2: on obstacles - best to account for static obstacles by simply
    # adding to the map then recalculating Dijkstra's as needed; for the other
    # robot, we have options: either stop and wait for it to leave frame, or
    # attempt to maneuever around it


if __name__ == "__main__":
    # initialization stuff goes here
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='sta', sn=sns.ROBOT6_SN)
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_chassis = ep_robot.chassis
    ep_chassis.sub_position(cs=0, freq=50, callback=sub_position_handler)
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    
    host = ''
    port = 13000 
    buf = 1024 
    addr = (host, port) 
    UDPSock = socket(AF_INET, SOCK_DGRAM) 
    UDPSock.bind(addr) 
                    
    tMain = threading.Thread(target=mainLoop)
    # tObstacles = threading.Thread(target=obstacleDetection)  # replace with obstacle detector
    tMain.start()
    # tObstacles.start()
