import argparse
import time
import numpy as np
import cv2
from socket import *
from robomaster import robot
from robomaster import camera
import sns
from river import angle_to_river
import threading
import detector
import path_planning
import detector

lego_goal_x = detector.COLS // 2
lego_goal_y = 480
lego_x_threshold = 15

detect_every_n_frames = 5

pos = np.zeros((3,))
def sub_position_handler(p):
    pos[0], pos[1], pos[2] = p[0], p[1], p[2]

att = np.zeros((3,)) # Yaw, pitch, roll
def sub_attitude_handler(a):
    att[0], att[1], att[2] = a[0], a[1], a[2]

def controller(next_position):
    K = [1, 1.2]
    diff = np.array(next_position) - pos[:2]
    return K * diff

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
    ep_arm.moveto(x=180, y=-90).wait_for_completed()

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
                    lego_x, lego_y = detector.get_closest_lego_coords(image)
                    cv2.circle(image, (int(lego_x), int(lego_y)), 5, (255, 255, 255), 5)
                    centered_with_lego = abs(lego_x - lego_goal_x) <= lego_x_threshold
                    z_speed = (lego_x - lego_goal_x) / 5
                    ep_chassis.drive_speed(x=0, y=0, z=z_speed, timeout=0.1)

                # Move forward to lego
                elif not in_front_of_lego:
                    lego_x, lego_y = detector.get_closest_lego_coords(image)
                    cv2.circle(image, (int(lego_x), int(lego_y)), 5, (255, 255, 255), 5)
                    print(lego_y)
                    in_front_of_lego = lego_y <= lego_goal_y
                    if in_front_of_lego:
                        ep_chassis.move(x=0.05, y=0, z=0, xy_speed=0.1).wait_for_completed()
                    else:
                        x_speed = 0.2
                        z_speed = (lego_x - lego_goal_x) / 10
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
                    break

                print(f'found_lego: {found_lego}')
                print(f'centered_with_lego: {centered_with_lego}')
                print(f'in_front_of_lego: {in_front_of_lego}')
                print(f'gripping_lego: {gripping_lego}')

                cv2.imshow("image", image)
                cv2.waitKey(10)
            i = (i + 1) % detect_every_n_frames

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
            image = ep_camera.read_cv2_image(strategy='newest', timeout=5.0)
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
                            ep_chassis.drive_speed(x=0, y=0, z=-20, timeout=5.0)
                    else:
                        print("ang_disp = None ==> no river detected!")
                        ep_chassis.drive_speed(x=0, y=0, z=-20, timeout=5.0)

                elif not at_river:
                    retval = angle_to_river(image)
                    river_y_prop = np.median(retval["riverline"]) / retval["ylim"]
                    print(f"river y = {river_y_prop}")
                    if river_y_prop < 0.73:  # pretty arbitrary cutoff for now - adjust as needed
                        ep_chassis.drive_speed(x=0.2, y=0, z=0, timeout=0.1)
                    else:
                        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
                        break
            i = (i + 1) % detect_every_n_frames

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
    ep_arm.moveto(x=86, y=-22).wait_for_completed()
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

def straighten_bot():
    K = 2
    threshold = 5
    print(att[0])
    while abs(att[0]) > threshold:
        ep_chassis.drive_speed(x=0, y=0, z=-K*att[0], timeout=0.1)
    ep_chassis.drive_speed(x=0, y=0, z=0)

def mainLoop():
    trajectory_plot = np.zeros((480, 640, 3), dtype=np.uint8)
    plot_off_x = int(trajectory_plot.shape[1]/2)
    plot_off_y = int(trajectory_plot.shape[0]/2)
    plot_scale = 100

    map_ = path_planning.load_map('map_left.csv')
    graph, _ = path_planning.create_graph(map_)

    # Path planning to go from starting position to lego source
    # Just do this once, because after this will go from river to lego source
    pr = path_planning.bfs_reverse(graph, source_position_graph)
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
    print('*****')
    print('Made it to lego source')
    print('*****')
    # time.sleep(3)

    # Use NN to pick up LEGO
    grab_lego()
    ep_arm.moveto(x=86, y=-22).wait_for_completed() # move arm to transit position

    # Straighten robot
    # straighten_bot()
    # Move slightly backwards
    ep_chassis.move(x=-0.25, y=0, z=0, xy_speed=0.3).wait_for_completed()

    while True:
        # Path planning to go from source to river
        # current_position_graph = path_planning.real_to_graph_coords((pos[0], pos[1]))
        current_position_graph = source_position_graph
        pr = path_planning.bfs_reverse(graph, river_position_graph)
        path = path_planning.pr_to_path(current_position_graph, pr)
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
        print('Made it to river')
        print('*****')
        # time.sleep(3)

        # Align to river, move forward, and drop LEGO
        drop_at_river()
        # # Send signal to other robot
        # host = "192.168.50.4" # set to IP address of target computer 
        # port = 13000 
        # addr = (host, port) 
        # UDPSock = socket(AF_INET, SOCK_DGRAM) 
        # data = 'lego_dropped'
        # UDPSock.sendto(data.encode(), addr) 
        # UDPSock.close()
        # # Loop!

        # straighten_bot()

        # Path planning to go from river to lego source
        # TODO: Update river position graph to be current position
        # These won't be the same because dropping off the lego impacts our position
        pr = path_planning.bfs_reverse(graph, source_position_graph)
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
        print('Made it to lego source')
        print('*****')
        # time.sleep(3)

        # Use NN to pick up LEGO
        grab_lego()
        ep_arm.moveto(x=86, y=-22).wait_for_completed() # move arm to transit position

        # Reverse slightly backward
        ep_chassis.move(x=-0.25, y=0, z=0, xy_speed=0.3).wait_for_completed()

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

def test_straighten_bot():
    # ep_chassis.move(x=0, y=0, z=45).wait_for_completed()
    # straighten_bot()
    # print('Straight')
    # ep_chassis.move(x=0, y=0, z=-90).wait_for_completed()
    # straighten_bot()
    # print('Straight')
    ep_chassis.move(x=0.5, y=-0.2, z=37).wait_for_completed()
    straighten_bot()
    print('Straight')

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--left', action='store_true')
    parser.add_argument('--right', action='store_true')
    args = parser.parse_args()

    start_position_graph = (1, 1)
    source_position_graph = (13, 1)
    river_position_graph = (13, 11)

    if args.left:
        print('LEFT')
    elif args.right:
        print('RIGHT')
        start_position_graph = (27 - start_position_graph[0], start_position_graph[1])
        source_position_graph = (27 - source_position_graph[0], source_position_graph[1])
        river_position_graph = (27 - river_position_graph[0], river_position_graph[1])
    else:
        print('ERROR: What side are you starting on?')
        exit(1)

    # initialization stuff goes here
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='sta', sn=sns.ROBOT5_SN)
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_chassis = ep_robot.chassis
    ep_chassis.sub_position(cs=0, freq=50, callback=sub_position_handler)
    # ep_chassis.sub_attitude(freq=50, callback=sub_attitude_handler)
    ep_arm = ep_robot.robotic_arm
    ep_arm.moveto(x=86, y=-22).wait_for_completed()
    ep_gripper = ep_robot.gripper


    host = '192.168.50.220'
    port = 13000 
    buf = 1024 
    addr = (host, port) 
    UDPSock = socket(AF_INET, SOCK_DGRAM) 

    # tMain = threading.Thread(target=mainLoop)
    # tObstacles = threading.Thread(target=obstacleDetection)  # replace with obstacle detector
    # tMain.start()
    # tObstacles.start()

    mainLoop()
    # grab_lego()
    # test_straighten_bot()
