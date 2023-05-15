import cv2
import numpy as np
from roboflow import Roboflow
from robomaster import robot
from robomaster import camera

import sns

ROWS = 720
COLS = 1280

rf = Roboflow(api_key='E1vVvbJyFSDbkTIDLJIV')
lego_detector_project = rf.workspace().project('legodetector')
lego_detector = lego_detector_project.version(1).model
nav_detector_project = rf.workspace().project('navdetector')
nav_detector = nav_detector_project.version(1).model

def detect(model, image):
    return model.predict(image, confidence=40, overlap=30).json()['predictions']

def detect_legos(image):
    return detect(lego_detector, image)

def detect_nav(image):
    return detect(nav_detector, image)

def can_see_lego(image):
    legos = detect_legos(image)
    return len(legos) > 0

def get_closest_lego(image):
    """Closest lego is lego whose Euclidean distance to bottom center pixel is
    smallest."""
    legos = detect_legos(image)
    if len(legos) == 0:
        return None
    goal_x, goal_y = COLS // 2, ROWS
    distance = lambda lego: np.linalg.norm(np.array([lego['x'] - goal_x, lego['y'] - goal_y]))
    closest = min(legos, key=distance)
    # sorted_legos = sorted(legos, key=distance)
    # print('All:', [(l['x'], l['y'], distance(l)) for l in sorted_legos])
    # print('Closest:', closest['x'], closest['y'], distance(closest))
    return closest

def get_closest_lego_coords(image):
    closest = get_closest_lego(image)
    return closest['x'], closest['y']

def get_corners(o):
    x1 = round(o['x'] - o['width'] / 2)
    x2 = round(o['x'] + o['width'] / 2)
    y1 = round(o['y'] - o['height'] / 2)
    y2 = round(o['y'] + o['height'] / 2)
    return x1, x2, y1, y2

def get_objects(model, cls, image):
    return list(filter(lambda o: o['class'] == cls, detect(model, image)))

def get_obstacles(image):
    return get_objects(nav_detector, 'obstacle', image)

def get_obstacle_offset_from_center(obstacle):
    """Returns (x distance from center of object to center of camera frame,
    y distance from bottom of object to center of camera frame)"""
    x = obstacle['x'] - COLS // 2
    y = (obstacle['y'] + obstacle['height'] / 2) - ROWS // 2
    return x, y

def get_robots(image):
    return get_objects(nav_detector, 'robot', image)

def get_distance_to_robot(robot):
    """Determine distance to robot based on its pixel height."""
    y = robot['height'] # cm
    Y = 30 # cm
    f = 184.752*1.7 # pixels
    Z = f * Y / y
    D = np.sqrt(abs(Z**2 - Y**2)) # distance to robot
    scale = 3 # derived through experimentation, accounts for inaccuracies
    return D * scale

def get_close_robot_x_centers(image):
    close = 20 # cm
    robots = get_robots(image)
    centers = []
    for robot in robots:
        if get_distance_to_robot(robot) < close:
            centers.append(robot['x'])
    return centers

def test_lego_detector():
    i = 0
    while True:
        try:
            if i % 30 == 0:
                img = ep_camera.read_cv2_image(strategy='newest', timeout=5.0)   
                legos = detect_legos(img)
                for lego in legos:
                    x1, x2, y1, y2 = get_corners(lego)
                    pts = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]]).reshape((-1, 1, 2))
                    img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=1)
                closest_lego = get_closest_lego(img)
                if closest_lego is not None:
                    x1, x2, y1, y2 = get_corners(closest_lego)
                    pts = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]]).reshape((-1, 1, 2))
                    img = cv2.polylines(img, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
                img = cv2.line(img, (COLS // 2, 0), (COLS // 2, ROWS), color=(255, 0, 0), thickness=1)
                img = cv2.line(img, (0, ROWS - 1), (COLS, ROWS - 1), color=(255, 0, 0), thickness=1)
                cv2.imshow("img", img)
                cv2.waitKey(10)
            i = (i + 1) % 30

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)

def test_robot_detector():
    i = 0
    while True:
        try:
            if i % 30 == 0:
                img = ep_camera.read_cv2_image(strategy='newest', timeout=5.0)
                robots = get_robots(img)
                for robot in robots:
                    x1, x2, y1, y2 = get_corners(robot)
                    pts = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]]).reshape((-1, 1, 2))
                    img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=1)
                    print(get_distance_to_robot(robot))
                cv2.imshow("img", img)
                cv2.waitKey(10)
            i = (i + 1) % 30

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='sta', sn=sns.ROBOT6_SN)
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)

    # test_lego_detector()
    test_robot_detector()
