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

def get_closest_lego(image):
    """Closest lego is lego whose Euclidean distance to bottom center pixel is
    smallest."""
    legos = detect_legos(image)
    if len(legos) == 0:
        return None
    goal_x, goal_y = COLS // 2, ROWS
    distance = lambda lego: np.linalg.norm(np.array([lego['x'] - goal_x, lego['y'] - goal_y]))
    closest = min(legos, key=distance)
    sorted_legos = sorted(legos, key=distance)
    print('All:', [(l['x'], l['y'], distance(l)) for l in sorted_legos])
    print('Closest:', closest['x'], closest['y'], distance(closest))
    return closest

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='sta', sn=sns.ROBOT6_SN)
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)

    i = 0

    while True:
        try:
            if i % 30 == 0:
                img = ep_camera.read_cv2_image(strategy='newest', timeout=0.5)   
                legos = detect_legos(img)
                for lego in legos:
                    x1 = round(lego['x'] - lego['width'] / 2)
                    x2 = round(lego['x'] + lego['width'] / 2)
                    y1 = round(lego['y'] - lego['height'] / 2)
                    y2 = round(lego['y'] + lego['height'] / 2)
                    pts = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]]).reshape((-1, 1, 2))
                    img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=1)
                closest_lego = get_closest_lego(img)
                if closest_lego is not None:
                    x1 = round(lego['x'] - lego['width'] / 2)
                    x2 = round(lego['x'] + lego['width'] / 2)
                    y1 = round(lego['y'] - lego['height'] / 2)
                    y2 = round(lego['y'] + lego['height'] / 2)
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
            print ('Exiting')
            exit(1)
