from pupil_apriltags import Detector
import cv2
import numpy as np
from robomaster import robot
from robomaster import camera
import bonus_pd

if __name__ == "__main__":
    """
    Program flow:
        startup -> assume roughly on starting point, facing towards 32
        detect 32 and figure out exact location
        calculate path
        ...
    """
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis

    tag_size=0.16 # tag size in meters

    target = np.array([0, 0.5]) # 0.5 m away from tag

    img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)   
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray.astype(np.uint8)

    K=np.array([[184.752, 0, 320], [0, 184.752, 180], [0, 0, 1]])

    results = bonus_pd.at_detector.detect(gray, estimate_tag_pose=False)

    for res in results:
        # If AprilTag in frame, only 1 result 
        pose = bonus_pd.find_pose_from_tag(K, res)
        rot, jaco = cv2.Rodrigues(pose[1], pose[1])

        pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
        img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
        cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)
        id = res.tag_id.decode("utf-8")
        print(id)
        t = np.array([pose[0][0], pose[0][2]])

