import cv2
from time import sleep
from robomaster import robot
from robomaster import camera
import sns

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='sta', sn=sns.ROBOT6_SN)

    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    images = []
    
    for i in range(20):
        input("Press Enter . . .")
        print(i)
        print("taking photo")
        img = ep_camera.read_video_frame(strategy='newest', timeout=0.5)
        if i % 3 == 0:
            images.append(img)
    ep_camera.stop_video_stream()
    ep_robot.close()
    for i, img in enumerate(images):
        cv2.imwrite(f'{i+105}.jpg', img)
