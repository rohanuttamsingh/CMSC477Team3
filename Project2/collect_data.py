import cv2
from robomaster import robot
from robomaster import camera

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    images = []
    
    for i in range(50):
        input("Press Enter . . .")
        print(i)
        print("taking photo")
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.1)
        images.append(img)
    ep_camera.stop_video_stream()
    ep_robot.close()
    for i, img in enumerate(images):
        cv2.imwrite(f'{i}.jpg', img)
