from river import angle_to_river
from robomaster import robot
from robomaster import camera
import matplotlib.pyplot as plt
import numpy as np
import sns

"""
unrelated to this file, but potentially good resource for finding the landing site:
https://pyimagesearch.com/2016/02/08/opencv-shape-detection/

python "Desktop/CMSC477 Workspace/CMSC477Team3/Project2/river_detection_test.py"
"""

if __name__ == "__main__":
    # example that graphs the mask and linear regression side by side
    # in real time with the camera stream
    plt.figure()
    plt.ion()
    plt.show()
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn = sns.ROBOT6_SN)#(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=True, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis
    while True:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            retval = angle_to_river(img)
            ang_disp = retval["ang_disp"]
            #print(ang_disp if ang_disp is not None else "no river detected")  # usually never None, due to noise
            river_y_prop = np.median(retval["riverline"]) / retval["ylim"]
            print(f"river y = {river_y_prop}")
            plt.clf()
            plt.subplot(1, 2, 1)
            plt.xlim(0, retval["xlim"])
            plt.ylim(retval["ylim"], 0)
            plt.plot(retval["x"], retval["riverline"])
            plt.plot(retval["x"], retval["linreg"])
            plt.draw()
            plt.subplot(1, 2, 2)
            plt.imshow(retval["mask"])
            plt.pause(0.1)
            """
            TODO from sign of ang_disp move left or right appropriately
             --> maybe move by small incrememnt then check camera again?
            """
        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)