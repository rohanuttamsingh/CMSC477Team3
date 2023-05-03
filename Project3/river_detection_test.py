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


- Path planning brings robot near river, signal that we're about to drop off a LEGO         
- Use OpenCV to correct angle to river at slight distance away                              DONE
- Move forward                                                                              DONE
- Extend arm                                                                                DONE
- Drop LEGO                                                                                 
- Retract arm
- Retreat backward
- Control transfers back to path planning

At arm pos (91, 32), river_y_prop ~= 0.73 at desired location

"""

if __name__ == "__main__":
    def print_pos(p):
        x = p[0]
        y = p[1]
        if y > 4000000000:
            y -= 4294967296
        print(f"Arm pos = {(x, y)}")

    # example that graphs the mask and linear regression side by side
    # in real time with the camera stream
    plt.figure()
    plt.ion()
    plt.show()
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn = sns.ROBOT5_SN)#(conn_type="ap")
    ep_arm = ep_robot.robotic_arm
    ep_arm.sub_position(freq=1, callback=lambda p: print_pos(p))
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis
    # ep_arm.moveto(x=0, y=0).wait_for_completed() # Position of arm while in transit to river
    while True:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=5.0)
            retval = angle_to_river(img)
            if retval is not None:
                ang_disp = retval["ang_disp"]
                #print(ang_disp if ang_disp is not None else "no river detected")  # usually never None, due to noise
                river_y_prop = np.median(retval["riverline"]) / retval["ylim"]
                print(f"angle to river = {retval['ang_disp']}")
                print(f"river y = {river_y_prop}")
                river_adj = np.array(retval["riverline"]) / retval["ylim"]
                river_adj = river_adj[river_adj >= 0.2]
                print(f"river_y_prop = {np.median(river_adj)}")
                #print(f"filtered riverline array length = {len(river_adj)}")
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
            ep_arm.unsub_position()
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)