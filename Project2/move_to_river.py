from river import angle_to_river
from robomaster import robot
from robomaster import camera
import matplotlib.pyplot as plt
import numpy as np
import sns

"""
unrelated to this file, but potentially good resource for finding the landing site:
https://pyimagesearch.com/2016/02/08/opencv-shape-detection/

python "Desktop/CMSC477 Workspace/CMSC477Team3/Project2/move_to_river.py"
"""

if __name__ == "__main__":
    # plt.figure()
    # plt.ion()
    # plt.show()
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn = sns.ROBOT5_SN)#(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=True, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis

    angled = True
    at_river = False

    while True:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            retval = angle_to_river(img)
            river_adj = np.array(retval["riverline"]) / retval["ylim"]
            river_adj = river_adj[river_adj >= 0.2]
            if angled:
                ang_disp = retval["ang_disp"]
                if retval is not None:
                    if ang_disp > 3:
                        print(f"ang_disp = {ang_disp} ==> turn to left")
                        ep_chassis.drive_speed(x=0, y=0, z=-3, timeout=0.1)
                    elif ang_disp < -3:
                        print(f"ang_disp = {ang_disp} ==> turn to right")
                        ep_chassis.drive_speed(x=0, y=0, z=3, timeout=0.1)
                    else:
                        river_y_avg = np.average(retval["riverline"])
                        river_y_med = np.median(retval["riverline"])
                        print(f"ang_disp = {ang_disp} ==> do not turn; riverline at {river_y_avg} (avg) or {river_y_med} (median) out of {retval['ylim']}")
                        angled = False
                else:
                    print("ang_disp = None ==> no river detected!")
            elif not at_river:
                river_y_prop = np.median(retval["riverline"]) / retval["ylim"]
                print(f"river y = {river_y_prop}")
                if river_y_prop < 0.935:  # pretty arbitrary cutoff for now - adjust as needed
                    ep_chassis.drive_speed(x=0.05, y=0, z=0, timeout=0.1)
                # elif river_y_prop > 0.94:
                #     ep_chassis.drive_speed(x=-0.05, y=0, z=0, timeout=0.1)
                else:
                    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
                    at_river = True
            else:
                break
            # plt.clf()
            # plt.subplot(1, 2, 1)
            # plt.xlim(0, retval["xlim"])
            # plt.ylim(retval["ylim"], 0)
            # plt.plot(retval["x"], retval["riverline"])
            # plt.plot(retval["x"], retval["linreg"])
            # plt.draw()
            # plt.subplot(1, 2, 2)
            # plt.imshow(retval["mask"])
            # plt.pause(0.1)
            """
            TODO from sign of ang_disp move left or right appropriately
             --> maybe move by small incrememnt then check camera again?
            """
        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)
