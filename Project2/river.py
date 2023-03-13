"""

detecting the "river" and adjusting position:

ORIENTATION:
- rotate bot until river intersects both the left and right side of the camera view
- continue to rotate until the river is exactly horizontal across the camera view


DISTANCE:
- we want the line to be roughly horizontal at some location vaguely near the bottom
  of the camera view
"""

import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import time

# define range of blue color in HSV
lower_blue = np.array([110,0,50])
upper_blue = np.array([130,255,255])

##
#  frame = `Mat` object type representing the current camera frame
def angle_to_river(frame):
    frame = cv.imread('3.jpg', cv.IMREAD_COLOR)
    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    # Threshold the HSV image to get only blue colors
    mask = cv.inRange(hsv, lower_blue, upper_blue)
    print("Started calculating...")
    time_s = time.time()
    riverline = []
    x = []
    for i in range(mask.shape[1]):
        nz = np.nonzero(mask[:,i])
        if len(nz[0]) > 0:
            x.append(i)
            riverline.append(-1 * np.average(nz))
    print("Finished calculating...")
    time_e = time.time()
    print(f"Calculation took {time_e - time_s} s")

    coefs = np.polyfit(x, riverline, 1)
    m, b = coefs[0], coefs[1]
    linreg = np.poly1d(coefs)
    print(f"m={m}, b={b}")
    ang_disp = np.degrees(np.arctan(m))
    print(f"Angle of line = {ang_disp}")
    print(f"Range = [{x[0]}, {x[-1]}]")

    """
    If ang_disp < 0, robot should turn right
    If ang_disp > 0, robot should turn left

    ==>

    continuously take camera frames while rotating, and
    stop once the ang_disp falls within tolerance?

    """

    plt.xlim(0, mask.shape[1])
    plt.ylim(-1 * mask.shape[0], 0)
    plt.plot(x, riverline)
    plt.plot(x, linreg(x))
    plt.show()
    
    return ang_disp


if __name__ == "__main__":
    #cap = cv.VideoCapture(0)
    while(1):
        # Take each frame
        #_, frame = cap.read()
        frame = cv.imread('3.jpg', cv.IMREAD_COLOR)
        # Convert BGR to HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        # Threshold the HSV image to get only blue colors
        mask = cv.inRange(hsv, lower_blue, upper_blue)
        # Bitwise-AND mask and original image
        res = cv.bitwise_and(frame,frame, mask= mask)
        cv.imshow('frame',frame)
        cv.imshow('mask',mask)
        cv.imshow('res',res)
        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break
        #break  # TODO remove this line

    cv.destroyAllWindows()

    print("Started calculating...")
    time_s = time.time()
    riverline = []
    x = []
    for i in range(mask.shape[1]):
        nz = np.nonzero(mask[:,i])
        if len(nz[0]) > 0:
            x.append(i)
            riverline.append(-1 * np.average(nz))
    print("Finished calculating...")
    time_e = time.time()
    print(f"Calculation took {time_e - time_s} s")

    coefs = np.polyfit(x, riverline, 1)
    m, b = coefs[0], coefs[1]
    linreg = np.poly1d(coefs)
    print(f"m={m}, b={b}")
    ang_disp = np.arctan(m)
    print(f"Angle of line = {np.degrees(ang_disp)}")
    print(f"Range = [{x[0]}, {x[-1]}]")

    """
    If ang_disp < 0, robot should turn right
    If ang_disp > 0, robot should turn left

    ==>

    continuously take camera frames while rotating, and
    stop once the ang_disp falls within tolerance?

    """

    plt.xlim(0, mask.shape[1])
    plt.ylim(-1 * mask.shape[0], 0)
    plt.plot(x, riverline)
    plt.plot(x, linreg(x))
    plt.show()