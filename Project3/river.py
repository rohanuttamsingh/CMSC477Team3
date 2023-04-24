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
lower_blue = np.array([110,100,50])
upper_blue = np.array([130,255,255])

"""
Function that receives a camera frame and returns a dict containing the
estimated angular displacement, as well as a few other potentially useful items.
Full breakdown of returned object:
retval = {
    "ang_disp": the angular displacement,
    "x": the horizontal locations of the image w/ any detected blue pixel,
    "riverline": the estimated vertical position of the river at each x,
    "linreg": the linear regression model based on 'x' and 'riverline',
    "xlim": width of the image,
    "ylim": -1 * the height of the image,
    "mask": the mask of the frame
}
For practical purposes, only the ang_disp value is needed; the rest are provided
for debugging measures if one wishes to visualize the mathematical analysis
being performed.
"""
def angle_to_river(frame):
    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    # Threshold the HSV image to get only blue colors
    mask = cv.inRange(hsv, lower_blue, upper_blue)
    # print("Started calculating...")
    time_s = time.time()
    riverline = []
    x = []
    for i in range(mask.shape[1]):
        if 290 < i < 350:  # ignore the middle part where it's holding a lego
            continue       # to handle the edge case of holding a blue lego
        nz = np.nonzero(mask[:,i])
        if len(nz[0]) > 0:
            x.append(i)
            riverline.append(-1 * np.average(nz))
    # print("Finished calculating...")
    time_e = time.time()
    # print(f"Calculation took {time_e - time_s} s")

    riverline = np.array(riverline)
    x = np.array(x)
    ylim = -1 * mask.shape[0]
    idx = riverline / ylim >= 0.2
    riverline = riverline[idx]
    x = x[idx]

    if len(riverline) > 0:
        coefs = np.polyfit(x, riverline, 1)
        m, b = coefs[0], coefs[1]
        linreg = np.poly1d(coefs)
        # print(f"m={m}, b={b}")
        ang_disp = np.degrees(np.arctan(m))
        # print(f"Angle of line = {ang_disp}")
        # print(f"Range = [{x[0]}, {x[-1]}]")

        """
        If ang_disp < 0, robot should turn right
        If ang_disp > 0, robot should turn left

        ==>

        continuously take camera frames while rotating, and
        stop once the ang_disp falls within tolerance?

        """

        # plt.xlim(0, mask.shape[1])
        # plt.ylim(-1 * mask.shape[0], 0)
        # plt.plot(x, riverline)
        # plt.plot(x, linreg(x))
        # plt.show()
        retval = {
            "ang_disp": ang_disp,
            "x": x,
            "riverline": riverline,
            "linreg": linreg(x),
            "xlim": mask.shape[1],
            "ylim": -1 * mask.shape[0],
            "mask": mask
        }
        return retval
    else:
        return None

# USED FOR DEBUGGING - NOT PART OF MAIN CODE
def oldmain(imgno):
    #cap = cv.VideoCapture(0)
    #while(1):
        # Take each frame
        #_, frame = cap.read()
    frame = cv.imread(f'river_imgs/{imgno}.jpg', cv.IMREAD_COLOR)
    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    # Threshold the HSV image to get only blue colors
    mask = cv.inRange(hsv, lower_blue, upper_blue)
    # Bitwise-AND mask and original image
    res = cv.bitwise_and(frame,frame, mask= mask)
        #cv.imshow('frame',frame)
        #cv.imshow('mask',mask)
        #cv.imshow('res',res)
        # k = cv.waitKey(5) & 0xFF
        # if k == 27:
        #     break
        #break  # TODO remove this line

    cv.destroyAllWindows()

    print("Started calculating...")
    time_s = time.time()
    riverline = []
    riverline_med = []
    x = []
    for i in range(mask.shape[1]):
        nz = np.nonzero(mask[:,i])
        if len(nz[0]) > 0:
            x.append(i)
            riverline.append(-1 * np.average(nz))
            riverline_med.append(-1 * np.median(nz))
    print("Finished calculating...")
    time_e = time.time()
    print(f"Calculation took {time_e - time_s} s")

    """
    TODO address outliers in mask after finding median?
    """
    coefs = np.polyfit(x, riverline, 1)
    coefs2 = np.polyfit(x, riverline_med, 1)
    m, b = coefs[0], coefs[1]
    m2, b2 = coefs2[0], coefs2[1]
    linreg = np.poly1d(coefs)
    linreg2 = np.poly1d(coefs2)
    print(f"m={m}, b={b}\nm2={m2}, b2={b2}")
    ang_disp = np.arctan(m)
    ang_disp2 = np.arctan(m2)
    print(f"Angle of line = {np.degrees(ang_disp)} (using mean) or {np.degrees(ang_disp2)} (using median)")
    print(f"Range = [{x[0]}, {x[-1]}]")

    """
    If ang_disp < 0, robot should turn right
    If ang_disp > 0, robot should turn left

    ==>

    continuously take camera frames while rotating, and
    stop once the ang_disp falls within tolerance?

    """

    plt.subplot(1, 3, 3)
    plt.xlim(0, mask.shape[1])
    plt.ylim(-1 * mask.shape[0], 0)
    plt.plot(x, riverline, 'r')
    plt.plot(x, riverline_med, 'b')
    plt.plot(x, linreg(x), 'm')
    plt.plot(x, linreg2(x), 'c')
    plt.subplot(1, 3, 2)
    plt.imshow(mask)
    plt.subplot(1, 3, 1)
    plt.imshow(cv.cvtColor(frame, cv.COLOR_BGR2RGB))
    plt.show()

# USED FOR DEBUGGING - NOT PART OF MAIN CODE
def colortest():
    frame = cv.imread('river_imgs/2.jpg', cv.IMREAD_COLOR)
    
    # define range of blue color in HSV
    lb = np.array([110,0,50])
    ub = np.array([130,255,255])

    for i in range(11):
        lb = np.array([110, 10*i, 50])
        ub = np.array([130, 255, 255])
        # Convert BGR to HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        # Threshold the HSV image to get only blue colors
        mask = cv.inRange(hsv, lb, ub)

        plt.subplot(1, 2, 1)
        plt.imshow(cv.cvtColor(frame, cv.COLOR_BGR2RGB))
        plt.subplot(1, 2, 2)
        plt.title(f"lb = {np.array2string(lb)}")
        plt.imshow(mask)
        plt.show()
    

    #while 1:
    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    # Threshold the HSV image to get only blue colors
    mask = cv.inRange(hsv, lower_blue, upper_blue)
    # Bitwise-AND mask and original image
    # res = cv.bitwise_and(frame,frame, mask= mask)
    #cv.imshow('frame',frame)
    #cv.imshow('mask',mask)
    #cv.imshow('res',res)
    #k = cv.waitKey(5) & 0xFF
        # if k == 27:
        #     break


if __name__ == "__main__":
    # for i in range(10):
    #     frame = cv.imread(f"river_imgs/{i}.jpg", cv.IMREAD_COLOR)
    #     print(angle_to_river(frame))
    # angle_to_river(cv.imread('river_imgs/9.jpg', cv.IMREAD_COLOR))
    # oldmain()
    # colortest()
    for i in range(10):
        oldmain(i)