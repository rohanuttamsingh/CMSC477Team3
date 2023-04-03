from cone import find_orange
from robomaster import robot
from robomaster import camera
import matplotlib.pyplot as plt
import numpy as np
import sns
import cv2
import imutils
import heapq

class ShapeDetector:
    def __init__(self):
        pass
    def detect(self, c):
        # initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
    # if the shape is a triangle, it will have 3 vertices
        if len(approx) == 3:
            shape = "triangle"
        # if the shape has 4 vertices, it is either a square or
        # a rectangle
        elif len(approx) == 4:
            shape = "rectangle"
            # # compute the bounding box of the contour and use the
            # # bounding box to compute the aspect ratio
            # (x, y, w, h) = cv2.boundingRect(approx)
            # ar = w / float(h)
            # # a square will have an aspect ratio that is approximately
            # # equal to one, otherwise, the shape is a rectangle
            # shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
        # if the shape is a pentagon, it will have 5 vertices
        elif len(approx) == 5:
            shape = "pentagon"
        elif len(approx) == 6:
            shape = "hexagon"
        # otherwise, we assume the shape is a circle
        else:
            shape = "circular?"
        # return the name of the shape
        return {"name": shape, "s": len(approx)}

"""
unrelated to this file, but potentially good resource for finding the landing site:
https://pyimagesearch.com/2016/02/08/opencv-shape-detection/

python "Desktop/CMSC477 Workspace/CMSC477Team3/Project2/cone_detection.py"
"""

def detect_dropoff(img):
    retval = find_orange(img)
    print("Evaluating mask for shapes...")
    cnts = cv2.findContours(retval["mask"].copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    sd = ShapeDetector()
    shapes = []
    for c in cnts:
        # compute the center of the contour, then detect the name of the
        # shape using only the contour
        M = cv2.moments(c)
        if M["m00"] <= 150:
            continue
        cX = int((M["m10"] / M["m00"]))# * ratio)
        cY = int((M["m01"] / M["m00"]))# * ratio)
        if cY >= 275:
            continue
        shape = sd.detect(c)
        print(f"Found a {shape['name']} w/ area {M['m00']} at ({cX, cY})")
        heapq.heappush(shapes, (-1 * shape['s'], M['m00'], cX, cY))
    if len(shapes) > 0:
        if -shapes[0][0] >= 4: # At least 4 sides
            return shapes[0][2], shapes[0][3]
    return None

if __name__ == "__main__":
    # example that graphs the mask and linear regression side by side
    # in real time with the camera stream
    plt.figure()
    plt.ion()
    plt.show()
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn = sns.ROBOT5_SN)#(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=True, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis
    while True:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            retval = find_orange(img)
            print("Evaluating mask for shapes...")
            cnts = cv2.findContours(retval["mask"].copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            sd = ShapeDetector()
            shapes = []
            for c in cnts:
                # compute the center of the contour, then detect the name of the
                # shape using only the contour
                M = cv2.moments(c)
                if M["m00"] <= 200:
                    continue
                cX = int((M["m10"] / M["m00"]))# * ratio)
                cY = int((M["m01"] / M["m00"]))# * ratio)
                if cY >= 275:
                    continue
                shape = sd.detect(c)
                print(f"Found a {shape['name']} w/ area {M['m00']} at ({cX, cY})")
                heapq.heappush(shapes, (-1 * shape['s'], M['m00'], cX, cY))
            print(f"Location of most-sided shape is ({shapes[0][2]}, {shapes[0][3]})")
            plt.clf()
            plt.imshow(retval["mask"])
            plt.pause(1)
        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)