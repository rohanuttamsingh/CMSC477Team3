import time
import robomaster
from robomaster import robot
def transfer6():
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm
    # close gripper
    ep_gripper.close(power=50)
    time.sleep(5)
    ep_gripper.pause()
