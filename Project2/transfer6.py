import time
import robomaster
from robomaster import robot
def transfer6(xd,yd):
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm
    # open gripper
    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.pause()
    # Move to xd,yd
    ep_arm.move(x=xd, y=yd).wait_for_completed()
    # close gripper
    ep_gripper.close(power=50)
    time.sleep(10)
    ep_gripper.pause()
    # Move to original position
    ep_arm.move(x=-xd, y=-yd).wait_for_completed()
