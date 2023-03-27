import time
import robomaster
from robomaster import robot

def pick():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm
    # open gripper
    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.pause()
    # Move to 150,-40 which is where the block will be 
    ep_arm.move(x=150, y=-50).wait_for_completed()
    # close gripper
    ep_gripper.close(power=50)
    time.sleep(1)
    ep_gripper.pause()
    # Move to original position
    ep_arm.move(x=-130, y=50).wait_for_completed()
    ep_robot.close()

if __name__ == '__main__':
    pick()
