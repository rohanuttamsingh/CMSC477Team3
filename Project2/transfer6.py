import time
from robomaster import robot

def transfer6(xd,yd):
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm
    # open gripper
    ep_gripper.open(power=50)
    time.sleep(3)
    ep_gripper.pause()
    # Move to xd,yd
    ep_arm.move(x=xd, y=0).wait_for_completed()
    ep_arm.move(x=0, y=yd).wait_for_completed()
    # close gripper
    ep_gripper.close(power=50)
    time.sleep(3)
    ep_gripper.pause()
    # Move to original position
    ep_arm.move(x=0, y=-yd).wait_for_completed()
    ep_arm.move(x=-xd, y=0).wait_for_completed()

    ep_robot.close()

if __name__ == '__main__':
    transfer6(100, -100)
