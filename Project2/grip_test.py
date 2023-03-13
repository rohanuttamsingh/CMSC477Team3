import time
from robomaster import robot


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper

    # open gripper
    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.pause()

    # Move forward 20mm
    ep_arm.move(x=50, y=0).wait_for_completed()

    # # Move downward 20mm
    ep_arm.move(x=0, y=-75).wait_for_completed()

    # close gripper
    ep_gripper.close(power=50)
    time.sleep(1)
    ep_gripper.pause()

    # Move upward 20mm
    ep_arm.move(x=0, y=75).wait_for_completed()

    # Move backward 20mm
    ep_arm.move(x=-50, y=0).wait_for_completed()

    ep_robot.close()
