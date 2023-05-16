import time
from robomaster import robot
import sns


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='sta', sn=sns.ROBOT5_SN)
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm

    ep_gripper.open(power=50)
    time.sleep(2)
    ep_gripper.pause()

    # ep_arm.moveto(x=86, y=-22).wait_for_completed()
    # ep_arm.moveto(x=200, y=50).wait_for_completed()

    ep_robot.close()
