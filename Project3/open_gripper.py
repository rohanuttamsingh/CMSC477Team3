import time
from robomaster import robot
import sns


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='sta', sn=sns.ROBOT6_SN)
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm

    ep_gripper.open(power=50)
    time.sleep(2)
    ep_gripper.pause()


    ep_robot.close()
