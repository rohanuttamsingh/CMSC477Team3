import time
from robomaster import robot
import sns


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='sta', sn=sns.ROBOT6_SN)
    ep_arm = ep_robot.robotic_arm

    # ep_arm.moveto(x=86, y=-22).wait_for_completed()
    # ep_arm.moveto(x=180, y=-80).wait_for_completed()
    # ep_arm.moveto(x=86, y=-22).wait_for_completed()
    # ep_arm.moveto(x=180, y=-80).wait_for_completed()
    # ep_arm.moveto(x=90, y=20).wait_for_completed()
    # ep_arm.moveto(x=86, y=-22).wait_for_completed()
    # ep_arm.move(x=0, y=-50).wait_for_completed()
    ep_arm.move(x=90, y=60).wait_for_completed()

    ep_robot.close()
