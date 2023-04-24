import time
import robomaster
from robomaster import robot

def transfer5():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm
    #Sleep for 10 seconds once transfer5 is called 
    time.sleep(10)
    # open gripper
    ep_gripper.open(power=50)
    time.sleep(5)
    ep_gripper.pause()


 

   
