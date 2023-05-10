# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""
forward, bottom-most position: (208, -69)

"""


import robomaster
from robomaster import robot
import time
import sns
xd = 170
yd = 65

if __name__ == '__main__':

    def print_pos(p):
        x = p[0]
        y = p[1]
        if y > 4000000000:
            y -= 4294967296
        print((x, y))

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn=sns.ROBOT5_SN)
        
    ep_arm = ep_robot.robotic_arm
    ep_arm.sub_position(freq=5, callback=lambda p: print_pos(p))

    ep_gripper = ep_robot.gripper
    # open gripper
    ep_gripper.open(power=50)
    time.sleep(3)
    ep_gripper.pause()
    #ep_arm.moveto(x=208, y=-69).wait_for_completed()
   
    # ep_arm.moveto(x=71, y=30).wait_for_completed()
    # time.sleep(2)
    # ep_arm.move(x=140, y=-80).wait_for_completed()
    # ep_arm.move(x=0, y=-160).wait_for_completed()
    # Move forward 20mm
    # ep_arm.moveto(x=71, y=30).wait_for_completed()
    # ep_arm.moveto(x=71, y=yd).wait_for_completed()
    # #ep_arm.moveto(x=xd, y=yd).wait_for_completed()
    # #ep_arm.moveto(x=xd, y=30).wait_for_completed()
    # ep_arm.moveto(x=71, y=30).wait_for_completed()
    
    
    
    
    # for i in range(150, 250, 5):
    #     ep_arm.moveto(x=i, y=0).wait_for_completed()
    #time.sleep(3)
    #ep_arm.moveto(x=xd, y=yd).wait_for_completed()
    #ep_arm.moveto(x=xd, y=yd).wait_for_completed()


    # close gripper
    # ep_gripper.close(power=50)
    # time.sleep(3)
    # ep_gripper.pause()
   
    # time.sleep(3)
    # ep_arm.moveto(x=xd, y=-yd/2).wait_for_completed()
    # #ep_arm.moveto(x=-xd, y=0).wait_for_completed()

    # # open gripper
    # ep_gripper.open(power=50)
    # time.sleep(3)
    # ep_gripper.pause()
    # # ep_arm.moveto(x=0, y=0).wait_for_completed()

    # ep_arm.moveto(x=200, y=30).wait_for_completed()

    time.sleep(2)

    ep_arm.unsub_position()

    ep_robot.close()
