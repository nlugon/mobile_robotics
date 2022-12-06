import time 
import math
from tdmclient import ClientAsync, aw
import asyncio
import json


class Movement:

    def __init__(self, robot):
        self.robot_ctr = robot
        self.forward_speed = 100 # just  to test
        self.right_wheel_offset = 0
        self.left_wheel_offset = 0
        self.rotation_speed = 100
        self.rot_delta_t= 10

    async def motors(self,left, right):
        return {
            "motor.left.target": [left],
            "motor.right.target": [right],
        }
    async def stop(self):
        await self.robot_ctr.set_variables({"motor.left.target": [0],
                                            "motor.right.target": [0]})

    async def step_forward(self,forward_t):
        await self.robot_ctr.set_variables({"motor.left.target": [self.forward_speed + self.left_wheel_offset],
                                            "motor.right.target": [self.forward_speed + self.right_wheel_offset]})

        time.sleep(forward_t)
        await self.stop()

    async def turn_left(self,diff_orient):
        #90 degree turn
        await self.robot_ctr.set_variables({"motor.left.target": [2**16-self.rotation_speed],
                                            "motor.right.target": [self.rotation_speed]})
  
        #time.sleep(diff_orient*self.rot_delta_t)
        time.sleep(2.33) 
        await self.stop()

    async def turn_right(self,diff_orient):
        #90 degree turn
        await self.robot_ctr.set_variables({"motor.left.target": [self.rotation_speed],
                                            "motor.right.target": [2**16-self.rotation_speed]})
 
        time.sleep(2.33) 
        await self.stop()
    

    def turn(self,ang):
        if ang > 0:
            self.turn_right(ang)
        else:
            self.turn_left(-ang)

    

        
    def regulateAng(self,ang):
        if ang > math.pi:
            return ang - 2 * math.pi
        elif ang < -math.pi:
            return ang + 2 * math.pi
        else:
            return ang

async def main():
    client = ClientAsync()
    node = await client.lock()
    await node.wait_for_variables()
    move_obj = Movement(node)
    #node.send_set_variables(move_obj.motors(100,100))
    #time.sleep(5)
    #node.send_set_variables(move_obj.motors(0,0))
    #time.sleep(2)
    #await move_obj.turn_right(10)
    #time.sleep(2)
    #await move_obj.stop()
    await move_obj.step_forward(3)
    await move_obj.turn_right(3)
    await move_obj.turn_left(3)
    turn_ang = math.pi/4
    move_obj.turn(turn_ang)
    print("Variables loaded", node.var)
    

# execute the main function
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.close()