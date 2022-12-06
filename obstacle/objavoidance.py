import tdmclient
from tdmclient import ClientAsync, aw
import time
import math
import asyncio
def motors(l_speed=500, r_speed=500, verbose=False):
    return {
        "motor.left.target": [l_speed],
        "motor.right.target": [r_speed],
    }



def object_close(node,wall_threshold, verbose=False):
    if any([x>wall_threshold for x in node['prox.horizontal'][:-2]]):
        return True
    return False

async def follow_obj(node,client,motor_speed=70, wall_threshold=1000, white_threshold=200, verbose=False):

    await node.set_variables(motors(motor_speed, motor_speed))
           
    prev_state="forward"
    goal = False
    # number of obstacle avoided
    num_obstacle = 0
    while not goal:
        
        if object_close(node,wall_threshold):
            if prev_state=="forward": 
                await node.set_variables(motors(motor_speed, -motor_speed))
                prev_state="turning"
                num_obstacle += 1
                print("Obstacle detected, turning")
        if num_obstacle > 200:
            goal = True
            print("Goal reached")
            await node.set_variables(motors(0, 0))
            break 
        else:
            if prev_state=="turning": 
                await node.set_variables(motors(motor_speed, motor_speed))
                prev_state="forward"

        await client.sleep(0.1) 
        # exit after 200 obstacles
      

    return 



async def main():
    client = ClientAsync()
    node = await client.lock()
    await node.wait_for_variables()
    await follow_obj(node,client)
    await client.sleep(10)
    await node.set_variables(motors(0, 0))

    # if close then set motors to 0,0


    #time.sleep(5)
    #node.send_set_variables(move_obj.motors(0,0))
    #time.sleep(2)
    #await move_obj.turn_right(10)
    #time.sleep(2)
    #await move_obj.stop()
    #print("Variables loaded", node.var)
    

# execute the main function
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.close()


 