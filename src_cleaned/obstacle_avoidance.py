from tdmclient import ClientAsync
import time
import numpy as np
import atexit 
# This function just goes around the obstacle till sensor distance is less than a threshold from each side (except the back)
# it is a simple implementation, already working on a better one

def move(left, right):
    speed = {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }
    return speed

def led(color):
    led = {
        "leds.top": [color],
    }
    return led


def close_by(node, variables):
    try:
        # check for kidnapping 
        thymio_kidnapped = False
        prox = variables["prox.horizontal"]
        if any([x< 20 for x in variables["prox.ground.delta"]]):
            print("thymio is kidnapped")
            node.send_set_variables(move(0, 0))
            thymio_kidnapped = True
        else:
            # check if thymio was kidnapped and is now free
            if thymio_kidnapped:
                print("thymio is free")
                node.send_set_variables(move(100, 100))
                thymio_kidnapped = False
            else:
                # for x in np.array(variables["prox.horizontal"])[1:3]]) or variables["prox.horizontal"][0] > 3900 or variables["prox.horizontal"][4] > 3900:
                if any([x>3700 for x in prox[1:4]]) or (prox[0] > 3900) or (prox[4] > 3900):
                    print("obstacle is very close")
                    sum_right = prox[4] + 1.2*prox[3]
                    sum_left = prox[0] + 1.2*prox[1]
                    k_avoid = 1/300

                    speed_diff = int(k_avoid*(sum_right-sum_left))
                    
                    speed_left = -speed_diff
                    speed_right = speed_diff
                    node.send_set_variables(move(speed_left, speed_right))
                    state_avoidance = True
                elif state_avoidance:
                    # follow path
                    # if position is not in the middle of the path
                    # call get position function and calculate the error
                    # if error is too big, turn
                    print("no obstacle")
                    node.send_set_variables(move(100, 100))
                    state_avoidance = False

    except KeyError:
        pass


with ClientAsync() as client:
    async def prog():
        with await client.lock() as node:
            await node.watch(variables=True)
            print("watching")
            node.add_variables_changed_listener(close_by)
            # Will sleep forever:
            await client.sleep()
            
            #node.remove_variables_changed_listener(close_by)
    client.run_async_program(prog)

    # close the client once the program is done
    client.close()






    