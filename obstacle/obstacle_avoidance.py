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


                if any([x>3000 and x < 3200 for x in variables["prox.horizontal"]]):
                    print("obstacle is very close") 
                    avoid = False
                    #prox = variables["prox.horizontal"]
                    var_prox= np.array(variables["prox.horizontal"])
                    if (var_prox[:1].sum() > var_prox[3:4].sum()):
                        node.send_set_variables(move(100, -100))
                    elif var_prox[2] > 3000: 
                        node.send_set_variables(move(100, -100))
                    else:
                        node.send_set_variables(move(-100, 100))

                elif any([x>3700 for x in variables["prox.horizontal"]]):
                    print("obstacle is close")
                    # ON PLACE ROTATION
                    avoid = False
                    #prox = variables["prox.horizontal"]
                    var_prox= np.array(variables["prox.horizontal"])
                    if (var_prox[:1].sum() > var_prox[3:4].sum()):
                        node.send_set_variables(move(0, -100))
                    else:
                        node.send_set_variables(move(-100, 0))

                else :
                    # follow path
                    avoid = True
                    # if position is not in the middle of the path
                    # call get position function and calculate the error
                    # if error is too big, turn
                    print("no obstacle")
                    node.send_set_variables(move(100, 100))
                    

            


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






    