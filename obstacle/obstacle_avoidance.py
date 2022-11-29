from tdmclient import ClientAsync
import time
# This function just goes around the obstacle till sensor distance is less than a threshold from each side (except the back)
# it is a simple implementation, already working on a better one
def move(left, right):
    speed = {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }
    return speed

def close_by(node, variables):
    try:
        
        if any([x>2000 and x < 3500 for x in variables["prox.horizontal"]]):
            print("wall very close") 
            #prox = variables["prox.horizontal"]
            node.send_set_variables(move(30, -30))
        elif any([x>3500 for x in variables["prox.horizontal"]]):
            node.send_set_variables(move(0, 0))
            
            print("wall too close to be avoided by following")

        else :
            node.send_set_variables(move(50, 50))


    except KeyError:
        pass


with ClientAsync() as client:
    async def prog():
        with await client.lock() as node:
            await node.watch(variables=True)
            node.add_variables_changed_listener(close_by)
            # Will sleep forever:
            await client.sleep()
            
            #node.remove_variables_changed_listener(close_by)
    client.run_async_program(prog)
    # close the client once the program is done
    client.close()






    