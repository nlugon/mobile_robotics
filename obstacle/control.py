from tdmclient import ClientAsync, aw
import math




def motors(l_speed=500, r_speed=500, verbose=False):
    return {
        "motor.left.target": [l_speed],
        "motor.right.target": [r_speed],
    }

def get_prox(node, verbose=False):
    return node['prox.horizontal']

def get_fake_pos(node, verbose=False):
    return 

def get_vertices(verbose=False):
    return [
        [0,0],
        [0,1],
        [1,1],
        [1,0]
    ]

    

            


def global_adjustment_angle(thymio_xy, curr_ang):
    
    # vert is a 2d array of the vertice
    # thymio_xy is the current position of the thymio
  
    # get the angle of the line
    vert = get_vertices()
    # angle of the line to the x axis
    angle = math.atan2(vert[1][1]-vert[0][1], vert[1][0]-vert[0][0])

    # error is the difference between the current angle and the angle of the line

    angle_difference = angle - curr_ang
    error = range(-0.1,0.1)
    while angle_difference not in range(-0.1,0.1):
        if angle_difference > 0:
            # turn left
            return motors(-100,100)
        else:
            # turn right
            return motors(100,-100)
           
    return motors(0,0)


def global_adjustment_distance(thymio_xy, goal):
  
    vert = get_vertices()
    # get 2D array 
    while thymio_xy not in vert or thymio_xy != goal:
        # get the distance between the thymio and the goal
        distance = math.sqrt((thymio_xy[0]-goal[0])**2 + (thymio_xy[1]-goal[1])**2)
        if distance > 0.1:
            # move forward
            return motors(100,100)
        # once reached the goal, stop
        return motors(0,0)

def simulate():
    # simulate the thymio
    # get the current position of the thymio
    thymio_xy = get_fake_pos()
    # get the current angle of the thymio
    curr_ang = 0
    # get the goal
    goal = [1,1]
    # get the vertices
    vert = get_vertices()
    # get the prox
    prox = get_prox()
    # get the angle of the line
    angle = math.atan2(vert[1][1]-vert[0][1], vert[1][0]-vert[0][0])
    # error is the difference between the current angle and the angle of the line
    angle_difference = angle - curr_ang
    acceptable_error = range(-0.1,0.1)
    while angle_difference not in acceptable_error:
        if angle_difference > 0:
            # turn left
            print("turning left")
        else:
            # turn right
            print("turning right")
           
    # get the distance between the thymio and the goal
    distance = math.sqrt((thymio_xy[0]-goal[0])**2 + (thymio_xy[1]-goal[1])**2)
    while distance > 0.1:
        # move forward
        print("moving forward")
    # once reached the goal, stop
    print("goal reached")









