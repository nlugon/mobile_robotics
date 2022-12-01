from sympy import Segment, Point
import numpy as np

traj = [[0,0],[1,2],[2,2]]

x = 5
y = 5
traj_part = 0
kp = 0.05

def controller(x,y,traj,traj_part):
    if Point(x,y).distance(Point(traj[traj_part+1][0],traj[traj_part+1][1])) < 10: #threshold
        traj_part += 1
    position = Point(x,y)
    horz = Segment(Point(0,0),Point(1,0))
    thymio_pos = Segment(Point(0,0),Point(x,y))
    traj_segment = Segment(Point(traj[traj_part][0],traj[traj_part][1]),Point(traj[traj_part+1][0],traj[traj_part+1][1]))

    error = traj_segment.distance(position)

    #check on which side we are
    correction = error*kp
    if correction > np.pi/2:
        correction = np.pi/2

    traj_angle = traj_segment.angle_between(horz)

    if traj_angle > thymio_pos.angle_between(horz):
        new_angle = error*kp + traj_angle
    else:
        new_angle = error*kp - traj_angle

    return new_angle