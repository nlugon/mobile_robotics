from sympy import Segment, Point
import numpy as np
from sympy import Segment, Point, Ray, Polygon
class TrajFollower:
    def __init__(self,dist_threshold,speed_offset=50,kp_ref_angle=0.02,kp_follow_angle = 100):
        self.dist_threshold = dist_threshold
        self.path_step = 0
        self.speed_offset = speed_offset
        self.kp_ref_angle = kp_ref_angle
        self.kp_follow_angle = kp_follow_angle
    
    def get(self,x,y,angle,path):
        traj_seg = Segment(Point(*tuple(path[self.path_step])),Point(*tuple(path[self.path_step+1])))
        traj_ray = Ray(Point(*tuple(path[self.path_step+1])),Point(*tuple(path[self.path_step])))
        robot_dot = Point(x,y)
        robot_ray = Ray(Point(*tuple(path[self.path_step+1])),robot_dot)
        error = traj_seg.distance(robot_dot)
        ref_angle = np.arctan((path[self.path_step][1]-path[self.path_step+1][1])/(path[self.path_step][0]-path[self.path_step+1][0]))
        if robot_ray.closing_angle(traj_ray) < 0:
            error = -error

        correction = self.kp_ref_angle*error
        correction = max(-np.pi/2, correction)
        correction = min(np.pi/2, correction)
        target_angle = ref_angle + correction

        angle_error = target_angle-angle
        speed_diff = self.kp_follow_angle*angle_error

        speed_left = self.speed_offset + speed_diff
        speed_right = self.speed_offset - speed_diff

        return speed_right, speed_left

    def incr_step(self):
        self.path_step += 1

    def reset_step(self):
        self.path_step = 0

    def get_step(self):
        return self.path_step