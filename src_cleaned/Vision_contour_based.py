
import cv2
import numpy as np
from ipywidgets import *
import matplotlib.pyplot as plt
import math
from math import atan2, cos, sin, sqrt, pi

class Vision:
    def __init__(self):
        self.cap = None
        self.frame_count = 0
        self.ret = False
        self.frame = None
        self.obstacle_vertices = None
        self.initial_thymio_pos = None
        self.goal_pos = None
        self.obj_terrain = None

    def get_goal_position(self,img, min_blue = np.array([70, 80, 0]), max_blue = np.array([120, 255, 255])):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_hsv_blur = cv2.medianBlur(img_hsv, 15)
        mask = cv2.inRange(img_hsv_blur, min_blue, max_blue)
        mask = cv2.erode(mask, None, iterations=2) # 3x3 kernel used
        mask = cv2.dilate(mask, None, iterations=2)
        
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] # RETR_EXTERNAL to get external contour, CHAIN_APPROX_SIMPLE to get geometrical shape 
        output = cv2.bitwise_and(img, img, mask=mask)
        
        x_center = 0
        y_center = 0
        goal_detected = False
        if len(contours) != 0:
            
            # find the biggest countour (c) by the area
            c = max(contours, key = cv2.contourArea)
            M = cv2.moments(c)
            x_center = int(M['m10']/M['m00'])
            y_center = int(M['m01']/M['m00'])
            if cv2.contourArea(c) > 5000:
                goal_detected = True
                (x,y),radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y))
                radius = int(radius)
                cv2.circle(output,center,radius,(0,255,0),10) 
                
        plt.imshow(output)
            
            
        return x_center,y_center, goal_detected
    
    def get_coords(self,x, y, angle, imwidth, imheight):
        x1_length = (x-imwidth) / math.cos(angle)
        y1_length = (y-imheight) / math.sin(angle)
        length = max(abs(x1_length), abs(y1_length))
        endx1 = x + length * math.cos(math.radians(angle))
        endy1 = y + length * math.sin(math.radians(angle))

        x2_length = (x-imwidth) / math.cos(angle+180)
        y2_length = (y-imheight) / math.sin(angle+180)
        length = max(abs(x2_length), abs(y2_length))
        endx2 = x + length * math.cos(math.radians(angle+180))
        endy2 = y + length * math.sin(math.radians(angle+180))

        return endx1, endy1, endx2, endy2

    def get_thymio_position(self,img, kalman_center, kalman_angle, min_red = np.array([150, 0, 180]), max_red = np.array([179, 255, 255])):
    
    # RETURNS CENTER COORDINATES, RADIUS AND ORIENTATION (provides extra coordinants (xb, yb) to know which direction, as well as angle) OF THYMIO ROBOT

    
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_hsv_blur = cv2.medianBlur(img_hsv, 15)
        mask = cv2.inRange(img_hsv_blur, min_red, max_red)
        mask = cv2.erode(mask, None, iterations=2) # 3x3 kernel used
        mask = cv2.dilate(mask, None, iterations=2)    
        output = cv2.bitwise_and(img, img, mask=mask)
        
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        #print(contours)
        #print(hierarchy)
        #cv2.drawContours(output ,[contours[0]],-1,(0,255,0),5)
        #cv2.drawContours(output ,[contours[1]],-1,(255,255,0),10)
        #cv2.drawContours(output ,[contours[2]],-1,(0,255,255),5)
        #plt.imshow(output)
        
        
        
        
        thymio_detected = False
        center = kalman_center
        radius = 0
        angle = kalman_angle
        
        if len(contours) != 0:

            # find the biggest countour c by the area
            c = max(contours, key = cv2.contourArea)
            if cv2.contourArea(c) > 5000:
                thymio_detected = True
                print(cv2.contourArea(c))
                        

                # Minimum Enclosing Circle :
                (x_circle,y_circle),radius = cv2.minEnclosingCircle(c)
                center = (int(x_circle),int(y_circle))
                radius = int(radius)
                cv2.circle(output,center,radius,(0,255,0),10) 

                for contour in contours:
                    area = cv2.contourArea(contour)
                    #if area > 2400 and area < 3200:
                    if area > cv2.contourArea(c)/10 and area < cv2.contourArea(c)/10+2000:
                        print(cv2.contourArea(contour))
                        cv2.drawContours(output ,[contour],-1,(255,255,0),10)
                        (x_front,y_front),radius_front = cv2.minEnclosingCircle(contour)
                        center_front = (int(x_front),int(y_front))
                        radius_front = int(radius_front)
                        cv2.circle(output,center_front,radius_front,(255,255,0),10)
                        
                        angle = int(math.degrees(math.atan2(center_front[1]-center[1],center_front[0]-center[0]))) 
            
                
        plt.imshow(output)
            
        return center, radius, angle, thymio_detected


    def get_obstacle_position(self,img, thymio_radius, min_green = np.array([30, 87, 0]), max_green = np.array([90, 255, 255])):

        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_hsv_blur = cv2.medianBlur(img_hsv, 15)
        mask = cv2.inRange(img_hsv_blur, min_green, max_green)
        mask = cv2.erode(mask, None, iterations=2) # 3x3 kernel used
        mask = cv2.dilate(mask, None, iterations=2)
        
        y_max, x_max = mask.shape

        # Expand obstacle size by the radius of the thymio
        dilatation_size = int(thymio_radius*1.2) # add security margin of 20% of thymio's radius
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2 * dilatation_size + 1,2 * dilatation_size + 1))
        mask_dilated = cv2.dilate(mask, kernel, iterations = 1)

        contours = cv2.findContours(mask_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] # RETR_EXTERNAL to get external contour, CHAIN_APPROX_SIMPLE to get geometrical shape 

        output = cv2.bitwise_and(img, img, mask=mask)
        nb_obstacles = 0
        vertices = []
        if len(contours) != 0:
            contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
            for c in contours:
                area = cv2.contourArea(c)
                #print(area)
                if area > 10000:
                    nb_obstacles += 1
                    epsilon = 0.015*cv2.arcLength(c,True)
                    approx = cv2.approxPolyDP(c,epsilon,True)
                    cv2.drawContours(output, [approx], -1, (0, 255, 255), 20)
                    vertices.append(approx)
                else: 
                    break

        plt.imshow(output)
        
        return (vertices, nb_obstacles, x_max, y_max)