
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

    def get_goal_position(self,filtered_img):
    
    # RETURNS TUPLE (X,Y) CORRESPONDING TO COORDINATES OF CENTER OF END GOAL POSITION
        filtered_img_hsv = cv2.cvtColor(filtered_img, cv2.COLOR_RGB2HSV) # convert rgb to hsv
        min_blue = np.array([60, 0, 0])
        max_blue = np.array([150, 255, 255])
        mask = cv2.inRange(filtered_img_hsv, min_blue, max_blue) # mask for blue color
        
        output = cv2.bitwise_and(filtered_img, filtered_img, mask=mask)
        
        ret,thresh = cv2.threshold(mask, 40, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # use for geometrical shapes : CHAIN_APPROX_SIMPLE 
        
        if len(contours) != 0:

            # find the biggest countour (c) by the area
            c = max(contours, key = cv2.contourArea)
            M = cv2.moments(c)
            x_center = int(M['m10']/M['m00'])
            y_center = int(M['m01']/M['m00'])
            
        (x,y),radius = cv2.minEnclosingCircle(c)
        center = (int(x),int(y))
        radius = int(radius)
        cv2.circle(output,center,radius,(0,255,0),10) 
        plt.imshow(output)
        plt.show()
            
            
        return (x_center,y_center)
    
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

    def get_thymio_position(self,filtered_img):
    
    # RETURNS CENTER COORDINATES, RADIUS AND ORIENTATION (provides extra coordinants (xb, yb) to know which direction, as well as angle) OF THYMIO ROBOT
    
        filtered_img_hsv = cv2.cvtColor(filtered_img, cv2.COLOR_RGB2HSV) # convert rgb to hsv
        min_red = np.array([0, 125, 200]) # before [0, 0, 220]
        max_red = np.array([80, 255, 255]) # before [30, 255, 255]
        mask = cv2.inRange(filtered_img_hsv, min_red, max_red) # mask for red color
        
        output = cv2.bitwise_and(filtered_img, filtered_img, mask=mask)
        
        ret,thresh = cv2.threshold(mask, 40, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        if len(contours) != 0:

            # find the biggest countour (c) by the area
            c = max(contours, key = cv2.contourArea)
            M = cv2.moments(c)
            x_M = int(M['m10']/M['m00'])
            y_M = int(M['m01']/M['m00'])
            
            (x_circle,y_circle),radius = cv2.minEnclosingCircle(c)
            center_circle = (int(x_circle),int(y_circle))
            radius = int(radius)
            cv2.circle(output,center_circle,radius,(0,255,0),10) 
        
        
            # cv.minAreaRect returns: (center(x, y), (width, height), angle of rotation) 
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            center_rect = (int(rect[0][0]),int(rect[0][1])) 
            width = int(rect[1][0])
            height = int(rect[1][1])
            angle = int(rect[2])

            cv2.drawContours(output,[box],0,(255,0,0),10)
            
            
            #pca_center, pca_p1, pca_p2, pca_angle = getOrientation(c,output) # not very accurate for now
            angle2 = math.degrees(atan2((center_circle[1]-center_rect[1]),(center_circle[0]-center_rect[0])))
            
            endx1, endy1, endx2, endy2 = self.get_coords(center_circle[0], center_circle[1], angle2, filtered_img.shape[1], filtered_img.shape[0])
            
            cv2.line(output,(int(endx1),int(endy1)),(int(endx2),int(endy2)),(200,0,225),15)
            plt.imshow(output)
            plt.show()
            
            return (center_circle, radius, center_rect, angle2)
        else:
            #print("No red object found")
            return 0,0,0,0

    def get_obstacle_position(self,filtered_img, thymio_radius, min_green = np.array([30, 87, 0]), max_green = np.array([90, 255, 255])):
    
        # RETURNS LIST OF VERTICES AND NUMBER OF OBSTACLES
        
        filtered_img_hsv = cv2.cvtColor(filtered_img, cv2.COLOR_RGB2HSV) # convert rgb to hsv
        mask = cv2.inRange(filtered_img_hsv, min_green, max_green) # mask for green color
        mask = cv2.medianBlur(mask,15) # Remove noise in the mask
        ret,thresh = cv2.threshold(mask, 40, 255, 0) # Could remove this
        
        output = cv2.bitwise_and(filtered_img, filtered_img, mask=mask)
        
        
        dilatation_size = int(thymio_radius)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2 * dilatation_size + 1,2 * dilatation_size + 1))
        thresh_dilated = cv2.dilate(thresh, kernel, iterations = 1)
        
        contours, hierarchy = cv2.findContours(thresh_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # use for geometrical shapes : CHAIN_APPROX_SIMPLE 
        
        if len(contours) != 0:
            
            nb_obstacles = 0
            vertices = []
            

            for c in contours:
                area = cv2.contourArea(c)

                if area > 1000:
                    nb_obstacles += 1

                    epsilon = 0.015*cv2.arcLength(c,True)
                    approx = cv2.approxPolyDP(c,epsilon,True)
                    cv2.drawContours(output, [approx], -1, (0, 255, 255), 20)
                    print(approx)
                    vertices.append(approx)                 
                    
                    
                    # Plotting for troubleshooting

                    plt.figure(figsize=(9,3))

                    plt.subplot(131)
                    plt.imshow(mask, cmap=plt.cm.gray)
                    plt.axis('off')
                    plt.subplot(132)
                    plt.imshow(thresh_dilated)
                    plt.axis('off')
                    plt.subplot(133)
                    plt.imshow(output)
                    plt.axis('off')

                    plt.subplots_adjust(wspace=0.02, hspace=0.02, top=1, bottom=0, left=0, right=1)
                    plt.show()
        
    
        plt.imshow(output)                  
        return (vertices, nb_obstacles)


if __name__ == '__main__':
    vis_obj = Vision()
    # put your image path here
    img = cv2.imread('src/sample_image.png')
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    filtered_img = cv2.bilateralFilter(img,9,75,75)
    center_circle, radius, center_rect, angle = vis_obj.get_thymio_position(filtered_img)
    vertices, nb_obstacles = vis_obj.get_obstacle_position(filtered_img, radius)
    goal_position = vis_obj.get_goal_position(filtered_img)
    print(center_circle, radius, center_rect, angle)
    print(vertices, nb_obstacles)


