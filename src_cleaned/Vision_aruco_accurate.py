import cv2
from cv2 import aruco
import numpy as np
import math


def dist(pt1, pt2):
    ''' Returns the distance between 2 given points expressed as a list of len 2 '''
    distance = ( (pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2 )**0.5
    return distance

def get_thymio_position(img, img_output):
    
    '''
    Function that outputs Thymio position, radius and orientation from camera frames using Aruco marker detection
    Parts of code taken from https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/aruco_basics.html

    Inputs: - img : image from camera in BGR format
            - img_output : copy of image img on which will be drawn Thymio's contours (for visualization)

    Output: - center : tuple (x,y) representing the center of the Thymio (in pixels)
            - radius : radius of the Thymio (in pixels)
            - angle : orientation of the Thymio (in radians)
            - thymio_detected : boolean set to True if Thymio was detected, False if not
    '''
    
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(img_output, corners, ids)
    
    
    thymio_detected = False
    center = (0,0)
    radius = 0
    angle = 0
    
    if np.all(ids != None):
        for i in range(len(ids)):
            if ids[i] == 0:
                thymio_detected = True
                c = corners[i][0]

                top_center = (c[0:2, 0].mean(),c[0:2, 1].mean())
                bottom_center = (c[2:4, 0].mean(), c[2:4, 1].mean())
                left_center = (c[0:4:3, 0].mean(), c[0:4:3, 1].mean())
                right_center = (c[1:3, 0].mean(), c[1:3, 1].mean())
                points = np.array([ [c[2,0],c[2,1]], [c[3,0],c[3,1]], [left_center[0],left_center[1]], [right_center[0],right_center[1]] ])

                square_center = (int(c[:, 0].mean()), int(c[:, 1].mean()))
                radius = int(max( dist(square_center,c[0]), dist(square_center,c[1]), dist(square_center,c[2]), dist(square_center,c[3])))
                angle = math.atan2(top_center[1]-bottom_center[1],top_center[0]-bottom_center[0])
                
                
                center = (int(points[:, 0].mean()), int(points[:, 1].mean()))
                cv2.arrowedLine(img_output, (center[0],center[1]), (int(top_center[0]),int(top_center[1])), (255,255,0), 10)
                cv2.circle(img_output,square_center,radius,(255,0,0),10)

                break

        
    return center, radius, angle, thymio_detected


def get_obstacle_position(img, thymio_radius, min_green = np.array([30, 80, 70]), max_green = np.array([70, 255, 150])):
    '''
    Function that outputs vertices of the obstacles (dilated by the Thymio's radius) for the visibility graph path planning
        Parts of code taken from :
        https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html
        https://realpython.com/python-opencv-color-spaces/
        https://notebook.community/ricklon/opencvraspberrypi/notebook/openCV%20color%20detection
        
    Inputs: - img : image from camera in BGR format
            - thymio_radius : radius of Thymio in pixels (can be found with the get_thymio_position function)
            - min_green : minimum values for Hue, Saturation and Value for hsv mask
            - max_green : minimum values for Hue, Saturation and Value for hsv mask

    Output: - vertices : array (1 x nb_obstacles x 2) containing list of each obstacle's vertices
            - nb_obstacles : number of obstacles detected (after the dilatation)
            - x_max : length of the image img
            - y_max : width of the image img
    '''    

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
            if area > 1000:
                nb_obstacles += 1
                epsilon = 0.015*cv2.arcLength(c,True)
                approx = cv2.approxPolyDP(c,epsilon,True)
                cv2.drawContours(output, [approx], -1, (0, 255, 255), 20)
                vertices.append(approx)
            else: 
                break

    #plt.imshow(output)

    return vertices, nb_obstacles, x_max, y_max


def get_goal_position(img):
    '''
    Function that outputs goal position from initial camera frame
    Parts of code taken from https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/aruco_basics.html


    Inputs: - img : image from camera in BGR format

    Output: - goal_center : tuple (x,y) representing the center of the goal (in pixels)
            - goal_detected : boolean set to True if goal was detected, False if not
    '''
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    
    goal_detected = False
    goal_center = (0,0)
    goal_radius = 0
    
    if np.all(ids != None):
        for i in range(len(ids)):
            if ids[i] == 1:
                goal_detected = True
                c = corners[i][0]
                goal_center = (int(c[:, 0].mean()), int(c[:, 1].mean()))
                goal_radius = int(max( dist(goal_center,c[0]), dist(goal_center,c[1]), dist(goal_center,c[2]), dist(goal_center,c[3])))
                
                break
          
        
    return goal_center, goal_radius, goal_detected