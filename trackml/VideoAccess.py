import cv2
import cv2 as cv
from matplotlib import pyplot as plt
import math
from math import atan2, cos, sin, sqrt, pi
import numpy as np

global center_circle, radius, center_rect, angle2

cap = cv2.VideoCapture(0)
count = 0

def get_coords(x, y, angle, imwidth, imheight):

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

def get_thymio_position(filtered_img):
    
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
        M = cv.moments(c)
        x_M = int(M['m10']/M['m00'])
        y_M = int(M['m01']/M['m00'])
        
        (x_circle,y_circle),radius = cv.minEnclosingCircle(c)
        center_circle = (int(x_circle),int(y_circle))
        radius = int(radius)
        cv.circle(output,center_circle,radius,(0,255,0),10) 
        # cv.minAreaRect returns: (center(x, y), (width, height), angle of rotation) 
        rect = cv.minAreaRect(c)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        center_rect = (int(rect[0][0]),int(rect[0][1])) 
        width = int(rect[1][0])
        height = int(rect[1][1])
        angle = int(rect[2])

        cv.drawContours(output,[box],0,(255,0,0),10)
        
        
        #pca_center, pca_p1, pca_p2, pca_angle = getOrientation(c,output) # not very accurate for now
        angle2 = math.degrees(atan2((center_circle[1]-center_rect[1]),(center_circle[0]-center_rect[0])))
        
        endx1, endy1, endx2, endy2 = get_coords(center_circle[0], center_circle[1], angle2, filtered_img.shape[1], filtered_img.shape[0])
        
        cv.line(output,(int(endx1),int(endy1)),(int(endx2),int(endy2)),(200,0,225),15)
        plt.imshow(output)

    return (center_circle, radius, center_rect, angle2)


# Check if the webcam is opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")

while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, None, fx=1, fy=1, interpolation=cv2.INTER_AREA)
    cv2.imshow('Input', frame)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_blur = cv2.bilateralFilter(frame_rgb,9,75,75)
    thymioLoc, thymioRadius, RectLoc, thymioAngle = get_thymio_position(frame_blur)

    c = cv2.waitKey(1)
    count = count +1
    print(count)
    print('Center of Circle:',thymioLoc, 'Center of Rectangle:',RectLoc, 'Angle:' ,thymioAngle)
    if c == 27:
        break

cap.release()
cv2.destroyAllWindows()