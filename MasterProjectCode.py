# MASTER CODE FOR MOBILE ROBOTICS PROJECT

#Importing Libraries
import cv2
import cv2 as cv
from matplotlib import pyplot as plt
import math
from math import atan2, cos, sin, sqrt, pi
import numpy as np



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

def PD(Kp, Kd, MV_bar=0):
    #initialize stored data 
    e_prev = 0
    t_prev = -100
    I = 0

    #initial control 
    MV = MV_bar

    while True:
        #yield MV, wait for new t, RV, SP
        t, PV, SP = yield MV

        #PID Calculations
        e = SP - PV

        P = Kp*e
        #I = I + Ki*e*(t-t_prev)
        D = Kd*(e-e_prev)/(t-t_prev)

        MV = MV_bar + P + I + D

        e_prev = e
        t_prev = t

class Node:
    def __init__(self,position,id,type="node",neighbours=[]):
        #properties
        self.position = position #[x,y]
        self.id = id #if = 0 it's the start node
        self.type = type #types: "node", "start", "end"
        self.point2d = Point(*tuple(position))

        #connections
        self.access_cost = 0
        self.access_node = 0
        self.visibility_list = neighbours #contains the

    def print(self):
        print("== Node {0} : Position:{1} - Type:{2} - Cost:{4} with node {5} - Visibility:{3}".format(self.id,self.position,self.type,self.visibility_list,self.access_cost,self.access_node))
    
    def plot(self,color="green"):
        plt.scatter(*tuple(self.position),color=color)

# START OF THE PATH PLANNING FUNCTIONS

#the terrain contains all the groups, each group corresponds to an obstacle (exept the first one which contains the start and end nodes)
class Terrain:
    def __init__(self):
        self.list_nodes = []
        self.list_obstacles = [] #list of sympy.Polygon that defines the obstacles
        self.size_nodes = 0

    def add_node(self,position,type="node",neighbours=[]):
        self.list_nodes.append(Node(position,self.size_nodes,type,neighbours))
        self.size_nodes += 1

    def add_obstacle(self,list_vertices):
        first_id = self.size_nodes
        last_id = self.size_nodes + len(list_vertices)-1
        #add first node of obstacle
        self.add_node(list_vertices[0],neighbours=[last_id,first_id+1])
        for i in range(1,len(list_vertices)-1):
            self.add_node(list_vertices[i],neighbours=[self.size_nodes-1,self.size_nodes+1])
        #add last node of obstacle
        self.add_node(list_vertices[-1],neighbours=[last_id-1,first_id])
        #add the obstacle polygon
        self.list_obstacles.append(Polygon(*tuple([self.list_nodes[i].point2d for i in range(first_id,last_id+1)])))
            
    def print(self):
        counter = 0
        for _node in self.list_nodes:
            _node.print()
            #print("Real Node: " + str(counter))
            counter += 1

    def plot_visibility(self,node_id):
        self.list_nodes[node_id].plot("red")
        for id in self.list_nodes[node_id].visibility_list:
            self.list_nodes[id].plot()
            plt.plot([self.list_nodes[node_id].position[0],self.list_nodes[id].position[0]],[self.list_nodes[node_id].position[1],self.list_nodes[id].position[1]],color="orange")

    def plot_path(self,end_node_id):
        current_id = end_node_id
        while current_id > 0:
            next_id = self.list_nodes[current_id].access_node
            plot_line(self.list_nodes[current_id].position,self.list_nodes[next_id].position)
            current_id = next_id

    def complete_visibility(self):
        for node_a_id in range(self.size_nodes):
            for node_b_id in range(node_a_id+1,self.size_nodes):
                visibility_line = Segment(self.list_nodes[node_a_id].point2d,self.list_nodes[node_b_id].point2d)
                visible = True
                for _obstacle in self.list_obstacles:
                    if len(_obstacle.intersection(visibility_line)) > 1:
                        visible = False
                        break
                if visible:
                    self.list_nodes[node_a_id].visibility_list.append(node_b_id)
                    self.list_nodes[node_b_id].visibility_list.append(node_a_id)

    def node_distance(self,node_a_id,node_b_id):
        return np.sqrt((self.list_nodes[node_a_id].position[0]-self.list_nodes[node_b_id].position[0])**2 + (self.list_nodes[node_a_id].position[1]-self.list_nodes[node_b_id].position[1])**2)

    def complete_cost(self):
        current_nodes_id = [0]
        timeout = 0
        while (len(current_nodes_id) > 0) and (timeout < 20):
            timeout += 1
            #print("New cycle")
            #print(current_nodes_id)
            #self.print()
            next_nodes_id = []
            for id_base in current_nodes_id:
                for id_target in self.list_nodes[id_base].visibility_list:
                    cost = round(self.node_distance(id_base,id_target) + self.list_nodes[id_base].access_cost,2)
                    if (self.list_nodes[id_target].access_cost > cost) or ((self.list_nodes[id_target].access_cost == 0) and (self.list_nodes[id_target].type !="start")):
                        self.list_nodes[id_target].access_cost = cost
                        self.list_nodes[id_target].access_node = id_base
                        #print("NODE {0} - Update Node {1} to cost {2}".format(id_base,id_target,cost))
                        next_nodes_id.append(id_target)
            current_nodes_id = list(set(next_nodes_id[:]))
            #print(len(current_nodes_id))




def plot_line(c1,c2):
    plt.plot([c1[0],c2[0]],[c1[1],c2[1]],color="purple")

def random_polygon(Np_min,Np_max,r_min,r_max,center=[0,0]): #[DUMMY]
    radius = random.randint(r_min,r_max)
    point_list = [[center[0]+radius,center[1]]]
    Np = 1
    radius = random.randint(r_min,r_max)
    angle = random.randint(int(360/Np_max), int(360/Np_min))
    while angle < 360:
        point_list.append([center[0]+int(radius*np.cos(np.deg2rad(angle))), center[1]+int(radius*np.sin(np.deg2rad(angle)))])
        radius = random.randint(r_min,r_max)
        angle += random.randint(int(360/Np_max), int(360/Np_min))
    return point_list

#BEGINING OF LIVE THYMIO CORRECTION AND NAVIGATION PROCEDURE
count = 0

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
    thymio_x = thymioLoc(1)
    thymio_y = thymioLoc(2)
    P = [thymio_x,thymio_y,thymioAngle]
    SP = point with minimal distance on the line from the thymio location

    controller = PD(2, 0.1, 2)        # create pid control
    controller.send(None)              # initialize






    c = cv2.waitKey(1)
    count = count +1
    print(count)
    print('Center of Circle:',thymioLoc, 'Center of Rectangle:',RectLoc, 'Angle:' ,thymioAngle)
    if c == 27:
        break

cap.release()
cv2.destroyAllWindows()