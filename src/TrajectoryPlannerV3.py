import matplotlib.pyplot as plt
from sympy import Point, Polygon, Line, Segment
import random
import numpy as np
import time

class Node:
    def __init__(self,position,id,type="node",neighbours=[],obstacle_id=None):
        #properties
        self.position = position #[x,y]
        self.id = id #if = 0 it's the start node
        self.type = type #types: "node", "start", "end"
        
        self.point2d = Point(*tuple(position))
        self.obstacle_id = obstacle_id

        #connections
        self.access_cost = 0
        self.access_node = 0
        self.visibility_list = neighbours[:]

    def print(self):
        print("== Node {0} : Position:{1} - Type:{2} - Cost:{4} with node {5} - Visibility:{3} - Obstacle: {6}".format(self.id,self.position,self.type,self.visibility_list,self.access_cost,self.access_node,self.obstacle_id))
    
    def plot(self,color="green"):
        plt.scatter(*tuple(self.position),color=color)

#the terrain contains all the groups, each group corresponds to an obstacle (exept the first one which contains the start and end nodes)
class Planner:
    def __init__(self):
        self.list_nodes = []
        self.list_obstacles = [] #list of sympy.Polygon that defines the obstacles

        self.start_node_id = None
        self.end_node_id = None
    
    def add_node(self,position,type="node",neighbours=[],obstacle_id=None):
        self.list_nodes.append(Node(position,len(self.list_nodes),type,neighbours,obstacle_id))

    def set_start(self,position):
        if self.start_node_id == None:
            self.start_node_id = len(self.list_nodes)
            self.add_node(position,"start",[])
        else:
            self.list_nodes[self.start_node_id] = Node(position,id=self.start_node_id,type="start",neighbours=[],obstacle_id=None)

    def set_end(self,position):
        if self.end_node_id == None:
            self.end_node_id = len(self.list_nodes)
            self.add_node(position,"end",[])
        else:
            self.list_nodes[self.end_node_id] = Node(position,id=self.end_node_id,type="end",neighbours=[],obstacle_id=None)

    #add nodes and obstacle polygon from vertices
    def add_obstacle(self,list_vertices):
        current_obstacle_id = len(self.list_obstacles)
        first_id = len(self.list_nodes)
        last_id = len(self.list_nodes) + len(list_vertices)-1
        #add first node of obstacle
        self.add_node(list_vertices[0][0],neighbours=[last_id,first_id+1],obstacle_id=current_obstacle_id)
        for i in range(1,len(list_vertices)-1):
            self.add_node(list_vertices[i][0],neighbours=[len(self.list_nodes)-1,len(self.list_nodes)+1],obstacle_id=current_obstacle_id)
        #add last node of obstacle
        self.add_node(list_vertices[-1][0],neighbours=[last_id-1,first_id],obstacle_id=current_obstacle_id)
        #add the obstacle polygon
        self.list_obstacles.append(Polygon(*tuple([self.list_nodes[i].point2d for i in range(first_id,last_id+1)])))

    def update_visibility_node(self,node_id,is_vertices=False):
        self.list_nodes[node_id].visibility_list = []
        for node_b in self.list_nodes:
            if node_b.id != node_id:
                visibility_line = Segment(self.list_nodes[node_id].point2d,node_b.point2d)
                visible = True
                for obs_id in range(len(self.list_obstacles)):
                    n_inter = len(self.list_obstacles[obs_id].intersection(visibility_line))
                    if n_inter > 1:
                        visible = False
                        break
                if visible:
                    self.list_nodes[node_id].visibility_list.append(node_b.id)
                    if not (node_id in node_b.visibility_list):
                        node_b.visibility_list.append(node_id)
                else:
                    if node_id in node_b.visibility_list:
                        node_b.visibility_list.remove(node_id)

    def update_visibility_all(self,first_time=True):
        if not first_time:
            for node in self.list_nodes:
                node.visibility_list = []
        for node_a in self.list_nodes:
            for node_b in self.list_nodes[node_a.id+1:]:
                if not node_b.id in node_a.visibility_list:
                    visibility_line = Segment(node_a.point2d,node_b.point2d)
                    visible = True
                    for obs_id in range(len(self.list_obstacles)):
                        inter = self.list_obstacles[obs_id].intersection(visibility_line)
                        if not first_time:
                            inter = [obj for obj in inter if type(obj) != Segment] #allows points of same side to see each other
                        n_inter = len(inter)
                        test = str(self.list_obstacles[obs_id].intersection(visibility_line))
                        if node_a.obstacle_id == node_b.obstacle_id and node_a.obstacle_id == obs_id:
                            if n_inter > 2:
                                visible = False
                                break
                            elif self.list_obstacles[obs_id].encloses_point(visibility_line.midpoint):
                                visible = False
                                break
                        elif n_inter > 1:
                            visible = False
                            break
                    if visible:
                        node_a.visibility_list.append(node_b.id)
                        node_b.visibility_list.append(node_a.id)

    def update_cost(self):
        time_start = time.time()
        current_nodes_id = [self.start_node_id]
        timeout = 0
        while (len(current_nodes_id) > 0) and (timeout < 100):
            timeout += 1
            next_nodes_id = []
            for id_base in current_nodes_id:
                for id_target in self.list_nodes[id_base].visibility_list:
                    cost = round(float(self.list_nodes[id_base].point2d.distance(self.list_nodes[id_target].point2d) + self.list_nodes[id_base].access_cost),2)
                    if (self.list_nodes[id_target].access_cost > cost) or ((self.list_nodes[id_target].access_cost == 0) and (self.list_nodes[id_target].type !="start")):
                        self.list_nodes[id_target].access_cost = cost
                        self.list_nodes[id_target].access_node = id_base
                        next_nodes_id.append(id_target)
            current_nodes_id = list(set(next_nodes_id[:])) #remove duplicate ids
        print("Time compute cost: {0}".format(time.time()-time_start))

    def get_path(self):
        path = [self.list_nodes[self.end_node_id].position]
        current_id = self.end_node_id
        while self.list_nodes[current_id].access_node != self.start_node_id:
            next_node_id = self.list_nodes[current_id].access_node
            path.append(self.list_nodes[next_node_id].position)
            current_id = next_node_id
        path.append(self.list_nodes[self.start_node_id].position)
        path.reverse()
        return path

    #adds the nodes of obstacles, adds polygons, computes visibility for each node
    def setup(self,obstacle_vertices,start_pos=None,end_pos=None):
        for _obstacle in obstacle_vertices:
            print("NEW OBSTACLE : " + str(_obstacle))
            self.add_obstacle(_obstacle)
        if start_pos != None:
            self.set_start(start_pos)
        if end_pos != None:
            self.set_end(end_pos)
        self.update_visibility_all()
        self.update_cost()
        

    #===== DEBUG =====
    def print(self):
        counter = 0
        for _node in self.list_nodes:
            _node.print()
            counter += 1
    
    def plot_path(self):
        path = self.get_path()
        x_axis = []
        y_axis = []
        for point in path:
            x_axis.append(point[0])
            y_axis.append(point[1])
        plt.plot(x_axis,y_axis)

    def plot_node(self,node_id):
        self.list_nodes[node_id].plot()

    def plot_polygons(self):
        for polygon in self.list_obstacles:
            x_axis = []
            y_axis = []
            for point in polygon.vertices:
                x_axis.append(point.x)
                y_axis.append(point.y)
            x_axis.append(x_axis[0])
            y_axis.append(y_axis[0])
            plt.plot(x_axis,y_axis)

    def plot_visibility(self,node_id):
        self.list_nodes[node_id].plot("red")
        for id in self.list_nodes[node_id].visibility_list:
            self.list_nodes[id].plot()
            plt.plot([self.list_nodes[node_id].position[0],self.list_nodes[id].position[0]],[self.list_nodes[node_id].position[1],self.list_nodes[id].position[1]],color="orange")


def random_polygon(Np_min,Np_max,r_min,r_max,center=[0,0]): #[DUMMY]
    radius = random.randint(r_min,r_max)
    point_list = [[[center[0]+radius,center[1]]]]
    Np = 1
    radius = random.randint(r_min,r_max)
    angle = random.randint(int(360/Np_max), int(360/Np_min))
    while angle < 360:
        point_list.append([[center[0]+int(radius*np.cos(np.deg2rad(angle))), center[1]+int(radius*np.sin(np.deg2rad(angle)))]])
        radius = random.randint(r_min,r_max)
        angle += random.randint(int(360/Np_max), int(360/Np_min))
    return point_list

if __name__ == "__main__":
    #========== COMPUTER VISION OUTPUT
    #cv_output = [[[1,1],[0,4],[4,4],[4,0]],[[5,5],[0,10],[12,10]]]
    #cv_output = [random_polygon(4,8,20,40,[25,25]),random_polygon(4,8,10,40,[50,80])]
    #cv_output = [[[[0,0]],[[0,1]],[[1,1]],[[1,0]]],[[[2,2]],[[2,4]],[[4,4]],[[4,2]]]]
    cv_output = [[[[ 248,  848]],[[238, 1079]],[[ 765, 1075]],[[ 756,  814]],[[ 377,  805]]],
                [[[1144,  810]],[[1146, 1079]],[[1674, 1079]],[[1661,  826]],[[1517,  796]]],
                [[[ 581,    0]],[[ 583,  377]],[[ 710,  659]],[[1176,  680]],[[1193,  228]],[[1053,  205]],[[1049,    0]]]]
    cv_start = [0,1200]
    cv_end = [1300,400]

    #========== SETUP
    planner = Planner()
    planner.setup(cv_output,cv_start,cv_end)
    planner.print()
    planner.plot_polygons()
    planner.plot_node(planner.start_node_id)
    planner.plot_node(planner.end_node_id)
    planner.plot_visibility(planner.start_node_id)
    print("OPTIMAL PATH: " + str(planner.get_path()))
    planner.plot_path()
    plt.show()

    start_time = time.time()
    planner.set_start([400,600])
    planner.update_visibility_node(planner.start_node_id)
    planner.update_cost()
    print("TIME FOR REPLANING: {0}s".format(time.time()-start_time))
    planner.plot_polygons()
    planner.plot_visibility(planner.start_node_id)
    planner.plot_node(planner.end_node_id)
    planner.plot_path()
    plt.show()