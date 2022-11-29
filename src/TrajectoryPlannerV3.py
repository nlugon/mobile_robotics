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
        self.visibility_list = neighbours

    def print(self):
        print("== Node {0} : Position:{1} - Type:{2} - Cost:{4} with node {5} - Visibility:{3} - Obstacle: {6}".format(self.id,self.position,self.type,self.visibility_list,self.access_cost,self.access_node,self.obstacle_id))
    
    def plot(self,color="green"):
        plt.scatter(*tuple(self.position),color=color)

#the terrain contains all the groups, each group corresponds to an obstacle (exept the first one which contains the start and end nodes)
class Planner:
    def __init__(self):
        self.list_nodes = []
        self.start_node_id = None
        self.end_node_id = None
        self.list_obstacles = [] #list of sympy.Polygon that defines the obstacles
        self.size_nodes = 0

    #adds the nodes of obstacles, adds polygons, computes visibility for each node
    def setup(self,obstacle_vertices):
        for _obstacle in obstacle_vertices:
            self.add_obstacle(_obstacle)
        self.compute_visibility_all()

    def add_node(self,position,type="node",neighbours=[],obstacle_id=None):
        self.list_nodes.append(Node(position,self.size_nodes,type,neighbours,obstacle_id))
        self.size_nodes += 1

    def set_start_node(self,position):
        if self.start_node_id == None:
            self.start_node_id = len(self.list_nodes)
            self.add_node(position,type="start",neighbours=[],obstacle_id=None)
        else:
            self.list_nodes[self.start_node_id] = Node(self,position,self.start_node_id,type="start",neighbours=[],obstacle_id=None)

    def set_end_node(self,position):
        if self.end_node_id == None:
            self.end_node_id = len(self.list_nodes)
            self.add_node(position,type="end",neighbours=[],obstacle_id=None)
        else:
            self.list_nodes[self.end_node_id] = Node(self,position,self.end_node_id,type="end",neighbours=[],obstacle_id=None)

    def add_obstacle(self,list_vertices):
        current_obstacle_id = len(self.list_obstacles)
        first_id = self.size_nodes
        last_id = self.size_nodes + len(list_vertices)-1
        #add first node of obstacle
        self.add_node(list_vertices[0][0],neighbours=[last_id,first_id+1],obstacle_id=current_obstacle_id)
        for i in range(1,len(list_vertices)-1):
            self.add_node(list_vertices[i][0],neighbours=[self.size_nodes-1,self.size_nodes+1],obstacle_id=current_obstacle_id)
        #add last node of obstacle
        self.add_node(list_vertices[-1][0],neighbours=[last_id-1,first_id],obstacle_id=current_obstacle_id)
        #add the obstacle polygon
        self.list_obstacles.append(Polygon(*tuple([self.list_nodes[i].point2d for i in range(first_id,last_id+1)])))
            
    def print(self):
        counter = 0
        for _node in self.list_nodes:
            _node.print()
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

    def get_path(self,end_node_id):
        path_list = [self.list_nodes[end_node_id].position]
        current_id = end_node_id
        while current_id > 0:
            next_id = self.list_nodes[current_id].access_node
            current_id = next_id
            path_list.append(self.list_nodes[current_id].position)
        path_list.reverse()
        return path_list

    def update_visibility(self,node_a_id):
        for node_b_id in range(len(self.list_nodes)):
            if node_b_id in self.list_nodes[node_a_id].visibility_list:
                    continue
                visibility_line = Segment(self.list_nodes[node_a_id].point2d,self.list_nodes[node_b_id].point2d)
                visible = True
                for i in range(len(self.list_obstacles)):
                    n_inter = len(self.list_obstacles[i].intersection(visibility_line))
                    if self.list_nodes[node_a_id].obstacle_id == self.list_nodes[node_b_id].obstacle_id and self.list_nodes[node_a_id].obstacle_id == i:
                        if n_inter > 2:
                            visible = False
                            break
                        elif self.list_obstacles[i].encloses_point(visibility_line.midpoint):
                            visible = False
                            break
                    elif n_inter > 1:
                        visible = False
                        break
                if visible:
                    self.list_nodes[node_a_id].visibility_list.append(node_b_id)
                    self.list_nodes[node_b_id].visibility_list.append(node_a_id)
        print("Time compute visibility: {0}".format(time.time()-time_start))

    def compute_visibility_all(self):
        time_start = time.time()
        for node_a_id in range(self.size_nodes):
            for node_b_id in range(node_a_id+1,self.size_nodes):
                if node_b_id in self.list_nodes[node_a_id].visibility_list:
                    continue
                visibility_line = Segment(self.list_nodes[node_a_id].point2d,self.list_nodes[node_b_id].point2d)
                visible = True
                for i in range(len(self.list_obstacles)):
                    n_inter = len(self.list_obstacles[i].intersection(visibility_line))
                    if self.list_nodes[node_a_id].obstacle_id == self.list_nodes[node_b_id].obstacle_id and self.list_nodes[node_a_id].obstacle_id == i:
                        if n_inter > 2:
                            visible = False
                            break
                        elif self.list_obstacles[i].encloses_point(visibility_line.midpoint):
                            visible = False
                            break
                    elif n_inter > 1:
                        visible = False
                        break
                if visible:
                    self.list_nodes[node_a_id].visibility_list.append(node_b_id)
                    self.list_nodes[node_b_id].visibility_list.append(node_a_id)
        print("Time compute visibility: {0}".format(time.time()-time_start))

    def node_distance(self,node_a_id,node_b_id):
        return np.sqrt((self.list_nodes[node_a_id].position[0]-self.list_nodes[node_b_id].position[0])**2 + (self.list_nodes[node_a_id].position[1]-self.list_nodes[node_b_id].position[1])**2)

    def update_cost(self):
        time_start = time.time()
        current_nodes_id = [0]
        timeout = 0
        while (len(current_nodes_id) > 0) and (timeout < 20):
            timeout += 1
            next_nodes_id = []
            for id_base in current_nodes_id:
                for id_target in self.list_nodes[id_base].visibility_list:
                    cost = round(self.node_distance(id_base,id_target) + self.list_nodes[id_base].access_cost,2)
                    if (self.list_nodes[id_target].access_cost > cost) or ((self.list_nodes[id_target].access_cost == 0) and (self.list_nodes[id_target].type !="start")):
                        self.list_nodes[id_target].access_cost = cost
                        self.list_nodes[id_target].access_node = id_base
                        next_nodes_id.append(id_target)
            current_nodes_id = list(set(next_nodes_id[:])) #remove duplicate ids
        print("Time compute cost: {0}".format(time.time()-time_start))


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

if __name__ == "__main__":
    #========== COMPUTER VISION OUTPUT
    #cv_output = [[[1,1],[0,4],[4,4],[4,0]],[[5,5],[0,10],[12,10]]]
    #cv_output = [random_polygon(4,8,20,40,[25,25]),random_polygon(4,8,10,40,[50,80])]
    cv_output = [[[[ 248,  848]],[[238, 1079]],[[ 765, 1075]],[[ 756,  814]],[[ 377,  805]]],
                [[[1144,  810]],[[1146, 1079]],[[1674, 1079]],[[1661,  826]],[[1517,  796]]],
                [[[ 581,    0]],[[ 583,  377]],[[ 710,  659]],[[1176,  680]],[[1193,  228]],[[1053,  205]],[[1049,    0]]]]
    cv_start = [0,0]
    cv_end = [1300,400]

    #========== SETUP
    planner = Planner()
    planner.setup(cv_output)
    planner.set_start()
    planner.set_end()

    planner.update_cost()
    planner.get_path()


    """
    print(terrain.size_nodes)
    terrain.complete_visibility()
    terrain.complete_cost()
    print("==================================================")
    terrain.print()
    print("PATH: " + str(terrain.get_path(1)))
    plt.figure()
    terrain.plot_path(1)
    for polygon in cv_output:
        plt.plot([coord[0] for coord in polygon]+[polygon[0][0]], [coord[1] for coord in polygon]+[polygon[0][1]])
    plt.scatter(*tuple(cv_start))
    plt.scatter(*tuple(cv_end))
    #terrain.plot_visibility(5)
    plt.show()"""