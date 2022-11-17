import matplotlib.pyplot as plt
from sympy import Point, Polygon, Line, Segment
import random
import numpy as np

#DUMMY
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

def plot_node_line(node_a,node_b,color="yellow"):
    plt.plot([node_a.coords[0],node_b.coords[0]],[node_a.coords[1],node_b.coords[1]],color=color)

class Node:
    def __init__(self,coords,node_id,obs_id,start=False,stop=False):
        #node identity
        self.coords = coords
        self.node_id = node_id
        self.obs_id = obs_id
        self.point2d = Point(coords[0],coords[1])

        #node properties
        self.start = start
        self.stop = stop

        #connections
        self.access_cost = 0
        self.access_node = [] #contains [obs_id,node_id] of node with less access cost
        self.visibility_list = [] #contains [obs_id,node_id] of the accessible nodes

    def plot(self,color="green"):
        plt.scatter(self.coords[0],self.coords[1],color=color)

    def print(self):
        print("== Node({0},{1}) : coords({2}), visibility({3})".format(self.obs_id,self.node_id,str(self.coords),str(self.visibility_list)))

#list of nodes, and a polygon that defines an obstacle
class Obstacle:
    def __init__(self,obs_id):
        self.obs_id = obs_id
        self.node_list = []
    
    def add_node(self,node):
        self.node_list.append(node)

    def create_polygon(self):
        self.polygon = Polygon(*tuple([node.point2d for node in self.node_list]))

    def plot(self):
        x_axis = [node.coords[0] for node in self.node_list]
        x_axis.append(x_axis[0])
        y_axis = [node.coords[1] for node in self.node_list]
        y_axis.append(y_axis[0])
        plt.plot(x_axis,y_axis)

#contains the list of obstacles and functions
class NodeNetwork:
    def __init__(self,vertices_list,start_pos,stop_pos):
        self.network = [] #list of Obstacle class objects
        self.node_start = Node(start_pos,0,-1,start=True)
        self.node_stop = Node(stop_pos,1,-1,start=False)
        for _obstacle in vertices_list:
            obs_id = len(self.network)
            self.network.append(Obstacle(obs_id))
            for _node in _obstacle:
                node_id = len(self.network[-1].node_list)
                self.network[-1].add_node(Node(_node,node_id,obs_id))
            self.network[-1].create_polygon()

    def plot(self):
        self.node_start.plot(color="purple")
        self.node_stop.plot(color="purple")
        for _obs in self.network:
            _obs.plot()

    def plot_visibility(self,obs_id,node_id):
        if obs_id == -1:
            if node_id == 0:
                plot_node = self.node_start
            else:
                plot_node = self.node_stop
        else:
            plot_node = self.network[obs_id].node_list[node_id]
        plot_node.plot("red")
        for id in plot_node.visibility_list:
            self.network[id[0]].node_list[id[1]].plot()
            plot_node_line(self.network[id[0]].node_list[id[1]],plot_node)
        
        
        


    def check_visibility(self,node_a,node_b): #checks if both points can see each other
        visibility_line = Polygon(node_a.point2d,node_b.point2d)
        obs_a = self.network[node_a.obs_id]
        if node_a.obs_id == node_b.obs_id: #if both nodes are on the same obstacle
            #check if nodes are neighbours
            midpoint_ab = Segment(node_a.point2d,node_b.point2d).midpoint
            if ((node_a.node_id+1)%len(obs_a.node_list) == node_b.node_id) or ((node_a.node_id+len(obs_a.node_list)-1)%len(obs_a.node_list) == node_b.node_id):
                return True
            elif obs_a.polygon.encloses_point(midpoint_ab):
                return False

        for _obs in self.network:
            if (obs_a.obs_id == _obs.obs_id) and (node_a.obs_id == node_b.obs_id):
                continue
            if len(_obs.polygon.intersection(visibility_line)) > 1:
                return False
        return True

    #fill the access data of each node
    def compute_visibility(self):
        for current_obs in self.network:
            for current_node in current_obs.node_list:
                #check visibility with other nodes
                for _obs in self.network:
                    for _node in _obs.node_list:
                        if current_node.coords == _node.coords:
                            continue
                        elif self.check_visibility(current_node,_node):
                            current_node.visibility_list.append([_node.obs_id,_node.node_id])

    def compute_access_costs(self):
        pass





if __name__ == "__main__":
    start_pos = [0,0]
    stop_pos = [100,100]
    test_polygon = [random_polygon(4,6,20,40,center=[80,80]),random_polygon(4,6,20,40,center=[30,30])]
    test_obs = NodeNetwork(test_polygon,start_pos,stop_pos)
    test_obs.compute_visibility()
    vis_id = [0,4]
    plt.figure()
    test_obs.plot_visibility(vis_id[0],vis_id[1])
    test_obs.plot()
    plt.show()
