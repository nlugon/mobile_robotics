import matplotlib.pyplot as plt
from sympy import Point, Polygon, Line, Segment
import random
import numpy as np

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

if __name__ == "__main__":
    terrain = Terrain()
    #cv_output = [[[1,1],[0,4],[4,4],[4,0]],[[5,5],[0,10],[12,10]]]
    cv_output = [random_polygon(4,8,20,40,[25,25]),random_polygon(4,8,10,40,[50,80])]
    cv_start = [0,0]
    cv_end = [100,100]

    #add start and end to terrain
    terrain.add_node(cv_start,type="start",neighbours=[])
    terrain.add_node(cv_end,type="end",neighbours=[])

    for _obstacle in cv_output:
        terrain.add_obstacle(_obstacle)
    
    print(terrain.size_nodes)
    terrain.complete_visibility()
    terrain.complete_cost()
    print("==================================================")
    terrain.print()

    plt.figure()
    terrain.plot_path(1)
    for polygon in cv_output:
        plt.plot([coord[0] for coord in polygon]+[polygon[0][0]], [coord[1] for coord in polygon]+[polygon[0][1]])
    plt.scatter(*tuple(cv_start))
    plt.scatter(*tuple(cv_end))
    #terrain.plot_visibility(5)
    plt.show()