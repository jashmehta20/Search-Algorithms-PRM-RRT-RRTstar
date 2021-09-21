# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from scipy import spatial
import math
from numpy import random
from PIL import Image
from random import randrange

# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        distance=math.sqrt((node1.row-node2.row)**2 + (node1.col-node2.col)**2)             # Calculating euclidean distance
        return distance
    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if there are obstacles
            False if the new node is valid to be connected
        '''
        # Check obstacle between nodes
        # get all the points in between
        points_between = zip(np.linspace(node1.row, node2.row, dtype=int), 
                             np.linspace(node1.col, node2.col, dtype=int))
        # check if any of these are obstacles
        for point in points_between:
            if self.map_array[point[0]][point[1]] == 0:
                return True
        return False

    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###
        a=randrange(100)                                # pick random variable for goal bias 
        if a<=goal_bias:                                # if value within goal bias pick new point as goal
            y=True
        else:
            y=False
        if y==False:
            p1rows=random.randint(self.size_row)
            p1cols=random.randint(self.size_col)
            new_point=Node(p1rows,p1cols)
            return new_point
        
        else:
            return self.goal

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        dist_new_node=[]
        minindex=0
        for i in range(len(self.vertices)):
            distance=self.dis(self.vertices[i],point)           # calculate distance of new point with respect to all nodes and append in dist_new_node
            dist_new_node.append(distance)
        
        minindex=dist_new_node.index(min(dist_new_node))        # find index value of minimum distance to node and return node 
        return self.vertices[minindex]  


    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        neighbors=[]
        freeneighbors=[]
        for i in range(len(self.vertices)):     
            distance_rrt=self.dis(new_node,self.vertices[i])
            if distance_rrt<=neighbor_size:                     # find all neighbors within neighbour_size distance from new_node
                neighbors.append(self.vertices[i])

        for i in range(len(neighbors)):                         # find only the neighbouring nodes that do not have obstacles in between 
            collision_1=self.check_collision(neighbors[i],new_node)
            if collision_1==True:
                continue
            else:
                freeneighbors.append(neighbors[i])
        return freeneighbors                                # Return neighbours that are free from collision 


    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###
        checkdis=[]
        free=[]
        for i in range(len(neighbors)):                         # find neighbors that do not have obstacles inbetween them 
            collision=self.check_collision(neighbors[i],new_node)
            if collision==False:
                free.append(neighbors[i])
            else:
                continue
        for i in range(len(free)):                              # out of the free neighbors, find neighbors that have lowest cost to traverse
            cost1=free[i].cost+self.dis(free[i],new_node)
            checkdis.append(cost1)

        minindex=checkdis.index(min(checkdis))
        new_node.parent=neighbors[minindex]                     # make parent node of new node as the neighbor with lowest cost
        new_node.cost=neighbors[minindex].cost+int(self.dis(neighbors[minindex],new_node))  # calculate and update new node cost

        for i in range(len(free)):                              # check cost of rewiring 
            initcost= free[i].cost                              # if cost before rewire cost is more, we change parent node and update new cost
            rewiredcost=int(self.dis(new_node,free[i])) + new_node.cost

            if initcost > rewiredcost:
                neighbors[i].parent=new_node
                neighbors[i].cost=rewiredcost
            else:
                continue
        
    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col and cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()

    def angle(self,p1,p2):                          # using angle components to find new node in direction of point from nearest node
        step=10 
        distance_1=self.dis(p1,p2)
        if distance_1 > step:
            distance_1= step
        
        theta= math.atan2(p2.col - p1.col, p2.row - p1.row)
        new_node1=Node((int((p1.row+distance_1*math.cos(theta)))),(int((p1.col+distance_1*math.sin(theta)))))

        return new_node1

   
    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()
        ### YOUR CODE HERE ###
        goal_bias = 1                               # setting goal bias as 1 i.e. 1% chance of picking goal as new point
        step=10
        for i in range(n_pts):
            new_p = self.get_new_point(goal_bias)   # generate new point
            if self.map_array[new_p.row][new_p.col]==0:     # Check if new point is in collision 
                continue
            else:
                n_n = self.get_nearest_node(new_p)     # find nearest node to point 

                new_node =  self.angle(n_n,new_p)      # find new node in direction of new point

                collision = self.check_collision(n_n,new_node) # check if there is collision between new node and neighbor node 
                if collision == True:
                    continue
                else:
                    self.vertices.append(new_node)              # if no collision append new node 
                    new_node.cost = int(n_n.cost +self.dis(n_n,new_node))      # calculate cost of new node 
                    new_node.parent=n_n                # update parent node of new node and nearest node
                    dist = self.dis(new_node,self.goal)     # calculate distance to goal 


                    if dist<=step:            # if new node is very close to final goal

                        x = new_node                 # make nearest node as new node and new node as final goal
                        n_n = x
                        new_node = self.goal

                        new_node.parent = n_n       # set parent of new node(goal node) as nearest node(new node) 
                        new_node.cost = int(n_n.cost +self.dis(n_n,new_node))
                        self.vertices.append(new_node)

                        self.found=True 

                        break                       # terminate search as soon as goal is found 

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        self.draw_map()


    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.

        goal1 = False                           # if goal is close to new node set true 
        goal_bias = 4
        step=10
        for i in range(n_pts):

            print("Nodes tested:",i)
            new_p = self.get_new_point(goal_bias)           # get new point in free space
            if self.map_array[new_p.row][new_p.col]==0:
                continue
            else:
                n_n = self.get_nearest_node(new_p)          # get nearest neighbor of new point 
                if goal1==True:                             # if close to goal set new node as final goal 
                    new_node = self.goal
                else:
                    new_node = self.angle(n_n,new_p)        # find new node along direction of point 
                collision = self.check_collision(new_node,n_n)      # check collison between new node and nearest neighbor  
                if collision == True:
                    continue
                else:
                    neighbors = self.get_neighbors(new_node,20)         # find neighbors of new node
                    self.rewire(new_node,neighbors)                     # rewire all neighbors 
                    dist = self.dis(new_node,self.goal)     # Find distance to goal. If close to goal set goal1 as true and appened new node
                    if dist<=step:
                        goal1=True
                    self.vertices.append(new_node)

                    if dist==0:
                        self.found=True
                        goal1=False

         # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()

