# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from scipy import spatial
import math
from numpy import random
from PIL import Image

# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path


    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''
        # Check obstacle between nodes
        # get all the poitns in between
        points_between = zip(np.linspace(p1[0], p2[0], dtype=int), 
                             np.linspace(p1[1], p2[1], dtype=int))
        # check if any of these are obstacles
        for p in points_between:
            if self.map_array[p[0]][p[1]] == 0:
                return True
        return False

    def dis(self, point1, point2):                          
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        ### YOUR CODE HERE ###
        distance=math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)             # euclidean distance between 2 points
        return distance


    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        points=int(math.sqrt(n_pts))                            
        xdir=self.size_row
        ydir=self.size_col

        xsample= np.linspace(0,xdir-1,num=points)               # equally divide the graph based on number of points 
        ysample= np.linspace(0,ydir-1,num=points)
        xsample=np.round(xsample,decimals=0)                    # get interger value
        ysample=np.round(ysample,decimals=0)

        for i in xsample:
            for j in ysample:
                if self.map_array[int(i),int(j)] != 0:      
                    self.samples.append((int(i),int(j)))
    
    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        points=[]
        for i in range(n_pts):

            p1rows=random.randint(self.size_row)                        # randomise points on map
            p1cols=random.randint(self.size_col)
            p=(p1rows,p1cols)
            points.append(p)

        for i in range(len(points)):
            if self.map_array[points[i][0]][points[i][1]]==1:           # check points in free space 
                p=[points[i][0],points[i][1]]
                self.samples.append(p)


    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        self.samples.append((0, 0))
        obs_points=[]
        for i in range(n_pts):

            p1rows=random.randint(self.size_row)            # find random points that are within obstacle
            p1cols=random.randint(self.size_col)
            p=(p1rows,p1cols)
            if self.map_array[p]==0:                
                obs_points.append(p)

        for i in range(len(obs_points)):
            condition=True
            while condition:                                # find obstace in free space within gausian distance 
                p2x=int(np.random.normal(obs_points[i][0],10))
                p2y=int(np.random.normal(obs_points[i][1],10))

                if p2x<self.size_row and p2y<self.size_col and self.map_array[(p2x,p2y)]== 1:
                    self.samples.append((p2x,p2y))
                    condition=False


    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        self.samples.append((0, 0))
        obs_points=[]
        for i in range(n_pts):
            p1rows=random.randint(self.size_row)
            p1cols=random.randint(self.size_col)
            p=(p1rows,p1cols)
            if self.map_array[p]==0:                    # find random points that are within obstacle 
                obs_points.append(p)

        for i in range(len(obs_points)):
            p2x=int(np.random.normal(obs_points[i][0],13))
            p2y=int(np.random.normal(obs_points[i][1],13))

            if p2x<self.size_row and p2y<self.size_col and self.map_array[(p2x,p2y)]== 0:       # find another random poin within gausian distance and inside another obstacle
                midx= int((obs_points[i][0]+p2x)/2)
                midy= int((obs_points[i][1]+p2y)/2)
                if self.map_array[(midx,midy)] == 1:                                            # if midpoint between the 2 points lie in free space, append in samples 
                    self.samples.append((midx,midy))

    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict( zip( range( len(self.samples) ), node_pos) )
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])
        
        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y' ,ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12,  node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12,  node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()


    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
            r=15                                    # setting conectivity between samples
            a=10                                    # setting connectivity of start/goal with nearby samples
        elif sampling_method == "random":
            self.random_sample(n_pts)
            r=20
            a=15
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
            r=25
            a=75
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)
            r=30
            a=75

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # Store them as
        # pairs = [(p_id0, p_id1, weight_01), (p_id0, p_id2, weight_02), 
        #          (p_id1, p_id2, weight_12) ...]
        pairs = []                                                  # using resourse provided in readme file and KD Tree
        positions=np.array(self.samples)
        kdtree = spatial.KDTree(positions)
        pairsid = kdtree.query_pairs(r)                             # find k nearest neighbors
        pairsid = list(pairsid)

        for i in range(len(pairsid)):
            point11 = self.samples[pairsid[i][0]]                   # take 2 points from index values
            point21 = self.samples[pairsid[i][1]]
            collision = self.check_collision(point11,point21)      
            if collision == True:
                continue
            else:
                distance = self.dis(point11,point21)                # use pairs ID generated and find distance and append together in pairs
                if distance!=0:
                    p1index=self.samples.index(point11)
                    p2index=self.samples.index(point21)
                    pairs.append((p1index,p2index,distance))

        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01), 
        #                                     (p_id0, p_id2, weight_02), 
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of 
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.
        self.graph.add_nodes_from([])
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))
        return a


    def search(self, start, goal,a):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0), (start_id, p_id1, weight_s1), 
        #                (start_id, p_id2, weight_s2) ...]
        start_pairs = []
        for i in range(len(self.samples)):
            b=len(self.samples)
            point11 = self.samples[b-2]         #start point 
            point21 = self.samples[i]                           # iterate over all points in samples
            if point11!=point21:
                collision = self.check_collision(point11,point21)    
                if collision == True:
                    continue
                else:
                    distance = self.dis(point11,point21)            # if no collison between points find distance
                    if distance!=0 and distance<a:                      
                        p1index = self.samples.index(point11)       # find index of point in self.samples 
                        p2index = self.samples.index(point21)
                        start_pairs.append(('start',p2index,distance))      # append index and distance

        goal_pairs = []                                                 # Follow same steps for goal as did in start
        for i in range(len(self.samples)):
            b=len(self.samples)
            point11 = self.samples[b-1]                 # take goal point 
            point21 = self.samples[i]
            if point11!=point21:
                collision = self.check_collision(point11,point21)    
                if collision == True:
                    continue
                else:
                    distance = self.dis(point11,point21)
                    if distance!=0 and distance<a:
                        p1index = self.samples.index(point11)
                        p2index = self.samples.index(point21)
                        goal_pairs.append(('goal',p2index,distance))

        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)
        
        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" %path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")
        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
        