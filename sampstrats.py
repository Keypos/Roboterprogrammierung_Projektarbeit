import random
import math
import copy
import heapq
import numpy as np
import networkx as nx
import IPPRMBase
from IPPerfMonitor import IPPerfMonitor

from scipy.spatial.distance import euclidean, cityblock


class GaussianPRM(IPPRMBase.PRMBase):

    def __init__(self, _collChecker):
        super(GaussianPRM, self).__init__(_collChecker)
        self.graph = nx.Graph()

    def _inSameConnectedComponent(self, node1, node2):
        """ Check whether to nodes are part of the same connected component using
            functionality from NetworkX
        """
        for connectedComponent in nx.connected_components(self.graph):
            if (node1 in connectedComponent) & (node2 in connectedComponent):
                return True

        return False

    def _nearestNeighboursX(self, pos, radius):
        """ Brute Force method to find all nodes of a 
        graph near the given position **pos** with in the distance of
        **radius** in **increasing order**"""

        heap = list()
        # using (data=True) will generate a list of nodes with all attributes
        for node in self.graph.nodes(data=True):
            if euclidean(node[1]['pos'], pos) < radius:
                # use a heap-queue to sort the nodes in increasing order
                heapq.heappush(heap, (euclidean(node[1]['pos'], pos), node))
                # if len(heap) > 2 :
                #    break

        result = list()
        while len(heap) > 0:
            result.append(heapq.heappop(heap))

        return result

    def _learnRoadmapNearestNeighbour(self, config):
        i = 1
        while i < config['numNodes']:

            sample_func = simple_Gaus_Sampling if config['simple'] else Gaussian_sampling

            # Generate a 'randomly chosen, free configuration'
            pos = sample_func(self._collisionChecker)
            while pos == False:
                pos = sample_func(self._collisionChecker)

            # Find set of candidates to connect to sorted by distance
            result = self._nearestNeighboursX(pos, config['radius'])

            # check connection
            self.graph.add_node(i, pos=pos)
            for idx, data in enumerate(result):
                if not self._inSameConnectedComponent(i, data[1][0]):
                    if not self._collisionChecker.lineInCollision(pos, data[1][1]['pos']):
                        self.graph.add_edge(i, data[1][0])

            i += 1

    def planPath(self, start, goal, config):

        self._learnRoadmapNearestNeighbour(config)

        # find nearest, collision-free connection between node on graph and start
        result = self._nearestNeighboursX(start, config['radius'])
        for node in result:
            if not self._collisionChecker.lineInCollision(start, node[1][1]['pos']):
                self.graph.add_node("start", pos=start)
                self.graph.add_edge("start", node[1][0])
                break
        # find nearest, collision-free connection between node on graph and goal
        result = self._nearestNeighboursX(goal, config['radius'])

        for node in result:
            if not self._collisionChecker.lineInCollision(goal, node[1][1]['pos']):
                self.graph.add_node("goal", pos=goal)
                self.graph.add_edge("goal", node[1][0])
                break
        # find shortest path on graph
        path = nx.shortest_path(self.graph, "start", "goal")
        # return nodelist
        return path


class BasicGaussianPRM(IPPRMBase.PRMBase):

    def __init__(self, _collChecker):
        super(BasicGaussianPRM, self).__init__(_collChecker)
        self.graph = nx.Graph()

    @IPPerfMonitor
    def _inSameConnectedComponent(self, node1, node2):
        """ Check whether to nodes are part of the same connected component using
            functionality from NetworkX
        """
        for connectedComponent in nx.connected_components(self.graph):
            if (node1 in connectedComponent) & (node2 in connectedComponent):
                return True

        return False

    @IPPerfMonitor
    def _nearestNeighboursX(self, pos, radius):
        """ Brute Force method to find all nodes of a 
        graph near the given position **pos** with in the distance of
        **radius** in **increasing order**"""

        heap = list()
        # using (data=True) will generate a list of nodes with all attributes
        for node in self.graph.nodes(data=True):
            if euclidean(node[1]['pos'], pos) < radius:
                # use a heap-queue to sort the nodes in increasing order
                heapq.heappush(heap, (euclidean(node[1]['pos'], pos), node))
                # if len(heap) > 2 :
                #    break

        result = list()
        while len(heap) > 0:
            result.append(heapq.heappop(heap))

        return result

    @IPPerfMonitor
    def _learnRoadmapNearestNeighbour(self, config):

        i = 1
        while i <= config['numNodesBasic']:
            # Generate a 'randomly chosen, free configuration'
            newNodePos = self._getRandomFreePosition()
            self.graph.add_node(i, pos=newNodePos, color='#970a7e')
            i += 1


        j = i
        while j <= config['numNodesGauss'] + i:
            # Generate a 'randomly chosen, free configuration'
            pos = simple_Gaus_Sampling(self._collisionChecker)
            while pos == False:
                pos = simple_Gaus_Sampling(self._collisionChecker)
            self.graph.add_node(j, pos=pos, color='#9b870c')
            j += 1
        
        for node in self.graph.nodes(data=True):
            # Find set of candidates to connect to sorted by distance
            result = self._nearestNeighboursX(node[1]['pos'], config['radius'])
            for idx, data in enumerate(result):
                if not self._inSameConnectedComponent(node[0], data[1][0]):
                    if not self._collisionChecker.lineInCollision(node[1]['pos'], data[1][1]['pos']):
                        self.graph.add_edge(node[0], data[1][0])

    @IPPerfMonitor
    def planPath(self, startList, goalList, config):

        checkedStartList, checkedGoalList = self._checkStartGoal(
            startList, goalList)

        self._learnRoadmapNearestNeighbour(config)

        # find nearest, collision-free connection between node on graph and start
        result = self._nearestNeighboursX(checkedStartList[0], config['radius'])
        for node in result:
            print(node)
            if not self._collisionChecker.lineInCollision(checkedStartList[0], node[1][1]['pos']):
                self.graph.add_node("start", pos=checkedStartList[0], color='#00dd00')
                self.graph.add_edge("start", node[1][0])
                break
        # find nearest, collision-free connection between node on graph and goal
        result = self._nearestNeighboursX(checkedGoalList[0], config['radius'])

        for node in result:
            if not self._collisionChecker.lineInCollision(checkedGoalList[0], node[1][1]['pos']):
                self.graph.add_node("goal", pos=checkedGoalList[0], color='#DD0000')
                self.graph.add_edge("goal", node[1][0])
                break

        try:
            # find shortest path on graph
            path = nx.shortest_path(self.graph, "start", "goal")
        except:
            return []
        
        # return nodelist
        return path


# collChecker needed
# Functions
# Trys to get wanted pos else returns False
# -------------Bridge Functions----------------
def simple_Bridge_Sampling(collChecker):
    limits = collChecker.getEnvironmentLimits()
    pos = [random.uniform(limit[0], limit[1]) for limit in limits]

    if not collChecker.pointInCollision(pos):
        return False
    d = np.random.normal(1, 4)
    pos_x = pos[0]
    pos_y = pos[1]
    alpha = random.uniform(0, 360)*(math.pi/180)  # get an random angle in rad
    pos2_x = d*math.cos(alpha)+pos_x
    pos2_y = d*math.sin(alpha)+pos_y
    pos2 = [pos2_x, pos2_y]

    if not collChecker.pointInCollision(pos2):
        return False

    pos3_x = (pos_x+pos2_x)/2
    pos3_y = (pos_y+pos2_y)/2
    pos3 = [pos3_x, pos3_y]

    if collChecker.pointInCollision(pos3):
        return False

    return pos3


def Bridge_Sampeling(collChecker):

    limits = collChecker.getEnvironmentLimits()
    pos = [random.uniform(limit[0], limit[1]) for limit in limits]

    # get a colliding point
    for t in range(0, 10):
        while not collChecker.pointInCollision(pos):
            pos = [random.uniform(limit[0], limit[1]) for limit in limits]

        # store the x and y value of the colliding point
        pos_x = pos[0]
        pos_y = pos[1]

        for i in range(0, 50):
            # get a distance over a gaussian distribution
            me, sigma = 1, 1  # Mean value of the gaussian distribution, standard deviation
            d = np.random.normal(me, sigma)
            angle_list = [0, 2*math.pi]
            for n in range(2):
                # alpha=random.uniform(0,360)*(180/math.pi) #get an random angle in ra
                templist = copy.deepcopy(angle_list)
                new_angles_list = []
                verschoben = 1  # ja es gibt was hier zu sehen
                for idx in range(len(templist)-1):
                    new_angle = (templist[idx]+templist[idx+1])/2
                    new_angles_list.append(new_angle)
                    angle_list.insert(idx+verschoben, new_angle)
                    verschoben += 1
                for alpha in new_angles_list:
                    pos2_x = d*math.cos(alpha)+pos_x
                    pos2_y = d*math.sin(alpha)+pos_y
                    pos2 = [pos2_x, pos2_y]
                    # return point when in collision
                    if collChecker.pointInCollision(pos2):
                        break
                pos3_x = (pos_x+pos2_x)/2
                pos3_y = (pos_y+pos2_y)/2
                pos3 = [pos3_x, pos3_y]
                if collChecker.pointInCollision(pos3):
                    continue
                else:
                    return pos3

    # return [1,2]
    # if no point found with gaussian pick a random collison free point
    """
    while  collChecker.pointInCollision(pos):
        pos = [random.uniform(limit[0],limit[1]) for limit in limits]
    return pos
    """
    return False

# -------------Gaussian Functions----------------


def Gaussian_sampling(collChecker):

    limits = collChecker.getEnvironmentLimits()
    pos = [random.uniform(limit[0], limit[1]) for limit in limits]

    # get a colliding point
    for t in range(0, 10):
        while not collChecker.pointInCollision(pos):
            pos = [random.uniform(limit[0], limit[1]) for limit in limits]

        # store the x and y value of the colliding point
        pos_x = pos[0]
        pos_y = pos[1]

        # find a non colliding point in a given distance to point a
        for i in range(0, 50):
            # get a distance over a gaussian distribution
            # me,sigma = 0,1 #Mean value of the gaussian distribution, standard deviation
            d = np.random.normal(1, 1)
            angle_list = [0, 2*math.pi]
            for n in range(8):
                # alpha=random.uniform(0,360)*(180/math.pi) #get an random angle in ra
                templist = copy.deepcopy(angle_list)
                new_angles_list = []
                verschoben = 1  # ja es gibt was hier zu sehen
                for idx in range(len(templist)-1):
                    new_angle = (templist[idx]+templist[idx+1])/2
                    new_angles_list.append(new_angle)
                    angle_list.insert(idx+verschoben, new_angle)
                    verschoben += 1
                for alpha in new_angles_list:
                    pos2_x = d*math.cos(alpha)+pos_x
                    pos2_y = d*math.sin(alpha)+pos_y
                    pos2 = [pos2_x, pos2_y]
                    # return point when collision free
                    if not collChecker.pointInCollision(pos2):
                        return pos2

    return False
    # if no point found with gaussian pick a random collison free point
    """"
    while collChecker.pointInCollision(pos):
        pos = [random.uniform(limit[0],limit[1]) for limit in limits]
    return pos
    """


def simple_Gaus_Sampling(collChecker):

    # Get the limites for the graph
    limits = collChecker.getEnvironmentLimits()
    # Get a random position within the limits
    pos = [random.uniform(limit[0], limit[1]) for limit in limits]
    # If selected Configuration is not free pick the Point
    if not collChecker.pointInCollision(pos):
        return False
    # get a distance for the second Point over a gaussian distribution
    d = np.random.normal(0, 0.75)
    pos_x = pos[0]
    pos_y = pos[1]
    # get a random angle between 0 and 360
    alpha = random.uniform(0, 360)*(math.pi/180)
    # calculate the new Point with the random angle and the selected distance d
    pos2_x = d*math.cos(alpha)+pos_x
    pos2_y = d*math.sin(alpha)+pos_y
    # store the Point
    pos2 = [pos2_x, pos2_y]
    # check if the Point is collision free
    if collChecker.pointInCollision(pos2):
        return False
    # if the point is  collision free return it
    return pos2
