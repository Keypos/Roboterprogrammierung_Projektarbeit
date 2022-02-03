import random
import math
import copy
import heapq
import numpy as np
import networkx as nx
import IPPRMBase
from IPPerfMonitor import IPPerfMonitor

from sampling_algorithms import agressiv_Gaussian_sampling, simple_Gaussian_Sampling, normal_Gaus_Sampling, simple_Bridge_Sampling
from sampstrats import Gaussian_sampling

from scipy.spatial.distance import euclidean, cityblock


class GaussianPRM(IPPRMBase.PRMBase):
    """
    Klasse, die das normale Gauss-Sampling implementiert
    """
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

        # Decide which method to use
        sample_func = None
        if config['method'] == 'simple':
            sample_func = simple_Gaussian_Sampling
        elif config['method'] == 'normal':
            sample_func = Gaussian_sampling
        elif config['method'] == 'aggressive':
            sample_func = agressiv_Gaussian_sampling

        i = 1
        while i < config['numNodes']:
            # Generate a 'randomly chosen, free configuration'
            pos = False
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



class BridgePRM(IPPRMBase.PRMBase):
    """
    Klasse, die das normale Bridge-Sampling implementiert
    """
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

        # Decide which method to use
        sample_func = None
        if config['method'] == 'simple':
            sample_func = simple_Bridge_Sampling
        elif config['method'] == 'aggressive':
            sample_func = agressiv_Gaussian_sampling

        i = 1
        while i < config['numNodes']:
            # Generate a 'randomly chosen, free configuration'
            pos = False
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
    """
    Class that implements the mixed methods of basicPRM and GaussPRM
    """
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

        # Decide which method to use
        sample_func = None
        if config['method'] == 'simple':
            sample_func = simple_Gaussian_Sampling
        elif config['method'] == 'normal':
            sample_func = Gaussian_sampling
        elif config['method'] == 'aggressive':
            sample_func = agressiv_Gaussian_sampling        

        i = 1
        while i <= config['numNodesBasic']:
            # Generate a 'randomly chosen, free configuration'
            newNodePos = self._getRandomFreePosition()
            self.graph.add_node(i, pos=newNodePos, color='#970a7e')
            i += 1

        j = i
        while j <= config['numNodesGauss'] + i:
            # Generate a 'randomly chosen, free configuration'
            pos = False
            while pos == False:
                pos = sample_func(self._collisionChecker)
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