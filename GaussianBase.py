from IPPerfMonitor import IPPerfMonitor


from IPPlanerBase import PlanerBase


import random
import numpy as np
import math


class C_GaussianBase(PlanerBase):

    def __init__(self, collChecker):
        super(C_GaussianBase, self).__init__(collChecker)

    def _getRandomPosition(self):
        limits = self._collisionChecker.getEnvironmentLimits()
        pos = [random.uniform(limit[0], limit[1]) for limit in limits]
        return pos

    @IPPerfMonitor
    def _getRandomFreePosition(self):
        pos = self._getRandomPosition()
        while self._collisionChecker.pointInCollision(pos):
            pos = self._getRandomPosition()
        return pos

    @IPPerfMonitor
    def simple_Gaus_Sampling(collChecker):

        # Get the limites for the graph
        limits = collChecker.getEnvironmentLimits()
    # Get a random position within the limits
        pos = [random.uniform(limit[0], limit[1]) for limit in limits]
    # If selected Configuration is not free pick the Point
        if not collChecker.pointInCollision(pos):
            return False
    # get a distance for the second Point over a gaussian distribution
        d = np.random.normal(1, 1)
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
