"""
In dieser Datei werden sämtliche Sampling-Strategien implementiert die im Rahmen dieser Projektarbeit behandelt worden sind.
"""


import networkx as nx  # Library for hanling graphs (please check 4_Introduction_Graph)
import random # possibility to create random numbers (The "P" in PRM)

import numpy as np # functions for some calculations 
import math # functions for some calculations 

from IPPerfMonitor import IPPerfMonitor

# sort nearest neighbour brute force
import heapq
import copy
# reduce coding effort by using function provided by scipy
from scipy.spatial.distance import euclidean




def simple_Bridge_Sampling(collChecker):
    """
    In einer Schleife werden folgende Schritte durchgeführt um einen Punkt nach dem Bridge-Sampling zu generieren. Wird eine Bedingung in der Schleife nicht erfüllt beginnt diese von vorne:<br>
    Loop:
    1. Ein zufälliger Punkt Eins wird im C-Space generiert
    * **Bedingung**: Punkt Eins ist in Kollision
    2. Ein zufälliger Punkt Zwei wird im C-Space generiert. Dieser hat einen Abstand d, bestimmt nach der Gauß-Verteilung (siehe Gauß Sampling).
    * **Bedingung**: Punkt Zwei ist in Kollision
    3. Es wird in der Mitte zwischen den beiden Punkten ein dritter Punkt erstellt
    * **Bedingung**: Der dritte Punkt ist nicht in Kollision --> wird in die Roadmap hinzugefügt und die Schleife bricht ab
    """
    limits = collChecker.getEnvironmentLimits()  # get the limits from the enviroment 
    pos = [random.uniform(limit[0],limit[1]) for limit in limits] # generating a random pose within the limits 

    if not collChecker.pointInCollision(pos): # check if the pos is in collision 
        return False    # return false if pos is not in collision 
    d = np.random.normal(2,2) # get the distance d to the next point over a gaussian normal distribution 
    pos_x=pos[0]# store the x position of the pos
    pos_y=pos[1]# store the y position of the pos 
    alpha=random.uniform(0,360)*(math.pi/180) #get an random angle in rad between 0 and 180 degree 
    pos2_x=d*math.cos(alpha)+pos_x # calculate the x position for pos2
    pos2_y=d*math.sin(alpha)+pos_y # calculate the y position for pos2 
    pos2=[pos2_x,pos2_y]# store pos2 

    if not collChecker.pointInCollision(pos2):# check if pos2 is in collision 
        return False    # return false if pos is not in collision 
        
    pos3_x=(pos_x+pos2_x)/2 # calculate the x value of the intermediate point
    pos3_y=(pos_y+pos2_y)/2 # calculate the y value of the intermediate point 
    pos3=[pos3_x,pos3_y]# store pos3 

    if collChecker.pointInCollision(pos3):# check if pos3 is in collision 
        return False # return fals if pos3 is in collision 
        
    return pos3 # return pos3 wich now can be added to the roadmap 



def agressiv_Bridge_Sampeling(collChecker):
    """
    im Unterschied zum vorher gezeigten Verfahren wird der erste Punkt solange neu gewählt, bis ein Punkt gefunden wurde der Kollidiert. 
    Anschließend wird zu diesem Punkt in einer Schleife ein zweiter Punkt mit einem Abstand d(Gauße Normalverteilung) und varierenden Winkel gesucht. Dadurch, dass zu dem ersten gefunden Punkt ein zweiter, nicht kollidierender Punkt in einer Schleife gesucht wird, werden hier deutlich mehr Kollision Checks benötigt und die Berechnungsdauer dauert länger. Aufgrund Dessen wurden für die Benchmark Versuche die Funktion "Simple_Bridge_Sampling" verwendet.
    """

    limits = collChecker.getEnvironmentLimits()  # get the limits from the enviroment       
    pos = [random.uniform(limit[0],limit[1]) for limit in limits] # generating a random pose within the limits 


    #get a colliding point
    for t in range(0,10):# if no valid configuration with the pos was found pick a new first point(10 times)
        while not collChecker.pointInCollision(pos):# generate a randmom pos until the pos is in collision 
            pos = [random.uniform(limit[0],limit[1]) for limit in limits]
        
        
        pos_x=pos[0] # store the x position of the pos
        pos_y=pos[1] # store the y position of the pos

        
        for i in range(0,50): # check the angles for pos and a distance d if no valid config was found pick an new distance d(50 times)
            
            d=np.random.normal(2,2)# get the distance d to the next point over a gaussian normal distribution 
            angle_list = [0,2*math.pi] # create a angle list with value 0 and two pi
            for n in range(4):
                
                templist = copy.deepcopy(angle_list)# make a true copy from the angle list
                new_angles_list =[]# create a new angle list
                verschoben = 1 #set the var for the shift to one 
                for idx in range(len(templist)-1):
                    new_angle = (templist[idx]+templist[idx+1])/2 # create a new angle that is between the two picked angles 
                    new_angles_list.append(new_angle)# append the new angle
                    angle_list.insert(idx+verschoben,new_angle)# insert the new angle in the angle list
                    verschoben +=1 # increase the shift 
                for alpha in new_angles_list: # calculate the pos2 for the angles in the new_angle_list  
                    pos2_x= d*math.cos(alpha)+pos_x# calculate the x position for pos2
                    pos2_y= d*math.sin(alpha)+pos_y# calculate the y position for pos2
                    pos2=[pos2_x,pos2_y]# store pso2
                    if  collChecker.pointInCollision(pos2): #check if pos2 is in collision 
                        break # if pos2 is in collison continue wiht calculation of the third point 
                pos3_x=(pos_x+pos2_x)/2 # calculate the x value of the intermediate point
                pos3_y=(pos_y+pos2_y)/2 # calculate the y value of the intermediate point 
                pos3=[pos3_x,pos3_y]  #  store the pos3 
                if collChecker.pointInCollision(pos3): # check if pos3 is in collision 
                    continue # if in collision continue 
                else:
                    return pos3 # return pos3 wich now can be added to the roadmap  
                    
    return False # if no valild configuration could be found return False



def normal_Gaus_Sampling(collChecker):
    """
    Der Abstand d vom zweiten generierten Punkt zum Ersten wird zufällig mithilfe der Gaußverteilung bestimmt. <br>
    Hierbei gibt es folgende Paramenter:
    * **Mittelwert (Mean)**: Ist der "Hauptabstand" der am häufigsten genommen werden soll
    * **Standardabweichung (sigma)**: Gibt an wie oft und weit der Abstand d vom Hauptabstand abweichen soll 
    """
    # Get the limites for the graph 
    limits = collChecker.getEnvironmentLimits()   
    # Get a random position within the limits    
    pos = [random.uniform(limit[0],limit[1]) for limit in limits]
    # get a distance for the second Point over a gaussian distribution 
    d=np.random.normal(1,0.5) # get a distance d for the second point over a normal gaussian distribution
    pos_x=pos[0]# store the x value
    pos_y=pos[1]# store the y value
    
    # get a random angle between 0 and 360
    alpha=random.uniform(0,360)*(math.pi/180)
    # calculate the new Point with the random angle and the selected distance d 
    pos2_x=d*math.cos(alpha)+pos_x # calculate the x value of pos2
    pos2_y=d*math.sin(alpha)+pos_y# calculate the y valur of pos2 
    # store the Point 
    pos2=[pos2_x,pos2_y]# store pos2


        
    # check if the first Point is in collision and the secon ist collision free 
    if not collChecker.pointInCollision(pos2) and collChecker.pointInCollision(pos):
        return pos2 # return pos2 wich now can be added to the roadmap  
    # check if the second Point is in collision and the first is collision free 
    if not collChecker.pointInCollision(pos) and collChecker.pointInCollision(pos2):
        return pos # return pos wich now can be added to the roadmap  
    #if none of the both above conditions is true return False 
    return False



def simple_Gaussian_Sampling(collChecker):
    
    """
    1. finde einen Zufälligen Punkt im Konfigurationsraum und überprüfe diesen auf Kollision 
        * Kollisionsfrei: Punkt wird verworfen und Funktion liefert **False** zurück. 
        * Kollision: Puntk wird zwischengespeichert und X und Y Werte werden erfasst.
    2. Finde einen zweiten Punkt im Konfigurationsraum 
        * Der Abstand **d** zum zweiten Punkt wird zufällig aus einer Gaußschen Normalverteilung gewählt.
        * Wäle einen zufälligen Winkel **alpha** für die Richtung in der nach dem zweiten Punkt geschaut werden soll. 
        * Mittels dem Abstand **d** und dem Winkel **alpha** und trigonometrischen Funktionen werden die X und Y Koordinaten des zweiten Punktes berechnet. 
    3. Überprüfe den gefunden zweiten Punkt auf Kollision
        * Kollision: Punkt wird verworfen und die Funktion liefert **Fallse** zurück. 
        * Kollisionsfrei: Punkt wird zurückgeliefert -->Sampling war erfolgreich.
    """
    # Get the limites for the graph 
    limits = collChecker.getEnvironmentLimits()   
    # Get a random position within the limits    
    pos = [random.uniform(limit[0],limit[1]) for limit in limits]
    # If selected Configuration is not free pick the Point 
    if not collChecker.pointInCollision(pos):
        return False 
    # get a distance for the second Point over a gaussian distribution 
    d=np.random.normal(1,1)
    pos_x=pos[0]# store the x value 
    pos_y=pos[1]# store the y value 
    # get a random angle between 0 and 360 
    alpha=random.uniform(0,360)*(math.pi/180)
    # calculate the new Point with the random angle and the selected distance d 
    pos2_x=d*math.cos(alpha)+pos_x # calculate the x value of point 2 
    pos2_y=d*math.sin(alpha)+pos_y # calculate the y value of the point2 
   
    pos2=[pos2_x,pos2_y] #store the Point 
    # check if the Point is collision free 
    if collChecker.pointInCollision(pos2):
        return False
    
    return pos2 # return pos2 wich now can be added to the roadmap  



def agressiv_Gaussian_sampling(collChecker):
    
    """
    im Unterschied zu den vorher gezeigten Verfahren wird der erste Punkt solange neu gewählt, bis ein Punkt gefunden wurde der Kollidiert. 
    Anschließend wird zu diesem Punkt in einer Schleife ein zweiter Punkt mit einem Abstand d(Gauße Normalverteilung) und varierenden Winkel gesucht. Dadurch, dass zu dem ersten gefunden Punkt ein zweiter, nicht kollidierender Punkt in einer Schleife gesucht wird, werden hier deutlich mehr Kollision Checks benötigt und die Berechnungsdauer dauert länger. Aufgrund Dessen wurden für die Benchmark Versuche die Funktion "Simple_Gaussian_Sampling" verwendet.
    """
    limits = collChecker.getEnvironmentLimits()# Get the limites for the graph 
    pos = [random.uniform(limit[0], limit[1]) for limit in limits]# Get a Random position within the limits 

    # get a colliding point
    for t in range(0, 10):# if no valid configuration with the pos was found pick a new first point(10 times)
        while not collChecker.pointInCollision(pos):# generate a randmom pos until the pos is in collision 
            pos = [random.uniform(limit[0], limit[1]) for limit in limits]

       
        pos_x = pos[0]# store the x value of the point 
        pos_y = pos[1]# store the y value of the point 

        # find a non colliding point in a given distance to point a
        for i in range(0, 50): # check the angles for pos and a distance d if no valid config was found pick an new distance d(50 times)
            d = np.random.normal(1, 1)# get the distance d to the next point over a gaussian normal distribution 
            angle_list = [0, 2*math.pi]# create a angle list with value 0 and two pi
            for n in range(8):
                templist = copy.deepcopy(angle_list)# make a true copy from the angle list
                new_angles_list = []# create a new angle list
                verschoben = 1 #set the var for the shift to one 
                for idx in range(len(templist)-1):
                    new_angle = (templist[idx]+templist[idx+1])/2# create a new angle that is between the two picked angles 
                    new_angles_list.append(new_angle)# append the new angle
                    angle_list.insert(idx+verschoben, new_angle)# insert the new angle in the angle list
                    verschoben += 1 # increase the shift 
                for alpha in new_angles_list: # calculate the pos2 for the angles in the new_angle_list  
                    pos2_x = d*math.cos(alpha)+pos_x # calculate the x position for pos2
                    pos2_y = d*math.sin(alpha)+pos_y # calculate the y position for pos2
                    pos2 = [pos2_x, pos2_y]# store pos2
                    if not collChecker.pointInCollision(pos2):# check if pos2 is not in collision 
                        return pos2 # return pos3 wich now can be added to the roadmap  

    return False # if no valild configuration could be found return False