import random
import math
import copy
import numpy as np
#collChecker needed
#Functions
# Trys to get wanted pos else returns False
#-------------Bridge Functions----------------
def simple_Bridge_Sampling(collChecker):
    limits = collChecker.getEnvironmentLimits()        
    pos = [random.uniform(limit[0],limit[1]) for limit in limits]

    if not collChecker.pointInCollision(pos):
        return False
    d = np.random.normal(1,4)
    pos_x=pos[0]
    pos_y=pos[1]
    alpha=random.uniform(0,360)*(math.pi/180) #get an random angle in rad
    pos2_x=d*math.cos(alpha)+pos_x
    pos2_y=d*math.sin(alpha)+pos_y
    pos2=[pos2_x,pos2_y]

    if not collChecker.pointInCollision(pos2):
        return False
        
    pos3_x=(pos_x+pos2_x)/2
    pos3_y=(pos_y+pos2_y)/2  
    pos3=[pos3_x,pos3_y]

    if collChecker.pointInCollision(pos3):
        return False
        
    return pos3


def Bridge_Sampeling(collChecker):
    
    limits = collChecker.getEnvironmentLimits()        
    pos = [random.uniform(limit[0],limit[1]) for limit in limits]
    
    
    #get a colliding point
    for t in range(0,10):
        while not collChecker.pointInCollision(pos):
            pos = [random.uniform(limit[0],limit[1]) for limit in limits]
        
        #store the x and y value of the colliding point
        pos_x=pos[0]
        pos_y=pos[1]
    
        
        for i in range(0,50):
        #get a distance over a gaussian distribution   
            me,sigma = 1,1 #Mean value of the gaussian distribution, standard deviation 
            d=np.random.normal(me,sigma)
            angle_list = [0,2*math.pi]
            for n in range(2):
                #alpha=random.uniform(0,360)*(180/math.pi) #get an random angle in ra
                templist = copy.deepcopy(angle_list)
                new_angles_list =[]
                verschoben = 1 #ja es gibt was hier zu sehen
                for idx in range(len(templist)-1):
                    new_angle = (templist[idx]+templist[idx+1])/2
                    new_angles_list.append(new_angle)
                    angle_list.insert(idx+verschoben,new_angle)
                    verschoben +=1
                for alpha in new_angles_list:    
                    pos2_x= d*math.cos(alpha)+pos_x
                    pos2_y= d*math.sin(alpha)+pos_y
                    pos2=[pos2_x,pos2_y]
                    if  collChecker.pointInCollision(pos2): #return point when in collision 
                        break
                pos3_x=(pos_x+pos2_x)/2
                pos3_y=(pos_y+pos2_y)/2  
                pos3=[pos3_x,pos3_y]   
                if collChecker.pointInCollision(pos3):
                    continue
                else:
                    return pos3
                    
    #return [1,2]
    #if no point found with gaussian pick a random collison free point
    """
    while  collChecker.pointInCollision(pos):
        pos = [random.uniform(limit[0],limit[1]) for limit in limits]
    return pos
    """
    return False

#-------------Gaussian Functions----------------
def Gaussian_sampling(collChecker):
    
    limits = collChecker.getEnvironmentLimits()        
    pos = [random.uniform(limit[0],limit[1]) for limit in limits]
    
    
    #get a colliding point
    for t in range(0,10):
        while not collChecker.pointInCollision(pos):
            pos = [random.uniform(limit[0],limit[1]) for limit in limits]
        
        #store the x and y value of the colliding point
        pos_x=pos[0]
        pos_y=pos[1]
    
        #find a non colliding point in a given distance to point a 
        for i in range(0,50):
        #get a distance over a gaussian distribution   
            me,sigma = 0,1 #Mean value of the gaussian distribution, standard deviation 
            d=np.random.normal(me,sigma)
            angle_list = [0,2*math.pi]
            for n in range(8):
                #alpha=random.uniform(0,360)*(180/math.pi) #get an random angle in ra
                templist = copy.deepcopy(angle_list)
                new_angles_list =[]
                verschoben = 1 #ja es gibt was hier zu sehen
                for idx in range(len(templist)-1):
                    new_angle = (templist[idx]+templist[idx+1])/2
                    new_angles_list.append(new_angle)
                    angle_list.insert(idx+verschoben,new_angle)
                    verschoben +=1
                for alpha in new_angles_list:    
                    pos2_x= d*math.cos(alpha)+pos_x
                    pos2_y= d*math.sin(alpha)+pos_y
                    pos2=[pos2_x,pos2_y]
                    if not collChecker.pointInCollision(pos2): #return point when collision free
                        return pos2
    
    return False
    #if no point found with gaussian pick a random collison free point
    """"
    while collChecker.pointInCollision(pos):
        pos = [random.uniform(limit[0],limit[1]) for limit in limits]
    return pos
    """

def simple_Gaus_Sampling(collChecker):
    
    
    #Get the limites for the graph 
    limits = collChecker.getEnvironmentLimits()   
    #Get a random position within the limits    
    pos = [random.uniform(limit[0],limit[1]) for limit in limits]
    #If selected Configuration is not free pick the Point 
    if not collChecker.pointInCollision(pos):
        return False 
    #get a distance for the second Point over a gaussian distribution 
    d=np.random.normal(1,1)
    pos_x=pos[0]
    pos_y=pos[1]
    #get a random angle between 0 and 360 
    alpha=random.uniform(0,360)*(math.pi/180)
    #calculate the new Point with the random angle and the selected distance d 
    pos2_x=d*math.cos(alpha)+pos_x
    pos2_y=d*math.sin(alpha)+pos_y
    #store the Point 
    pos2=[pos2_x,pos2_y]
    #check if the Point is collision free 
    if collChecker.pointInCollision(pos2):
        return False
    #if the point is  collision free return it 
    return pos2