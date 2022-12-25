#!/usr/bin/env python3
"""
@author: shubham
"""

import numpy as np
import math
import random



def grid(ar, offset=False,resolution=1): ############# resoultion is in cm
    
    ar= ar * 100/resolution
    ar_z= ar[:,2]
    if False:
        ar_x= ar[:,0]
        ar_y= ar[:,1]
        
    else:
        lim =500  ################################ lim is importatant it decides z projection height levels. Configure as required.
        ar_x= ar[:,0][(ar_z>-lim) &(ar_z<lim)]
        ar_y= ar[:,1][(ar_z>-lim) & (ar_z<lim)]
    
    
    ar_x=np.where(ar_x < 0, np.floor(ar_x), np.ceil(ar_x))
    ar_y=np.where(ar_y < 0, np.floor(ar_y), np.ceil(ar_y))  
    
    a= np.vstack((ar_x,ar_y)).T
    values, counts =np.unique(a, axis=0, return_counts=True)


    return values,counts

def query(pt,resolution=1): #############3 resoultion is in cm
    i,j,k=np.multiply(pt,100/resolution)
    i= math.ceil(i) if i>0 else math.floor(i)
    j= math.ceil(j) if j>0 else math.floor(j)
    k= math.ceil(k) if k>0 else math.floor(k)
    

    return(i,j,k)


if __name__ == "__main__":   
    #pcd = o3d.io.read_point_cloud("/home/shubham/Downloads/Advanced computational topics in robotics/HW_4_python/022_pointcloud.pcd")
    #ar=np.asarray(pcd.points)
    # pnt=random.choice(ar)
    # dic=grid(ar)
    
    # Nr=neighbour(np.array([0.135091, 0.301387, 0.113265]),0.01,dic)
    pass