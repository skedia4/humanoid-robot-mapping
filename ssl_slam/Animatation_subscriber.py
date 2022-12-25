#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 21 01:45:53 2022

@author: shubham
"""

import rospy
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PointStamped
import ros_numpy
import numpy as np
from time import time
from matplotlib import pyplot as plt
import matplotlib.animation as animation


class EDT(object):
    def __init__(self):
        self.rate = rospy.Rate(5)
        self.edt_sub = rospy.Subscriber("/EDT", Image, self.edt_callback)
        self.pospix_sub  = rospy.Subscriber("/pospix", PointStamped, self.pospix_callback) 
        self.pos_pixel = [0,0]
        #self.rawpcd =  rospy.Subscriber("/velodyne_points_filtered", PointCloud2, self.raw_pcd_data_callback)
     
        self.edt = None
        
    def edt_callback(self, msg):
         
         pc = ros_numpy.numpify(msg)
         #points=np.zeros((pc.shape[0],3))
         # points[:,0]=pc['x']
         # points[:,1]=pc['y']
         # points[:,2]=pc['z']
         #p = pcl.PointCloud(np.array(points, dtype=np.float32))
    
         self.edt=pc#.astype(dtype= np.float32)
         
         
    def pospix_callback(self, msg):
         
        self.pos_pixel[1] = msg.point.x 
        self.pos_pixel[0] = msg.point.y 
         
         
    def start_cc(self):
        
        while not rospy.is_shutdown():
            if self.edt is not None:
                
               #print (self.edt.shape)
               ######################### Maybe Operations: currently plotting     
               plt.imshow(self.edt, interpolation='nearest')
               plt.show(block=False)
               plt.pause(0.0001)
            else:
                print("no data recieved")
            self.rate.sleep()
            
        
def data_edt_start():

    rospy.init_node('Py_Animation_node', anonymous=False)
    cc = EDT()

    try:
        cc.start_cc()  
    except rospy.ROSInterruptException:
        pass
  

   

def vizualization():
    rospy.init_node('Py_Animation_node', anonymous=False)
    cc = EDT()
    x = []; y =[]
    fig, ax = plt.subplots()
    #print(cc.edt)
    def updatefig(i):   ########### Iterable function with i
        
        try:
            while cc.edt is None:
                print("No data recieved")
                cc.rate.sleep()
              
            
            #### Random number now need to change with the pixel locations
            #temp=np.random.randint(1000, size=(2)) 
            temp = cc.pos_pixel
            
            ax.clear()
            ax.imshow(cc.edt, animated=True, interpolation='nearest', cmap='gray')
            
            #x.append(temp[0]); y.append(temp[1])   #########for whole trajectory
            #ax.plot(x, y) #########for whole trajectory
            ax.plot(temp[0],temp[1], '+r')
            
        except rospy.ROSInterruptException:
            pass
    

    ani = animation.FuncAnimation(fig, updatefig, interval=50, blit=False)
    plt.show()


if __name__ == '__main__':
   #data_edt_start() 
   vizualization()
   
   
   
   
   