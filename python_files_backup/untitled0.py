#!/usr/bin/env python3
"""
Created on Sun Feb  6 15:42:57 2022

@author: shubham
"""

import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import ros_numpy
import numpy as np
from time import time
from matplotlib import pyplot as plt
from pcd_to_EDT import *
from scipy.spatial.transform import Rotation as R


class Map(object):
    def __init__(self):
        self.rate = rospy.Rate(2)
        self.map_sub = rospy.Subscriber("/map", PointCloud2, self.map_callback)
        #self.rawpcd =  rospy.Subscriber("/velodyne_points_filtered", PointCloud2, self.raw_pcd_data_callback)
        self.local_sub = rospy.Subscriber("/odom", Odometry, self.odo_callback)
        self.rawpcd =  rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.raw_pcd_data_callback)
        
        
        self.cc= pcd_to_EDT()
        self.pos =None
        self.pos_pixel = None
        self.pos_pixel_all = []
        self.orientation = None
        self.map=None
        self.map_from_raw_pcd =None #np.array([[0,0,0],[0,0,0]])
        self.prev_count=0
        self.pcd = o3d.geometry.PointCloud()
        self.edt=None
        self.figure =0
        
        #self.pub = rospy.Publisher('EDT', PointCloud2, queue_size=10)
        self.pub_1 = rospy.Publisher('EDT', Image, queue_size=10)
        
        self.pub_2 = rospy.Publisher('/pospix', PointStamped, queue_size=1)

        self.pub_3 = rospy.Publisher("/pose", PoseWithCovarianceStamped, queue_size=2)
        
        self.edt_time_wind = np.zeros((100,1))
        

        
    def odo_callback(self, pose_msg):
        
        x    =  pose_msg.pose.pose.position.x  # x-coordinate
        y     = pose_msg.pose.pose.position.y # y-coordinate
        z     = pose_msg.pose.pose.position.z # z-coordinate
        self.pos = [x, y, z]
        pos_pixel= np.array(self.pos)* 100/(self.cc.resolution) + np.array([self.cc.offset_x,self.cc.offset_y, 0])
        pos_pixel= pos_pixel[0:2]
        self.pos_pixel = pos_pixel.astype(int)
        
        orientation_q = pose_msg.pose.pose.orientation
        self.orientation = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # pose = PoseWithCovarianceStamped()
        # pose.header.stamp = rospy.Time(0)
        # pose.header.frame_id = None
        # pose.pose.pose.position.x = x
        # pose.pose.pose.position.y = y
        # pose.pose.pose.position.z = z
        # pose.pose.pose.orientation.x = orientation_q.x
        # pose.pose.pose.orientation.y = orientation_q.y
        # pose.pose.pose.orientation.z = orientation_q.z
        # pose.pose.pose.orientation.w = orientation_q.w

        # pose.pose.covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0]
        # self.pub_3.publish(pose)
        header = pose_msg.header
        pose_w_c = pose_msg.pose
        msg = PoseWithCovarianceStamped()
        msg.header = header
        msg.pose = pose_w_c
        self.pub_3.publish(msg)



        
    def map_callback(self, grid_map):
        pc = ros_numpy.numpify(grid_map)
        points=np.zeros((pc.shape[0],3))
        points[:,0]=pc['x']
        points[:,1]=pc['y']
        points[:,2]=pc['z']
        #p = pcl.PointCloud(np.array(points, dtype=np.float32))
        
        self.map=points
        
        
    def raw_pcd_data_callback(self, data):
        pc = ros_numpy.numpify(data)
        points=np.zeros((pc.shape[0],3))
        
        ########################### ORiginal works for Velodyne filtered
        # points[:,0]=pc['x']
        # points[:,1]=pc['y']
        # points[:,2]=pc['z']
        
        ###########################Cheaged for raw point cloud data
        points[:,0]=pc['z']
        points[:,1]=-pc['x']
        points[:,2]=-pc['y']
        
        
        ##########################################################
        if self.pos is not None:
            points_tr=  points @ (self.rotation_matrix()).T  + np.array(self.pos)
        else:
            points_tr=  points 
        
        #self.map_from_raw_pcd= np.append(self.map_from_raw_pcd, points_tr, axis =0 )  
        self.map_from_raw_pcd= points_tr
        
        #print("HIIIIIIIIIIIIIIIIIIIIIII", self.map_from_raw_pcd.shape)

        
        
    def rotation_matrix(self):
    
    # Extract the values from Q
        #print (self.orientation)
        
        if self.orientation is not None:
            ########################################################################################
            # [q0, q1, q2, q3]= self.orientation
            # # First row of the rotation matrix
            # r00 = -2 * (q2 * q2 + q3 * q3) + 1
            # r01 = 2 * (q1 * q2 - q0 * q3)
            # r02 = 2 * (q1 * q3 + q0 * q2)
             
            # # Second row of the rotation matrix
            # r10 = 2 * (q1 * q2 + q0 * q3)
            # r11 = -2 * (q1 * q1 + q3 * q3)  +1
            # r12 = 2 * (q2 * q3 - q0 * q1)
             
            # # Third row of the rotation matrix
            # r20 = 2 * (q1 * q3 - q0 * q2)
            # r21 = 2 * (q2 * q3 + q0 * q1)
            # r22 = -2 * (q1 * q1 + q2 * q2) +1 
             
            # # 3x3 rotation matrix
            # rot_matrix = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
            ######################################################################################
            
            
            ############################################################################################
            rot_matrix = R.from_quat(self.orientation)
            rot_matrix =rot_matrix.as_matrix()
            return rot_matrix
        
        else:
            return np.zeros((3,3))
        #print (rot_matrix)                
        
        
        

    def start_cc(self):
        
        while not rospy.is_shutdown():
            if self.map is not None:
                
                if int(self.map.shape[0]) != int(self.prev_count):
                    current_pcd= self.map[self.prev_count:-1]
                    
                    ################## operations
                    start_1 = time()
                    self.cc.occupancy_grid(current_pcd)
                    start_2 = time()
                    
                    self.edt= self.cc.EDT(self.pos_pixel) ## self.pos_pixel
                    stop = time()
                    print ("Current postion pixel", self.pos_pixel)
                    print("Time Occupancy", start_2-start_1)
                    print("Time EDT ", stop-start_2)
                    
                    self.prev_count= self.map.shape[0]
                    self.figure += 1
                    plt.imshow(self.cc.data, cmap = 'gray', interpolation='nearest')
                    plt.savefig("./Figures/Figure_OCG_origin"+str(self.figure)+".png")
                    
                    
            else:
                print("no map recieved")
        
            self.rate.sleep()




    def start_cc_raw_pcd(self):
        while not rospy.is_shutdown():
            if self.map_from_raw_pcd is not None:

                ################## operations
                start_1 = time()
                self.cc.occupancy_grid(self.map_from_raw_pcd)
                #self.pcd.points = o3d.utility.Vector3dVector(self.map_from_raw_pcd)
                #o3d.io.write_point_cloud("Point_cld_data.ply", self.pcd)
                
                
                ########################################################### Time complexity EDT empyrically
                # temp =[]
                # for i in range (100):
                #     start_3 = time()
                #     self.edt= self.cc.EDT(self.pos_pixel,400-i*5)
                #     stop = time()
                #     temp.append(stop-start_3)
                #     #print("Time EDT _window " + str(1000-i*100), stop-start_3)
                    
                # self.edt_time_wind += np.array(temp).reshape(100,1)
                # print("Time EDT _window  ", self.edt_time_wind/(self.figure+1))
                ###########################################################
                    
                start_2 = time()
                #self.edt= self.cc.EDT(self.pos_pixel) ## self.pos_pixel
                self.edt= self.cc.EDT()
                ########################################################### Publishing as pointcloud data
                # data = self.edt.astype(dtype=[ ('x', np.float32), ('y', np.float32)])
                # msg = ros_numpy.msgify(PointCloud2, data)
                # self.pub_1.publish(msg)
                #############################################################
                
                ########################################################### Publishing as Image data
                data = self.edt.astype(dtype=[ ('x', np.float32), ('y', np.float32)])
                msg = ros_numpy.msgify(Image, self.cc.data.astype(dtype= np.uint16), "mono16")
                #msg = ros_numpy.msgify(Image, self.edt.astype(dtype= np.uint16), "mono16")
                self.pub_1.publish(msg)
                
                
                point = PointStamped()
                point.point.x = self.pos_pixel[0]
                point.point.y = self.pos_pixel[1]
                self.pub_2.publish(point)
                
                #############################################################               
                
                #print ("Current postion pixel", self.pos_pixel)
                print("Time Occupancy", start_2-start_1)
                print (self.edt.shape)
                #print(type(self.edt))
                stop = time()
                self.figure += 1
                #plt.imshow(self.cc.data, cmap = 'gray', interpolation='nearest')
                #plt.imshow(self.cc.colorify(),  interpolation='nearest')
                #plt.imshow(self.edt, interpolation='nearest')
                #plt.savefig("./Figures/Figure_dynamic_OCG"+str(self.figure)+".png")
                #plt.show(block=False)
                #plt.pause(0.001)
                
                outfile = 'edt_occ.npz'
                self.pos_pixel_all +=[self.pos_pixel]
                #np.savez(outfile, self.cc.data, self.edt, self.cc.offset_x, self.cc.offset_y, self.pos_pixel_all)
                
                print("Time EDT ", stop-start_2)
            else:
                print("no map recieved")
        
            self.rate.sleep()
        


def map_start():

    rospy.init_node('Py_map_node', anonymous=False)
    cc = Map()

    try:
        ######## From The processed map
        #cc.start_cc()
        
        ######## From the raw point cloud data
        cc.start_cc_raw_pcd()
        

    except rospy.ROSInterruptException:
        pass
    
    plt.imshow(cc.edt, interpolation='nearest')


if __name__ == '__main__':
   map_start() 
