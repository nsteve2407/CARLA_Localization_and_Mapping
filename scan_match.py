import open3d
import pypcd
import pcl
import random
import numpy as np

class ScanMatch :
    def __init__(self):
        self.map = None #Load HD Map
        self.ndt = None

    
    def scan_match_prev_scan(self,source,target,initial_guess):
        #Inputs:
        #Soucre: the pointcloud from the current time step
        # Target: PointCloud from previous time step

        # Output:
        # pose estimate in the form [x,y,theta]
        cloud_in = pcl.PointCloud() #the pointcloud from the current time step
        cloud_out = pcl.PointCloud()# Target: PointCloud from previous time step
        icp = cloud_in.make_IterativeClosestPoint()
        converged, transf, estimate, fitness = icp.icp(cloud_in, cloud_out)
        return 0
    
    def scan_match_map(self,source,initial_guess):
        #Inputs:
        #Soucre: the pointcloud from the current time step
        # Target: self.map cloud

        # Output:
        # pose estimate in the form [x,y,theta]


        return 0

