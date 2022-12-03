import open3d as o3d
import random
import numpy as np
import scipy.spatial as sp
import copy

class ScanMatch :
    def __init__(self):
        self.map = None #Load HD Map
        self.ndt = None
        self.prev_cloud = o3d.geometry.PointCloud()

    def preprocess_point_cloud(self,pcd, voxel_size):
        pcd_down = o3d.geometry.PointCloud.voxel_down_sample(pcd, voxel_size)

        radius_normal = voxel_size * 2
        o3d.geometry.PointCloud.estimate_normals(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = voxel_size * 5
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh

    def execute_global_registration(self,source_down, target_down, source_fpfh,
                                    target_fpfh, voxel_size):
        distance_threshold = voxel_size * 1.5
        # print(":: RANSAC registration on downsampled point clouds.")
        # print("   Since the downsampling voxel size is %.3f," % voxel_size)
        # print("   we use a liberal distance threshold %.3f." % distance_threshold)
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,False, distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 4, [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.pipelines.registration.RANSACConvergenceCriteria(40000, 500))
        return result

    def refine_registration(self,source, target, voxel_size,result_ransac):
        distance_threshold = voxel_size * 0.4
        # print(":: Point-to-plane ICP registration is applied on original point")
        # print("   clouds to refine the alignment. This time we use a strict")
        # print("   distance threshold %.3f." % distance_threshold)
        result = o3d.pipelines.registration.registration_icp(
            source, target, distance_threshold, result_ransac.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        return result
    
    def scan_match_prev_scan(self,source):
        #Inputs:
        #Soucre: the pointcloud from the current time step
        # Target: PointCloud from previous time step

        # Output:
        # pose estimate in the form [x,y,theta]
        # Target: PointCloud from previous time step
        # trans_init =     trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
        #                      [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        # source.transform(trans_init)
        source_down, source_fpfh = self.preprocess_point_cloud(source,3.0)
        target_down,targer_fpfh = self.preprocess_point_cloud(self.prev_cloud,3.0)

        result_ransac = self.execute_global_registration(source_down,target_down,source_fpfh,targer_fpfh,1.0)

        reg = self.refine_registration(source,self.prev_cloud,1.5,result_ransac)

        # reg = o3d.pipelines.registration.registration_icp(source,self.prev_cloud,0.02,init_guess,o3d.pipelines.registration.TransformationEstimationPointToPoint())
        angles  = sp.transform.Rotation.from_matrix(copy.copy(reg.transformation[:3,:3])).as_euler('zxy')
        traslation = copy.copy(reg.transformation[:3,3].T)
        return np.array([[traslation[0]],[traslation[1]],[angles[0]]])
    
    def scan_match_map(self,source,initial_guess):
        #Inputs:
        #Soucre: the pointcloud from the current time step
        # Target: self.map cloud

        # Output:
        # pose estimate in the form [x,y,theta]


        return 0

