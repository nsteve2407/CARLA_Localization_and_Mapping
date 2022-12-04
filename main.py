#!/usr/bin/env python3

import glob
import os
import sys
import argparse
import time
from datetime import datetime
import random
import numpy as np
from matplotlib import cm
import open3d as o3d
import pcl
from copy import deepcopy
import scipy.spatial as sp
import  matplotlib.pyplot as plt

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

#Localization Functions
from EKF import EKF
from scan_match import ScanMatch

# Other
import pandas as pd

VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])


def lidar_sensor_callback(point_cloud, point_list,pclCloud):
    # Prepares the lidar data for Scan Matching
    # PCD for scan matching is stored into global variables
    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
    data = np.reshape(data, (int(data.shape[0] / 4), 4))

    # Isolate the intensity and compute a color for it
    intensity = data[:, -1]
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
    int_color = np.c_[
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]

    # Isolate the 3D data
    points = data[:, :-1]

    # We're negating the y to correclty visualize a world that matches
    # what we see in Unreal since Open3D uses a right-handed coordinate system
    points[:, :1] = -points[:, :1]
    # pclCloud.from_array(points)
    # # An example of converting points from sensor to vehicle space if we had
    # # a carla.Transform variable named "tran":
    # points = np.append(points, np.ones((points.shape[0], 1)), axis=1)
    # points = np.dot(tran.get_matrix(), points.T).T
    # points = points[:, :-1]

    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)


def generate_lidar_bp(arg, world, blueprint_library, delta):
    #Generates Carla lidar bluprint with noise
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('noise_stddev', '0.2')

    lidar_bp.set_attribute('upper_fov', str(arg.upper_fov))
    lidar_bp.set_attribute('lower_fov', str(arg.lower_fov))
    lidar_bp.set_attribute('channels', str(arg.channels))
    lidar_bp.set_attribute('range', str(arg.range))
    lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))
    lidar_bp.set_attribute('points_per_second', str(arg.points_per_second))
    return lidar_bp



def main(arg):
# Main function to interact with simulator
    estimate = True # Run estimator
    viz = False
    estimate_log = []
    gt_log = []

    client = carla.Client(arg.host, arg.port)
    client.set_timeout(100.0)
    world = client.get_world()
    world = client.reload_world()

    try:
        # Initialize world, vehicle, lidar and Open3D visualizer
        original_settings = world.get_settings()
        settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)

        delta = 0.05

        settings.fixed_delta_seconds = delta
        settings.synchronous_mode = True
        settings.no_rendering_mode = arg.no_rendering
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter(arg.filter)[0]
        vehicle_transform = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)
        vehicle.set_autopilot(arg.no_autopilot)

        lidar_bp = generate_lidar_bp(arg, world, blueprint_library, delta)

        user_offset = carla.Location(arg.x, arg.y, arg.z)
        lidar_transform = carla.Transform(carla.Location(x=-0.5, z=1.8) + user_offset)

        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)

        point_list = o3d.geometry.PointCloud()
        pclCloud = pcl.PointCloud()

        lidar.listen(lambda data: lidar_sensor_callback(data, point_list,pclCloud))
        if viz:
            vis = o3d.visualization.Visualizer()
            vis.create_window(
                window_name='Carla Lidar',
                width=960,
                height=540,
                left=480,
                top=270)
            vis.get_render_option().background_color = [0.05, 0.05, 0.05]
            vis.get_render_option().point_size = 1
            vis.get_render_option().show_coordinate_frame = True
            fig = plt.figure(figsize=(12,9))
            plt.subplot(141)
            plt.ylabel('Position X')
            plt.xlabel('Frame #')
            plt.subplot(142)
            plt.ylabel('Position Y')
            plt.xlabel('Frame #')
            plt.subplot(143)
            plt.ylabel('Theta')
            plt.xlabel('Frame #')
            plt.subplot(144)
            plt.ylabel('Y')
            plt.xlabel('X')
            plt.pause(0.05)

        ## Initialize EKF and ScanMatch objects
        estimator = EKF()
        scan_match = ScanMatch()

        frame = 0
        dt0 = datetime.now()
        while True:
            if estimate:
                if  np.asarray(scan_match.prev_cloud.points).size==0:
                    tf = lidar.get_transform()
                    location = tf.location
                    rotation = tf.rotation
                    estimator.x = np.array([[location.x,location.y,np.deg2rad(rotation.yaw),0.5,0.5,0.5]]).T
                    estimator.initialized = True
                    scan_match.prev_cloud = deepcopy(point_list)
                    print("\nInitial State Initialized")
                else:
                    tf = lidar.get_transform()
                    location = tf.location
                    rotation = tf.rotation
                    # Predict
                    prev_state = deepcopy(estimator.x)
                    estimator.predict(delta) #delta set to 0.005 secs

                    #Update
                    # Use Lidar and gps on alternate frames

                    if frame%5==0:
                        est_delta_pose = estimator.x - prev_state
                        init_guess = np.eye(4,dtype=np.float32)
                        init_guess[:3,:3] = sp.transform.Rotation.from_euler('z',est_delta_pose[2]).as_matrix()
                        init_guess[0,3] = est_delta_pose[0]
                        init_guess[1,3] = est_delta_pose[1]
                        init_guess[2,3] = location.z
                        delta_pose = scan_match.scan_match_prev_scan(point_list)
                        lidar_gloal_pose = np.asarray([prev_state[0]+delta_pose[0,0]*np.cos(prev_state[2]),prev_state[1]+delta_pose[1,0]*np.sin(prev_state[2]),prev_state[2]+delta_pose[2,0]])
                        estimator.measurement_update_lidar(lidar_gloal_pose)
                        scan_match.prev_cloud = deepcopy(point_list)
                    else:
                        #Use gps
                        noise_x = np.random.normal(0,2,1)
                        noise_y = np.random.normal(0,2,1)
                        noise_theta = np.random.normal(0,0.3,1)

                        estimator.measurement_update_gps(np.array([location.x+noise_x,location.y+noise_y,np.deg2rad(rotation.yaw)+noise_theta])) # Add white noise
                        scan_match.prev_cloud = deepcopy(point_list)
                    if viz:
                        plt.subplot(141)
                        # plt.ylim(location.x-15,location.x+15)
                        plt.scatter(frame,estimator.x[0],c='b')
                        plt.scatter(frame,location.x,c='r')
                        plt.subplot(142)
                        # plt.ylim(location.y-15,location.y+15)
                        plt.scatter(frame,estimator.x[1],c='b')
                        plt.scatter(frame,location.y,c='r')
                        plt.subplot(143)
                        # plt.ylim(np.deg2rad(rotation.yaw)-0.25,np.deg2rad(rotation.yaw)+0.25)
                        plt.scatter(frame,estimator.x[2],c='b')
                        plt.scatter(frame,np.deg2rad(rotation.yaw),c='r')
                        plt.pause(0.05)
                        plt.subplot(144)
                        plt.scatter(estimator.x[0],estimator.x[1],c='b')
                        plt.scatter(location.x,location.y,c='r')
                        plt.pause(0.08)
                        # print("\nState estimate:")
                        # print("\nx:{} , y:{} ,theta:{}".format(estimator.x[0],estimator.x[1],estimator.x[2]))
                        # print("\nGround Truth:")
                        # print("\nx:{} , y:{} ,theta:{}".format(location.x,location.y,np.deg2rad(rotation.yaw)))
                    # Log estimate and GT
                    estimate_log.append(estimator.x.T.tolist()[0]+[location.x,location.y,np.deg2rad(rotation.yaw)])
                    # gt_log.append([location.x,location.y,rotation.yaw])


            # Update Visualizer
            if viz:
                if frame == 2:
                    vis.add_geometry(point_list)
                vis.update_geometry(point_list)

                vis.poll_events()
                vis.update_renderer()
            time.sleep(0.02)
            world.tick()

            process_time = datetime.now() - dt0
            sys.stdout.write('\r' + 'FPS: ' + str(1.0 / process_time.total_seconds()))
            sys.stdout.flush()
            dt0 = datetime.now()
            frame += 1
            if frame%200==0:
                df = pd.DataFrame(estimate_log,columns=['x_e','y_e','th_e','xd_e','yd_e','thd_e','x_gt','y_gt','th_gt'])
                df.to_csv("./log3.csv")
            if frame>3000:
                break

    finally:
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)

        vehicle.destroy()
        lidar.destroy()
        # vis.destroy_window()
        df = pd.DataFrame(estimate_log,columns=['x_e','y_e','th_e','xd_e','yd_e','thd_e','x_gt','y_gt','th_gt'])
        df.to_csv("./log4.csv")


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host CARLA Simulator (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port of CARLA Simulator (default: 2000)')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        help='use the no-rendering mode which will provide some extra'
        ' performance but you will lose the articulated objects in the'
        ' lidar, such as pedestrians')
    argparser.add_argument(
        '--semantic',
        action='store_true',
        help='use the semantic lidar instead, which provides ground truth'
        ' information')
    argparser.add_argument(
        '--no-noise',
        action='store_true',
        help='remove the drop off and noise from the normal (non-semantic) lidar')
    argparser.add_argument(
        '--no-autopilot',
        action='store_false',
        help='disables the autopilot so the vehicle will remain stopped')
    argparser.add_argument(
        '--show-axis',
        action='store_true',
        help='show the cartesian coordinates axis')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='model3',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--upper-fov',
        default=15.0,
        type=float,
        help='lidar\'s upper field of view in degrees (default: 15.0)')
    argparser.add_argument(
        '--lower-fov',
        default=-25.0,
        type=float,
        help='lidar\'s lower field of view in degrees (default: -25.0)')
    argparser.add_argument(
        '--channels',
        default=64.0,
        type=float,
        help='lidar\'s channel count (default: 64)')
    argparser.add_argument(
        '--range',
        default=100.0,
        type=float,
        help='lidar\'s maximum range in meters (default: 100.0)')
    argparser.add_argument(
        '--points-per-second',
        default=500000,
        type=int,
        help='lidar\'s points per second (default: 500000)')
    argparser.add_argument(
        '-x',
        default=0.0,
        type=float,
        help='offset in the sensor position in the X-axis in meters (default: 0.0)')
    argparser.add_argument(
        '-y',
        default=0.0,
        type=float,
        help='offset in the sensor position in the Y-axis in meters (default: 0.0)')
    argparser.add_argument(
        '-z',
        default=0.0,
        type=float,
        help='offset in the sensor position in the Z-axis in meters (default: 0.0)')
    args = argparser.parse_args()

    try:
        main(args)
    except KeyboardInterrupt:
        print(' - Exited by user.')
