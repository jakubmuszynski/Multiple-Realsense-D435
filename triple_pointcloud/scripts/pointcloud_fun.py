import time
import cv2
import numpy as np
import pyrealsense2 as rs
import rospy
import struct
import open3d


from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header

def get_point_cloud(depth_frame, color_frame, pc, decimate, colorizer):
    depth_frame = decimate.process(depth_frame)

    # Grab new intrinsics (may be changed by decimation)
    depth_intrinsics = rs.video_stream_profile(
        depth_frame.profile).get_intrinsics()
    w, h = depth_intrinsics.width, depth_intrinsics.height

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    depth_colormap = np.asanyarray(
        colorizer.colorize(depth_frame).get_data())

    mapped_frame, color_source = color_frame, color_image

    points = pc.calculate(depth_frame)
    pc.map_to(mapped_frame)

    # Pointcloud data to arrays
    v, t = points.get_vertices(), points.get_texture_coordinates()
    verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
    texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

    points = np.reshape(verts, (76800, 3))
    #points_2 = np.array([[0, 0, 0]])

    points[:, [1, 2]] = points[:, [2, 1]] 
    points[:, [2]] *= -1.0

    return verts, texcoords, points

def point_cloud_filtration(points, voxel_size_arg):

    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(points)
    downpcd = pcd.voxel_down_sample(voxel_size=voxel_size_arg)
    #downpcd = open3d.geometry.voxel_down_sample(pcd, voxel_size=voxel_size_arg)

    new_points = np.asarray(downpcd.points)    

    return new_points


def transform_point_cloud(points, position, orientation):

    c = np.cos(orientation[0] * 3.14/180)
    s = np.sin(orientation[0] * 3.14/180)

    rotation_x = np.array([[1, 0, 0],
                           [0, c,-s],
                           [0, s, c]])

    c = np.cos(orientation[1] * 3.14/180)
    s = np.sin(orientation[1] * 3.14/180)

    rotation_y = np.array([[c, 0,-s],
                           [0, 1, 0],
                           [s, 0, c]])

    c = np.cos(orientation[2] * 3.14/180)
    s = np.sin(orientation[2] * 3.14/180)

    rotation_z = np.array([[c,-s, 0],
                           [s, c, 0],
                           [0, 0, 1]])

    matrix = rotation_z @ rotation_y @ rotation_x

    new_points = np.dot(points, matrix.T)

    new_points[:, [0]] += position[0]
    new_points[:, [1]] += position[1]
    new_points[:, [2]] += position[2]   

    return new_points

def create_PointCloud2(new_points):

    x = new_points[:, 0]
    y = new_points[:, 1]
    z = new_points[:, 2]

    points2 = list([x[i], y[i], z[i], 
                    struct.unpack('I', struct.pack('BBBB',
                           200,
                           200,  
                           200,
                           255))[0]] for i in range(len(x)))
        
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),              
              PointField('rgba', 12, PointField.UINT32, 1),
              ]
  
    header = Header()
    header.frame_id = "map"
    pc2 = point_cloud2.create_cloud(header, fields, points2)
    pc2.header.stamp = rospy.Time.now()

    return pc2
