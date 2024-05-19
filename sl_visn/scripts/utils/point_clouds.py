import struct
import numpy as np
from math import isnan
from ctypes import * # convert float to uint32


datatype = {1:1, 2:1, 3:2, 4:2, 5:4, 6:4, 7:4, 8:8}

def read_point(point_2d, pc):
    """ get 3d point position from sensor_msgs point cloud """  
    arrayPosition = point_2d[0]*pc.row_step + point_2d[1]*pc.point_step # point_2d: y,x
    pos_x = arrayPosition + pc.fields[0].offset # X has an offset of 0
    len_x = datatype[pc.fields[0].datatype]
    pos_y = arrayPosition + pc.fields[1].offset # Y has an offset of 4
    len_y = datatype[pc.fields[1].datatype]
    pos_z = arrayPosition + pc.fields[2].offset # Z has an offset of 8
    len_z = datatype[pc.fields[2].datatype]

    try:
        x = struct.unpack('f', pc.data[pos_x: pos_x+len_x])[0] # read 4 bytes as a float number
        y = struct.unpack('f', pc.data[pos_y: pos_y+len_y])[0]
        z = struct.unpack('f', pc.data[pos_z: pos_z+len_z])[0]
        return [x,y,z]
    except:
        return None

def depth_pixel_to_metric_coordinate(point_2d, depth_image, camera_intrinsics):
    """ 
    [get 3D coordinate from depth image]
    Input: point 2d (list of doubles) (y and x value of the image coordinate), depth (double), 
        camera_intrinsics : The intrinsic values of the imager in whose coordinate system the depth_frame is computed
        |fx, 0,  ppx|
        |0,  fy, ppy|
        |0 , 0,  1  |
    Output: point 3d (x,y,z in meters) list of doubles
    """
    # [height, width] = depth_image.shape
    depth = depth_image[point_2d[0],point_2d[1]]/1000
    X = (point_2d[1] - camera_intrinsics[2])/camera_intrinsics[0]*depth
    Y = (point_2d[0] - camera_intrinsics[5])/camera_intrinsics[4]*depth
    return [X, Y, depth]

def depth_pixels_to_metric_coordinates(points_2d, depth_image, camera_intrinsics):
    """ 
    [get 3D coordinate from depth image]
    Input: point 2d (list of doubles) (y and x value of the image coordinate), depth (double), 
        camera_intrinsics : The intrinsic values of the imager in whose coordinate system the depth_frame is computed
        |fx, 0,  ppx|
        |0,  fy, ppy|
        |0 , 0,  1  |
    Output: point 3d (x,y,z in meters) list of doubles
    """
    # [height, width] = depth_image.shape
    depth = depth_image[points_2d[:,0],points_2d[:,1]]/1000
    X = (points_2d[:,1] - camera_intrinsics[2])/camera_intrinsics[0]*depth
    Y = (points_2d[:,0] - camera_intrinsics[5])/camera_intrinsics[4]*depth
    return np.transpose([X, Y, depth])

def depth_pixels_to_metric_coordinate(points_2d, depth_image, camera_intrinsics, mode='mean'):
    """ 
    [get 3D coordinate from depth image]
    Input: point 2d (list of doubles) (y and x value of the image coordinate), depth (double), 
        camera_intrinsics : The intrinsic values of the imager in whose coordinate system the depth_frame is computed
        |fx, 0,  ppx|
        |0,  fy, ppy|
        |0 , 0,  1  |
    Output: point 3d (x,y,z in meters) list of doubles
    """
    # [height, width] = depth_image.shape
    point_2d = np.mean(points_2d, axis=0)
    depths = depth_image[points_2d[:,0],points_2d[:,1]].flatten()/1000
    valid_ids = np.argwhere((depths!=np.nan) & (depths!=0))
    # valid_ids = ((not np.isnan(depths))& (depths!=0)).nonzero()[0]
    depths = depths[valid_ids].squeeze()
    if mode=='mean':
        depth = np.mean(depths)
    elif mode=='middle':
        depth = depths[depths.argsort()][depths.size//2]
    else:
        print('Unknown Mode! Returning mean instead!')
        depth = depths.mean(axis=0)

    X = (point_2d[1] - camera_intrinsics[2])/camera_intrinsics[0]*depth
    Y = (point_2d[0] - camera_intrinsics[5])/camera_intrinsics[4]*depth
    return np.transpose([X, Y, depth])

def metric_coordinate_to_pixel_coordinate(point_3d, camera_intrinsics):
    '''
    Input: 
    - point 3d (x,y,z in meters) list of doubles
    Output:
    - point 2d: y,x
    '''
    [X, Y, depth] = point_3d
    x = X/depth*camera_intrinsics[0] + camera_intrinsics[2]
    y = Y/depth*camera_intrinsics[4] + camera_intrinsics[5]
    return np.array([int(y),int(x)])

def read_point_from_region(point_2d, pc, region, mode='mean', camera_intrinsics=None):
    """ 
    [get a middle 3d point position from a region within point cloud]
    Input: point 2d (row, col), point cloud message, rectangular edge length
    Output: point 3d (x,y,z)
    """
    # assert all(point_2d), '2D target should not be on the edges!'
    point_2d = np.array(point_2d, dtype=np.int32)
    row, col = np.indices((region, region))-region//2
    grid_ids = np.transpose([row.flatten()+point_2d[0], col.flatten()+point_2d[1]])
    if camera_intrinsics is None:
        points_3d = []
        for id in grid_ids:
            if camera_intrinsics is None:
                point_3d = read_point(id, pc)
                if not isnan(point_3d[0]): points_3d.append(point_3d)
        # sort the positions according to the y values
        if len(points_3d)>0:
            points_3d = np.array(points_3d)
            if mode=='mean':
                return points_3d.mean(axis=0)
            elif mode=='middle':
                return points_3d[points_3d[:,1].argsort()][points_3d.shape[0]//2]
            else:
                print('Unknown Mode! Returning mean instead!')
                return points_3d.mean(axis=0)
    else:
        return depth_pixels_to_metric_coordinate(grid_ids, pc, camera_intrinsics, mode=mode)

def read_points_from_region(point_2d_list, pc, region, mode='mean', camera_intrinsics=None):
    """ get 3d point positions from a region within point cloud """
    points_3d = []
    for point_2d in point_2d_list:
        points_3d.append(read_point_from_region(point_2d, pc, region=region, mode=mode, camera_intrinsics=camera_intrinsics))
    return np.array(points_3d)

def xy_to_yx(points):
    return np.flip(points, axis=-1)

def euclidian_distance(point1, point2):
    return sum([(point1[x] - point2[x]) ** 2 for x in range(len(point1))]) ** 0.5

def is_sorted(a):
    ''' check if a list is in ascending order '''
    return np.all(a[:-1] <= a[1:])