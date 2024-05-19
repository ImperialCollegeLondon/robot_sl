import numpy as np
from math import pi
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, decompose_matrix, compose_matrix, quaternion_matrix, quaternion_multiply, euler_matrix

def tf_mat2ls(mat):
    _, _, euler, trans, _ = decompose_matrix(mat)
    return np.concatenate((trans, euler))

def tf_ls2mat(list):
    if len(list) == 6:
        return compose_matrix(translate=list[:3], angles=list[3:])
    elif len(list) == 7:
        return compose_matrix(translate=list[:3], angles=euler_from_quaternion(list[3:]))
    else:
        print(f'Wrong input dimension! Got a list with {len(list)} elements.')

def pose_msg_to_list(pose):
    return [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

def list_to_point_msg(list):
    return Point(*list[:3])

def list_to_pose_msg(list):
    p = Pose()
    p.position = Point(*list[:3])
    if len(list) == 6:
        p.orientation = Quaternion(*quaternion_from_euler(*list[3:]))
    elif len(list) == 7:
        p.orientation = Quaternion(*list[3:])
    else:
        print(f'Wrong input dimension! Got list with {len(list)} elements.')
        return
    return p

def mat_to_pose_msg(mat):
    return list_to_pose_msg(tf_mat2ls(mat))

def pose_msg_to_posestamped(pose_msg, frame_id):
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose = pose_msg
    return ps

def euler_mul(euler_b, euler_a):
    ''' euler multiplication. Apply rotation a first. '''
    return euler_from_quaternion(quaternion_multiply(quaternion_from_euler(*euler_a), quaternion_from_euler(*euler_b)))

# def concat(a, b):
#     ''' concatenate two 1d lists '''
#     return [*a, *b]

def eval_list(l):
    ''' evaluate a list of strings '''
    return [eval(i) if type(i) == str else i for i in l]

def ls_concat(a, b):
    ''' concatenate two 1d lists '''
    return [*a, *b]

def ls_add(l_a, l_b):
    ''' add two 1d lists together '''
    assert len(l_a) == len(l_b), f'Input lists have different lengths of {len(l_a)} and {len(l_b)}!'
    return [a+b for a,b in zip(l_a, l_b)]
    
def is_sorted(a):
    ''' check if a list is in ascending order '''
    return np.all(a[:-1] <= a[1:])

def position_projection(point_a, rotation, distance):
    '''
    Compute the point position given original point position, rotation and distance
    Rotation defined with respect to Z axis
    '''
    if len(rotation) == 3:
        rot_mat = euler_matrix(rotation[0], rotation[1], rotation[2], 'sxyz')[:3, :3]
    elif len(rotation) == 4:
        rot_mat = quaternion_matrix(rotation)[:3, :3]
    else:
        print('Undefined rotation format.')
    point_b = np.matmul(rot_mat, [0, 0, distance])
    return [a+b for a,b in zip(point_a, point_b)]


def calc_gripper_position(target, rotation, distance):
    '''
    Compute the gripper position given the target position, gripper rotation and distance
    '''
    if len(rotation) == 3:
        euler = rotation
    elif len(rotation) == 4:
        euler = euler_from_quaternion(rotation)
    else:
        print('Undefined rotation format.')
    euler = [euler[0]-pi, euler[1], euler[2]] # to the euler from the target's perspective
    return position_projection(target, euler, distance)