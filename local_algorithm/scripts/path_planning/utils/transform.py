import math 
import numpy as np

def quat_to_eul(quat):
    """ Convert a quaternion to corresponding euler angles

    Args:
        quat (array-like): quaternion (x,y,z,w)

    Returns:
        tuple: euler angle along (x,y,z) axises, always 3D
    """
    # TODO: implement function if necessary   
    pass

def eul_to_quat(eul):
    """ Convert 3d euler angles to a quaternion

    Args:
        eul (array-like): euler angles along (x,y,z) axises

    Returns:
        tuple: quaternion (x,y,z,w)
    """
    # TODO: implement function if necessary 
    return None

def rotation_eul(point, angle):
    """ Rotate a 3D point on the xy plane using euler angles along z axis (yaw)
        according to the right hand coordinate rule.  

    Args:
        point (array-like): 3D point. If 2D, extend to 3D with z value of 0
        angle (float): rotation angle along z axis. Radian

    Returns:
        object:numpy.array: the point after rotation
    """
    # convert angle to 3x3 rotation matrix
    # NOTE: only works for yaw rotation
    rot_matrix = np.zeros([3,3])
    rot_matrix[0][0] = math.cos(angle)
    rot_matrix[0][1] = -math.sin(angle)
    rot_matrix[0][2] = 0
    rot_matrix[1][0] = math.sin(angle)
    rot_matrix[1][1] = math.cos(angle)
    rot_matrix[1][2] = 0
    rot_matrix[2][0] = 0
    rot_matrix[2][1] = 0
    rot_matrix[2][2] = 1
    # prepare point for rotation matrix
    point_mat = np.array(point)
    point_mat = np.reshape(point_mat,[3,1])

    result = np.matmul(rot_matrix,point_mat)

    return np.squeeze(np.reshape(result,[1,3]))

def rotation_quat(point, rotation):
    """ Rotate a 3D point using quaternion. 

    Args:
        point (array-like): 3D point. If 2D, extend to 3D with z value of 0
        rotation (array-like): quaternion in (x,y,z,w)

    Returns:
        object:numpy.array: the point after rotation
    """
    # convert quaternion to 3x3 rotation matrix
    rot_matrix = np.zeros([3,3])
    rot_matrix[0][0] = 1 - 2*rotation[1]**2-2*rotation[2]**2
    rot_matrix[0][1] = 2*rotation[0]*rotation[1] - 2*rotation[2]*rotation[3]
    rot_matrix[0][2] = 2*rotation[0]*rotation[2] + 2*rotation[1]*rotation[3]
    rot_matrix[1][0] = 2*rotation[0]*rotation[1] + 2*rotation[2]*rotation[3]
    rot_matrix[1][1] = 1 - 2*rotation[0]**2-2*rotation[2]**2
    rot_matrix[1][2] = 2*rotation[1]*rotation[2] - 2*rotation[0]*rotation[3]
    rot_matrix[2][0] = 2*rotation[0]*rotation[2] - 2*rotation[1]*rotation[3]
    rot_matrix[2][1] = 2*rotation[1]*rotation[2] + 2*rotation[0]*rotation[3]
    rot_matrix[2][2] = 1 - 2*rotation[0]**2-2*rotation[1]**2
    # prepare point for rotation matrix
    point_mat = np.array(point)
    point_mat = np.reshape(point_mat,[3,1])

    result = np.matmul(rot_matrix,point_mat)

    return np.squeeze(np.reshape(result,[1,3]))

def tf_point(point, translation, rotation, quat = True):
    """ Transform a point according to the given translation and rotation. Apply 
        rotation first, then translation.

    Args:
        point (array-like): the point to be transformed, assuming 3D. 2D assume z is 0
        translation (array-like): linear translation along (x,y,z) axises
        rotation (array-like): yaw angle or quaternion rotation
        euler (bool, optional): Whether the given rotation is quaternion or 
        yaw angle. True means quat, False otherwise. Defaults to True.

    Returns:
        object:numpy.array: the point after transformation
    """
    if quat:
        transformed = rotation_quat(point,rotation)
    else:
        transformed = rotation_eul(point,rotation)
    print(transformed)
    for i in range(3):
        transformed[i] = transformed[i] + translation[i]
    return transformed

