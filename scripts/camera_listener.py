import rospy
from scipy.spatial.transform import Rotation as R
from scipy import linalg
import numpy as np
from geometry_msgs.msg import PoseStamped

# initialize camera pose data dict. for short-baseline camera poses
camera_pos_data = {}
subscriber_id = 0

# TODO: camera position offset matrix (use topic /tf_static)

# camera_mat and scale_mat are constants
camera_mat = np.array([[1078.17559,         0., 1010.57086,     0.],
                        [        0., 1076.46176,  463.06243,     0.],
                        [        0.,         0.,         1.,     0.],
                        [        0.,         0.,         0.,     1.]])
camera_mat_inv = linalg.inv(camera_mat)

scale_mat = np.array([[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]])
scale_mat_inv = linalg.inv(scale_mat)

def callback(data):
    # collect pose and orientation of camera in the world
    position = data.pose.position
    quaternion = [data.pose.orientation.x, data.pose.orientation.y, 
                  data.pose.orientation.z, data.pose.orientation.w]
    # convert from quat. to rotation matrix
    r_mat = R.from_quat(quaternion).as_matrix()

    # assemble world_mat
    world_mat = np.array([[r_mat[0][0], r_mat[0][1], r_mat[0][2], position.x],
                          [r_mat[1][0], r_mat[1][1], r_mat[1][2], position.y],
                          [r_mat[2][0], r_mat[2][1], r_mat[2][2], position.z],
                          [        0.0,         0.0,         0.0,        1.0]])
    world_mat_inv = linalg.inv(world_mat)
    
    # TEST see outputs of camera_mat, world_mat, scale_mat and inverses
    print(f"TEST camera_mat:\n{camera_mat}\n")
    print(f"TEST camera_mat_inv:\n{camera_mat_inv}\n")
    # print(f"TEST cam*cam_inv:\n{np.dot(camera_mat, camera_mat_inv)}\n")
    
    print(f"TEST world_mat:\n{world_mat}\n")
    print(f"TEST world_mat_inv:\n{world_mat_inv}\n")
    # print(f"TEST world*world_inv:\n{np.dot(world_mat, world_mat_inv)}\n")
    
    print(f"TEST scale_mat:\n{scale_mat}\n")
    print(f"TEST scale_mat_inv:\n{scale_mat_inv}\n")
    # print(f"TEST scale*scale_inv:\n{np.dot(scale_mat, scale_mat_inv)}\n")

    # TODO: count up using global index var.
    # Populate the dictionary in the desired order
    camera_pos_data[f"camera_mat_{i}"] = camera_mat
    camera_pos_data[f"camera_mat_inv_{i}"] = camera_mat_inv
    
    camera_pos_data[f"world_mat_{i}"] = world_mat
    camera_pos_data[f"world_mat_inv_{i}"] = world_mat_inv
    
    camera_pos_data[f"scale_mat_{i}"] = scale_mat
    camera_pos_data[f"scale_mat_inv_{i}"] = scale_mat

    # TODO: check if data is being collected, if not save the dict
    # make sure it doesn't auto-save when rosbag hasn't started yet (save once rosbag runs first, then stop when stops collecting data)
    if(True):
        # Save the ordered arrays to an .npz file
        np.savez("ordered_matrices.npz", **ordered_arrays)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/docking_control/vision_pose/pose', PoseStamped, callback)

    rospy.spin()

if __name__ == '__main__':
    # print(scipy.__version__)
    print("listening...")
    listener()
