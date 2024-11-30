import rospy
from scipy.spatial.transform import Rotation as R
from scipy import linalg
import numpy as np
from geometry_msgs.msg import PoseStamped
# from sensor_msgs.msg import Image
from marine_acoustic_msgs.msg import ProjectedSonarImage


# TODO: sonar position offset matrix (use topic /tf_static)

def callback_pose(data):
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
    
    # TEST see output of world_mat and inverse  
    print(f"TEST world_mat:\n{world_mat}\n")
    print(f"TEST world_mat_inv:\n{world_mat_inv}\n")
    # print(f"TEST world*world_inv:\n{np.dot(world_mat, world_mat_inv)}\n")

def callback_sonar_img(data):
    #TODO: collect sonar_img data (array)
    
    #TODO: put camera pose and sonar img into pkl file
    return
    

def listener():
    rospy.init_node('listener1', anonymous=True)
    rospy.Subscriber('/docking_control/vision_pose/pose', PoseStamped, callback_pose)
    rospy.Subscriber('/oculus/sonar_image', ProjectedSonarImage, callback_sonar_img)

    rospy.spin()

if __name__ == '__main__':
    # print(scipy.__version__)
    print("listening...")
    listener()
