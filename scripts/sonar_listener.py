import rospy
from scipy.spatial.transform import Rotation as R
from scipy import linalg
import numpy as np
import pickle
import time
from geometry_msgs.msg import PoseStamped
# from sensor_msgs.msg import Image
from marine_acoustic_msgs.msg import ProjectedSonarImage


class SonarListener:
    def __init__(self):
        # initialize sonar pose data dict. for short-baseline sonar poses
        self.sonar_data = {}
        self.sonar_pos = []
        self.sonar_img = []
        self.bag_started = False
        self.frame = 0
        # TODO: sonar position offset matrix (use topic /tf_static)

    def callback_pose(self, data):
        self.bag_started = True
        # collect pose and orientation of camera in the world
        position = data.pose.position
        quaternion = [data.pose.orientation.x, data.pose.orientation.y, 
                    data.pose.orientation.z, data.pose.orientation.w]
        # convert from quat. to rotation matrix
        r_mat = R.from_quat(quaternion).as_matrix()

        # assemble world_mat
        self.sonar_pos = np.array([[r_mat[0][0], r_mat[0][1], r_mat[0][2], position.x],
                            [r_mat[1][0], r_mat[1][1], r_mat[1][2], position.y],
                            [r_mat[2][0], r_mat[2][1], r_mat[2][2], position.z],
                            [        0.0,         0.0,         0.0,        1.0]])
        
        # TEST see output of world_mat and inverse  
        print(f"TEST world_mat:\n{self.sonar_pos}\n")
        
        self.frame += 1


    def callback_sonar_img(self, data):
        self.sonar_img = np.array([data.image.data])
        
        print(f"TEST sonar image:\n{self.sonar_img}\n")
    

    def record(self, event):
        if self.bag_started:
            self.sonar_data = {'PoseSensor': self.sonar_pos, 'ImagingSonar': self.sonar_img}
            
            with open(f"aoneus_data/sonar_data/00{self.frame}.pickle", 'wb') as f:
                pickle.dump(self.sonar_data, f)

    def listener(self):
        rospy.init_node('listener1', anonymous=True)
        rospy.Subscriber('/docking_control/vision_pose/pose', PoseStamped, self.callback_pose)
        rospy.Subscriber('/oculus/sonar_image', ProjectedSonarImage, self.callback_sonar_img)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.record)

        rospy.spin()

if __name__ == '__main__':
    s = SonarListener()
    print("listening...")
    s.listener()
