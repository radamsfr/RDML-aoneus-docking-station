import rospy
from scipy.spatial.transform import Rotation as R
from scipy import linalg
import numpy as np
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from marine_acoustic_msgs.msg import ProjectedSonarImage
from cv_bridge import CvBridge
import cv2
import os
import pickle

class Listener:
    def __init__(self):
        # initialize camera pose data dict. for short-baseline camera poses
        self.camera_data = {}
        self.camera_pos = []
        self.camera_pos_offset = []

        # initialize sonar pose data dict. for short-baseline sonar poses
        self.sonar_data = {}
        self.sonar_pos = []
        self.sonar_pos_offset = []
        self.sonar_img_mat = []
        self.sonar_img_mat_temp = []
        
        self.pos_offset_collected = False
        
        # image data collection
        self.camera_image = None
        self.camera_W = 512
        self.camera_H = 443
        self.sonar_image = None
        self.sonar_W = 50
        self.sonar_H = 64
        self.bridge = CvBridge()
          
        # initialize bag start/stop vars
        self.frame = 0
        self.bag_started = False
        self.timeout = 2.0
        self.last_received_time = 0.0
        
        # initialize directories for data collection
        path_to_data = "RDML_aoneus_data"
        os.makedirs(path_to_data + '/sonar_component/Data', exist_ok = True)
        os.makedirs(path_to_data + '/sonar_component/imgs', exist_ok = True)
        os.makedirs(path_to_data + '/camera_component/image', exist_ok = True)
        
        # camera_mat and scale_mat are constants
        self.camera_mat = np.array([[1078.17559,         0., 1010.57086,     0.],
                                [        0., 1076.46176,  463.06243,     0.],
                                [        0.,         0.,         1.,     0.],
                                [        0.,         0.,         0.,     1.]])
        self.camera_mat_inv = linalg.inv(self.camera_mat)

        self.scale_mat = np.array([[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]])
        self.scale_mat_inv = linalg.inv(self.scale_mat)
        
    
    def callback_pose_offset(self, data):
        if not len(self.camera_pos_offset) or not len(self.sonar_pos_offset):
            for t in data.transforms:
                if t.child_frame_id == "camera_link":
                    # collect pose and orientation of camera in the world
                    position = t.transform.translation
                    quaternion = [t.transform.rotation.x, t.transform.rotation.y, 
                                t.transform.rotation.z, t.transform.rotation.w]
                    r_mat = R.from_quat(quaternion).as_matrix()
                    
                    self.camera_pos_offset = np.array([[r_mat[0][0], r_mat[0][1], r_mat[0][2], position.x],
                                                    [r_mat[1][0], r_mat[1][1], r_mat[1][2], position.y],
                                                    [r_mat[2][0], r_mat[2][1], r_mat[2][2], position.z],
                                                    [        0.0,         0.0,         0.0,        1.0]])
                
                elif t.child_frame_id == "sonar_link":
                    # collect pose and orientation of sonar in the world
                    position = t.transform.translation
                    quaternion = [t.transform.rotation.x, t.transform.rotation.y, 
                                t.transform.rotation.z, t.transform.rotation.w]
                    r_mat = R.from_quat(quaternion).as_matrix()
                    
                    self.sonar_pos_offset = np.array([[r_mat[0][0], r_mat[0][1], r_mat[0][2], position.x],
                                                    [r_mat[1][0], r_mat[1][1], r_mat[1][2], position.y],
                                                    [r_mat[2][0], r_mat[2][1], r_mat[2][2], position.z],
                                                    [        0.0,         0.0,         0.0,        1.0]])
            
            # print(f"camera pos offset: {self.camera_pos_offset}")
            # print(f"sonar pos offset: {self.sonar_pos_offset}")
            if len(self.camera_pos_offset) and len(self.sonar_pos_offset):
                print("pos offsets collected")
                self.pos_offset_collected = True
                
            
    def callback_pose(self, data):
        # rospy.loginfo("CALLBACK POSE")
        self.bag_started = True
        self.last_received_time = time.time()
        
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
        
        self.camera_pos = np.dot(self.camera_pos_offset, world_mat)
        # rospy.loginfo(f"TEST camera_mat:\n{self.camera_pos}\n")
        self.sonar_pos = np.dot(self.sonar_pos_offset, world_mat)        
        # rospy.loginfo(f"TEST sonar_mat:\n{self.sonar_pos}\n")
        
        
    def callback_sonar_img_mat(self, data):
        # rospy.loginfo("CALLBACK SONAR IMG MAT")
        self.sonar_img_mat_temp = []        
        for i in data.image.data:
            self.sonar_img_mat_temp.append(np.uint8(i))

        self.sonar_img_mat_temp = np.array(self.sonar_img_mat_temp, dtype=np.uint8)
        # rospy.loginfo(f'shape of sonar img:{self.sonar_img_mat_temp.shape}')
        # self.sonar_img_mat = self.sonar_img_mat_temp.reshape(512, 443)
        self.sonar_img_mat = cv2.resize(self.sonar_img_mat_temp.reshape(512, 443), (69, 60))[0:50, 0:64]
        
        
        #TODO may have to resize sonar image matrix
        # self.sonar_img_mat = resize(self.sonar_image_mat, (115,100), anti_aliasing=True)
        # print("sonar mat shape:", self.sonar_img_mat.shape)
        # rospy.loginfo(f'shape of sonar img reshaped:{self.sonar_img_mat.shape}')
      
      
    def callback_camera_img(self, data):
        # rospy.loginfo("CALLBACK CAMERA IMAGE")
        self.camera_image = cv2.resize(self.bridge.imgmsg_to_cv2(data), (512, 443))
        
    def callback_sonar_img(self, data):
        # rospy.loginfo("CALLBACK SONAR IMAGE")
        # self.sonar_image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(data), cv2.COLOR_BGR2GRAY)
        self.sonar_image = cv2.resize(cv2.cvtColor(self.bridge.imgmsg_to_cv2(data), cv2.COLOR_BGR2GRAY), (69, 60))[0:50, 0:64]
      
        
    def record(self, event):
        if self.bag_started and self.last_received_time != 0.0:
            
            if (len(self.camera_pos) and len(self.sonar_img_mat) and len(self.sonar_pos) and
                self.camera_image is not None and self.sonar_image is not None):
                self.save_sonar()
                self.save_camera()

                cv2.imwrite(f"RDML_aoneus_data/camera_component/image/{self.frame:03}.png", self.camera_image)
                cv2.imwrite(f"RDML_aoneus_data/sonar_component/imgs/{self.frame:03}.png", self.sonar_image)
                # print("Wrote image %i" % self.frame)
                    
                self.frame += 1
                    
            elapsed_time = time.time() - self.last_received_time
            if elapsed_time > self.timeout:
                rospy.loginfo("No messages received. Saving camera data to .npz")
                np.savez("RDML_aoneus_data/camera_component/camera_pos_data.npz", **self.camera_data)
                self.timer.shutdown()
                return
                
           
    def save_camera(self):
        self.camera_data[f"camera_mat_{self.frame}"] = self.camera_mat
        self.camera_data[f"camera_mat_inv_{self.frame}"] = self.camera_mat_inv
        
        self.camera_data[f"world_mat_{self.frame}"] = self.camera_pos
        self.camera_data[f"world_mat_inv_{self.frame}"] = linalg.inv(self.camera_pos)
        
        self.camera_data[f"scale_mat_{self.frame}"] = self.scale_mat
        self.camera_data[f"scale_mat_inv_{self.frame}"] = self.scale_mat_inv
        
        rospy.loginfo(f"camera data collected and stored, frame {self.frame:03}\n")
        
        
    def save_sonar(self):
        self.sonar_data = {
            'PoseSensor': self.sonar_pos, 
            'ImagingSonar': self.sonar_img_mat
            }

        with open(f"RDML_aoneus_data/sonar_component/Data/{self.frame:03}.pkl", 'wb') as f:
            pickle.dump(self.sonar_data, f)
        
        rospy.loginfo(f"sonar data collected and stored, frame {self.frame:03}\n")
        
        
    def listener(self):
        rospy.init_node('listener', anonymous=True)
        
        while not self.pos_offset_collected:
            rospy.Subscriber('/tf_static', TFMessage, self.callback_pose_offset)
    
        rospy.Subscriber('/docking_control/vision_pose/pose', PoseStamped, self.callback_pose)
        rospy.Subscriber('/oculus/sonar_image', ProjectedSonarImage, self.callback_sonar_img_mat)
        rospy.Subscriber('/BlueROV2/video', Image, self.callback_camera_img)
        rospy.Subscriber('/oculus/drawn_sonar_rect', Image, self.callback_sonar_img)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.record)

        rospy.spin()
     
        
if __name__ == '__main__':
    s = Listener()
    print("listening...")
    s.listener()