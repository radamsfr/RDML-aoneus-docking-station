import rospy
from scipy.spatial.transform import Rotation as R
from scipy import linalg
import numpy as np
import time
from geometry_msgs.msg import PoseStamped

class CameraPosData:
    def __init__(self):
        # initialize camera pose data dict. for short-baseline camera poses
        self.camera_pos_data = {}
        self.frame = 0
        self.bag_started = False
        self.timeout = 1.0
        self.last_received_time = 0.0
        
        # TODO: camera position offset matrix (use topic /tf_static)
        
        # camera_mat and scale_mat are constants
        self.camera_mat = np.array([[1078.17559,         0., 1010.57086,     0.],
                                [        0., 1076.46176,  463.06243,     0.],
                                [        0.,         0.,         1.,     0.],
                                [        0.,         0.,         0.,     1.]])
        self.camera_mat_inv = linalg.inv(self.camera_mat)

        self.scale_mat = np.array([[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]])
        self.scale_mat_inv = linalg.inv(self.scale_mat)
        
    def callback(self, data):
        # 
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
        world_mat_inv = linalg.inv(world_mat)
        
        # TEST see outputs of camera_mat, world_mat, scale_mat and inverses
        print(f"TEST camera_mat:\n{self.camera_mat}\n")
        print(f"TEST camera_mat_inv:\n{self.camera_mat_inv}\n")
        print(f"TEST world_mat:\n{world_mat}\n")
        print(f"TEST world_mat_inv:\n{world_mat_inv}\n")
        print(f"TEST scale_mat:\n{self.scale_mat}\n")
        print(f"TEST scale_mat_inv:\n{self.scale_mat_inv}\n")

        # Populate the dictionary in the desired order
        self.camera_pos_data[f"camera_mat_{self.frame}"] = self.camera_mat
        self.camera_pos_data[f"camera_mat_inv_{self.frame}"] = self.camera_mat_inv
        
        self.camera_pos_data[f"world_mat_{self.frame}"] = world_mat
        self.camera_pos_data[f"world_mat_inv_{self.frame}"] = world_mat_inv
        
        self.camera_pos_data[f"scale_mat_{self.frame}"] = self.scale_mat
        self.camera_pos_data[f"scale_mat_inv_{self.frame}"] = self.scale_mat
        self.frame += 1


    def check_bag_stopped(self, event):
        """
        Timer function to check for message inactivity.
        """
        if self.bag_started and self.last_received_time != 0.0:
            elapsed_time = time.time() - self.last_received_time
            if elapsed_time > self.timeout:
                rospy.loginfo("No messages received. Saving data to .npz")
                np.savez("camera_pos_data.npz", **self.camera_pos_data)
                self.timer.shutdown()


    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/docking_control/vision_pose/pose', PoseStamped, self.callback)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.check_bag_stopped)

        # TODO: check if data is being collected, if not save the dict
        # make sure it doesn't auto-save when rosbag hasn't started yet (save once rosbag runs first, then stop when stops collecting data)
        # if(not start_collect and ):
        #     # Save the ordered arrays to an .npz file
        #     np.savez("ordered_matrices.npz", **camera_pos_data)
        rospy.spin()
        





if __name__ == '__main__':
    c = CameraPosData()
    print("listening")
    c.listener()
    
    