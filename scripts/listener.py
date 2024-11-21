import rospy
import numpy
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from marine_acoustic_msgs.msg import ProjectedSonarImage

cameras_sphere = {}


f = open("camera_pose.txt", "w")

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.header.stamp)

    position = data.pose.position
    quaternion = data.pose.orientation
    
    # TODO make translation matrix
    # collect position and quaternion values
    # convert quaternion values to rotation matrix
    # combine to make translation matrix
    
    # TODO
    # store in npz file with camera_matrix, scale_matrix, and inv_matrices
    
    
    # print("position:\n" + str(position) + '\n')
    print(f"rotation:\n{str(rotation)}\n")
    f.write(str(rotation) + '\n\n')

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/docking_control/vision_pose/pose', PoseStamped, callback)

    rospy.spin()

if __name__ == '__main__':
    print("listening...")
    listener()
