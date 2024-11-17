import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from marine_acoustic_msgs.msg import ProjectedSonarImage

f = open("timestamps.pkl", "w")

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.header.stamp)

    a = data.header.stamp.secs
    print(a)
    f.write(str(a) + '\n')

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/docking_control/vision_pose/pose', PoseStamped, callback)

    rospy.spin()

if __name__ == '__main__':
    print("listening...")
    listener()
