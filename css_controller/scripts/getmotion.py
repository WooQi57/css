import rospy
from sensor_msgs.msg import JointState

def cb(data):
    print(rospy.get_time(),data.position[1])

def listener():
    rospy.init_node('getmoton',anonymous=True)

    rospy.Subscriber("move_group/fake_controller_joint_states", JointState, cb)

    rospy.spin()

if __name__=='__main__':
    listener()

