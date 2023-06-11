#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def callback(data):
	pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
	buffer = 0.5
	desired_location = 2,2,2
	print("publishing")

	# Create pose message
	pose_msg = PoseStamped()
	pose_msg.pose.position.x = desired_location[0] # Desired position in the x-axis
	pose_msg.pose.position.y = desired_location[1] # Desired position in the y-axis
	pose_msg.pose.position.z = desired_location[2] # Desired position in the z-axis
    
    # publish
    
	while not rospy.is_shutdown():
	    pose_msg.header.stamp = rospy.Time.now()
	    pub.publish(pose_msg)
	    rate = rospy.Rate(10)
	    rate.sleep()


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/mavros/global_position/local', Odometry, callback)
    print("running spin")
    rospy.spin()

if __name__ == '__main__':
    listener()