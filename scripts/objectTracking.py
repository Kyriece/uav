#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, SetModeRequest, CommandBoolRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def callback(data):
	positionPub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
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
	    positionPub.publish(pose_msg)
	    rate = rospy.Rate(10)
	    rate.sleep()

	# arm_cmd = CommandBool()
	# arm_cmd.value = True

	# arm_result = arming_client.call(arm_cmd)
	# while not arm_result.success:
	# 	rospy.sleep(0.1)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/mavros/global_position/local', Odometry, callback)
    print("running spin")
    arm()
    rospy.spin()

def arm():
	state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)

	# Arming
	rospy.wait_for_service("/mavros/cmd/arming")
	arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

	# Mode
	rospy.wait_for_service("/mavros/set_mode")
	set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

	rate = rospy.Rate(20)

	# Wait for Flight Controller connection
	print("Waiting for connections")
	while not rospy.is_shutdown() and not current_state.connected:
		rate.sleep()

	guided_set_mode = SetModeRequest()
	guided_set_mode.custom_mode = 'GUIDED'

	# Set the mode
	set_mode_response = set_mode_client.call(guided_set_mode)

	if set_mode_response.mode_sent:
		print("Mode set to GUIDED")
		arm_cmd = CommandBoolRequest()
		arm_cmd.value = True
		arming_response = arming_client.call(arm_cmd)
		if arming_response.success:
			print("Armed")
		else:
			print("Failed to arm")
	else:
		print("Failed to set mode to GUIDED")



if __name__ == '__main__':
    listener()
