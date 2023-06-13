#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped, Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
import signal
from mavros_msgs.srv import CommandBool, SetMode, SetModeRequest, CommandBoolRequest, CommandTOL

current_state = State()
velocity_pub = None
takeoff_altitude = 3

def state_cb(msg):
    global current_state
    current_state = msg

def signal_handler(sig, frame):
    # Handle the interrupt signal (Ctrl+C)
    print("Stopping")
    stop_drone()

signal.signal(signal.SIGINT, signal_handler)

def callback(data):
    # Get pose data
    pose = data.pose.pose
    altitude = pose.position.z
    print(altitude)


    # Create a publisher for the velocity commands
    velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    # Create a Twist message and set the linear and angular velocities
    velocity_msg = TwistStamped()
    velocity_msg.twist.linear.z = -1.0

    # Publish the velocity message repeatedly
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        velocity_msg.header.stamp = rospy.Time.now()
        velocity_pub.publish(velocity_msg)
        rospy.sleep(0.1)
    else:
        velocity_msg.twist.linear.x = 0.0
        velocity_msg.twist.linear.y = 0.0
        velocity_msg.twist.linear.z = 0.0
        velocity_msg.header.stamp = rospy.Time.now()
        velocity_pub.publish(velocity_msg)
        print("Published stop")


	# while not rospy.is_shutdown():
	#     pose_msg.header.stamp = rospy.Time.now()
	#     positionPub.publish(pose_msg)
	#     rate = rospy.Rate(10)
	#     rate.sleep()

def listener():
    rospy.init_node('listener', anonymous=True)
    arm()
    takeoff()
    rospy.Subscriber('/mavros/global_position/local', Odometry, callback)
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
  
def land():
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

    # Create a SetModeRequest and set the custom mode to "AUTO.LAND"
    land_mode = SetModeRequest()
    land_mode.custom_mode = "AUTO.LAND"

    # Call the set_mode service to command the drone to land
    try:
        response = set_mode_client(0, land_mode.custom_mode)
        if response.mode_sent:
            print("Drone is landing...")
        else:
            print("Failed to send landing command.")
    except rospy.ServiceException as e:
        print("Service call failed: ", str(e))

def stop_drone():
    # Send a zero velocity command to stop the drone
    velocity_pub_2 = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    velocity_msg = TwistStamped()
    

def takeoff():
    rospy.wait_for_service('/mavros/cmd/takeoff')
    takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    takeoff_service(0.0, 0.0, 0.0, 0.0, takeoff_altitude)
    print("Taking off")

if __name__ == '__main__':
    listener()
