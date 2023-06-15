#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped, Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
import signal
from mavros_msgs.srv import CommandBool, SetMode, SetModeRequest, CommandBoolRequest, CommandTOL
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Range
import cv2
import cv2.aruco as aruco
import numpy as np


current_state = State()
velocity_pub = None
takeoff_altitude = 5

horizontal_speed = 0.5  # Horizontal speed when correcting position
position_threshold = 40  # Position error threshold in pixels (since we don't have real-world measurements now)


def state_cb(msg):
    global current_state
    current_state = msg

def image_callback(msg):
    global marker_center
    bridge = CvBridge()

    # Convert ROS Image message to OpenCV image
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Convert in gray scale
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    local_gray_image = gray_image

    # Load the predefined dictionary
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

    # Initialize the detector parameters using default values
    parameters = cv2.aruco.DetectorParameters_create()
    
    # Increase the minimum and maximum marker area
    # Increase the minimum and maximum marker perimeter rates
    parameters.minMarkerPerimeterRate = 0.0001  # Lower value to be more lenient
    parameters.maxMarkerPerimeterRate = 10.0   # Higher value to be more lenient


    # Detect the markers in the image
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(gray_image, dictionary, parameters=parameters)

    if markerIds is not None:
        # We have detected at least one marker
        # Get the center of the marker
        center_x = np.mean(markerCorners[0][0,:,0])
        center_y = np.mean(markerCorners[0][0,:,1])
        rospy.loginfo("Marker detected")
        # Update marker_center
        marker_center = np.array([center_x, center_y])
        centerObject(gray_image, marker_center)
        
def centerObject(gray_image, marker_center):
    vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
    velocity = Twist()
    
    # Calculate position error
    error_x = gray_image.shape[1]/2 - marker_center[0]  # x error
    error_y = gray_image.shape[0]/2 - marker_center[1]  # y error
    
    scaled_speed_x = calculate_scaled_speed(abs(error_x))
    scaled_speed_y = calculate_scaled_speed(abs(error_y))

    # X and Y control
    if abs(error_x) > position_threshold:
        if error_x > 0:
            print("Error X: Moving Left " + str(scaled_speed_x))
            velocity.linear.x = scaled_speed_x
        else:
            print("Error X: Moving Right " + str(scaled_speed_x))
            velocity.linear.x = -scaled_speed_x
    else:
        velocity.linear.x = 0

    if abs(error_y) > position_threshold:
        if error_y > 0:
            print("Error Y: Moving up" + str(error_y))
            velocity.linear.y = scaled_speed_y
        else:
            print("Error Y: Moving down " + str(error_y))
            velocity.linear.y = scaled_speed_y
    else:
        velocity.linear.y = 0
    
    vel_pub.publish(velocity)

def calculate_scaled_speed(error):
    if error > position_threshold:
        # Calculate the scaled speed within the range of 0 to maximum_speed
        scaled_speed = (error - position_threshold) / (horizontal_speed - position_threshold) * horizontal_speed
        return scaled_speed
    else:
        return 0.0

def callback(data):
    # Get pose data
    pose = data.pose.pose
    altitude = pose.position.z
    print(altitude)

    # Create a publisher for the velocity commands
    velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    # Create a Twist message and set the linear and angular velocities
    velocity_msg = TwistStamped()
        
    if altitude > takeoff_altitude:
        
        velocity_msg.twist.linear.z = -1.0

        # Publish the velocity message repeatedly
        rate = rospy.Rate(10)  # 10 Hz
        velocity_msg.header.stamp = rospy.Time.now()
        velocity_pub.publish(velocity_msg)
        rate.sleep()
    else:
        velocity_msg.twist.linear.z
        rate = rospy.Rate(10)  # 10 Hz
        velocity_msg.header.stamp = rospy.Time.now()
        velocity_pub.publish(velocity_msg)
        rate.sleep()
        

def listener():
    rospy.init_node('listener', anonymous=True)
    arm()
    takeoff()
    rospy.Subscriber('/mavros/global_position/local', Odometry, callback)
    image_sub = rospy.Subscriber("webcam/image_raw", Image, image_callback)
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

def stop_drone():
    # Send a zero velocity command to stop the drone
    velocity_pub_2 = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    velocity_msg = TwistStamped()
    velocity_msg.twist.linear.x = 0.0
    velocity_msg.twist.linear.y = 0.0
    velocity_msg.twist.linear.z = 0.0
    velocity_msg.header.stamp = rospy.Time.now()
    velocity_pub_2.publish(velocity_msg)
    

def takeoff():
    rospy.wait_for_service('/mavros/cmd/takeoff')
    takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    takeoff_service(0.0, 0.0, 0.0, 0.0, takeoff_altitude)
    print("Taking off")

if __name__ == '__main__':
    listener()
