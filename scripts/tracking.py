#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Range
from threading import Lock

# Globals
current_state = None
marker_center = None
gray_image=None
current_altitude = None
lock = Lock()

# Camera calibration parameters
# None in this case

target_z = 4.0  # Target altitude in meters
altitude_threshold = 0.1  # Altitude error threshold in meters
vertical_speed = 0.5  # Vertical speed when correcting altitude

horizontal_speed = 0.5  # Horizontal speed when correcting position
position_threshold = 0.1  # Position error threshold in pixels (since we don't have real-world measurements now)

def state_cb(msg):
    global current_state
    current_state = msg

# def altitude_callback(msg):
#     global current_altitude
#     current_altitude = msg.range

def image_callback(msg):
    global marker_center, lock
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

    # Detect the markers in the image
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(gray_image, dictionary, parameters=parameters)

    if markerIds is not None:
        # We have detected at least one marker
        # Get the center of the marker
        center_x = np.mean(markerCorners[0][0,:,0])
        center_y = np.mean(markerCorners[0][0,:,1])
        rospy.loginfo("Marker detected")
        # Update marker_center
        with lock:
            gray_image = local_gray_image
            marker_center = np.array([center_x, center_y])

rospy.init_node('offb_node', anonymous=True)
last_request = rospy.Time.now()
rate = rospy.Rate(20.0)
state_sub = rospy.Subscriber("mavros/state", State, state_cb)
image_sub = rospy.Subscriber("webcam/image_raw", Image, image_callback)
# altitude_sub = rospy.Subscriber("mavros/global_position/rel_alt", Range, altitude_callback)
vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

velocity = Twist()

while not rospy.is_shutdown():
    if current_state is not None and current_state.connected:
        if current_state.armed and current_state.mode == "GUIDED":
            with lock:
                if marker_center is not None and gray_image is not None:
                    # Calculate position error
                    error_x = gray_image.shape[1]/2 - marker_center[0]  # x error
                    error_y = gray_image.shape[0]/2 - marker_center[1]  # y error

                    # X and Y control
                    if abs(error_x) > position_threshold:
                        if error_x > 0:
                            velocity.linear.x = horizontal_speed
                        else:
                            velocity.linear.x = -horizontal_speed
                    else:
                        velocity.linear.x = 0

                    if abs(error_y) > position_threshold:
                        if error_y > 0:
                            velocity.linear.y = horizontal_speed
                        else:
                            velocity.linear.y = -horizontal_speed
                    else:
                        velocity.linear.y = 0
                    
                    # # Z control
                    # if abs(target_z - current_altitude) > altitude_threshold:
                    #     if target_z > current_altitude:
                    #         velocity.linear.z = vertical_speed
                    #     else:
                    #         velocity.linear.z = -vertical_speed
                    # else:
                    #     velocity.linear.z = 0

                    # Publish velocity command
                    vel_pub.publish(velocity)
                    
        
    rate.sleep()