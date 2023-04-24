#!/usr/bin/env python3

import rospy, cv2
from opencv_apps.msg import MomentArrayStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# callback function for contour_moments
def callbackContouMoments(receivedMsg):
    #rospy.loginfo("Test")
    rospy.loginfo(receivedMsg)

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(receivedMsg, "CV_8UC3")


#Function used to setup all publisher subscribers and so on
def setup():

	#Initiating node
	rospy.init_node('cellDetectionNode', anonymous=True)

	#Subscribing to /contour_moments/moments to get the Laserscanner information
	rospy.Subscriber('/contour_moments_edge_detection/image', Image, callbackContouMoments)

	#Setting refresh rate
	rate = rospy.Rate(15)
	while not rospy.is_shutdown():
		rate.sleep()
	rospy.spin()

if __name__ == "__main__":
	try:
		setup()
	except rospy.ROSInterruptException:
		pass