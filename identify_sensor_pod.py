import cv2
import numpy as np
import pdb
import sys
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge
import glob
import os
# from apriltag_ros import Detector
#import apriltag
from nav_msgs.msg import Odometry

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

IMAGE_TOPIC = "pod_images/image" #rospy.get_param("sensor_pod/image_topic")
POD_LOCATION_TOPIC = "/pod_location"

sensor_pod_template_path = 'sensor_pod_'

class SensorPodIdentifier:
	def __init__(self):
		# image_files = glob.glob(os.path.join("/all_images", "*.jpg"))
		# for img in image_files:
		# 	image_topic = os.path.splitext(os.path.basename(img))[0]
		# 	self.subscriber = rospy.Subscriber(SensorPodIdentifier.IMAGE_TOPIC, Image, self.callback)
		self.image_subscriber = rospy.Subscriber("/front_cam/camera/image", Image, self.callback)
		self.publisher = rospy.Publisher(POD_LOCATION_TOPIC, PoseStamped, queue_size=10)
		self.subscriber = rospy.Subscriber('/ground_truth/state', Odometry, self.odom_callback)
		self.range_sub = rospy.Subscriber('/sonar_height', Range, self.range_callback)
		self.latest_image = None
		self.bridge = CvBridge()
		self.drone_pose = None

	def odom_callback(self, msg: Odometry) -> None:
		"""
        Saves in the odometry for the drone to conduct its planning

        Parameters:
        msg (Odometry): pose information of the drone

        Returns:
        None
        """
		self.drone_pose = msg.pose.pose

	def callback(self,  msg):
		# self.latest_image = self.bridge.imgmsg_to_cv2(Image_data, "bgr8")
		self.latest_image = self.bridge.imgmsg_to_cv2(msg)

		#print(self.cd_color_segmentation(self.latest_image))
		# self.publish_sensor_pod_tip_pose()

	# def image_callback(self, msg):
	# 	# Convert ROS image message to OpenCV image
	# 	cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
	# 	# Convert image to grayscale
	# 	gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	# 	# Create April tag detector object
	# 	detector = Detector()
	# 	# Detect April tags in the image
	# 	tags = detector.detect(gray)
	# 	# Create April tag decoder object
	# 	decoder = apriltag.Detector()
	# 	# Decode the detected tag
	# 	tag_info = decoder.decode(gray, tags[0].tag_family)
	# 	# Print the tag ID
	# 	print("April tag ID:", tag_info[0].tag_id)

	def get_latest_image(self):
		return self.latest_image

	def image_print(self, img):
		"""
		Helper function to print out images, for debugging. Pass them in as a list.
		Press any key to continue
		"""
		cv2.imshow("image", img)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

	def filter_contours(self, contours):
		if len(contours) == 0:
			return None

		return [contour for contour in contours if cv2.contourArea(contour) > 1000] # WILL NEED TO CHANGE

	def get_largest_contour(self, contours):
		if len(contours) == 0:
			return None

		largest_contour = None
		largest_area = 0

		for contour in contours:
			new_area = cv2.contourArea(contour)
			if new_area > largest_area:
				x, y, w, h = cv2.boundingRect(contour)
				if w/h < 1.25 and w/h > 0.75:
					largest_area = new_area
					largest_contour = contour
		return largest_contour

	def cd_color_segmentation(self, img):
		"""
		Implement the cone detection using color segmentation algorithm
		Input:
			img: np.3darray; the input image with a cone to be detected. BGR.
			template_file_path; Not required, but can optionally be used to automate setting hue filter values.
		Return:
			bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px                                                                    (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
			"""

		# Create mask for orange cone. HSV threshods
		light_orange = (40, 100, 105) #(40, 100, 105) #(70, 180, 150)
		dark_orange = (170, 255, 255) #(170, 255, 255) #(150, 255, 255)

		# light_red = (252, 1, 2)
		# dark_red = (255, 0, 0)
		kernel = np.ones((7, 7), np.uint8)

		filtered_img = cv2.dilate(cv2.erode(img, kernel, iterations=1), kernel, iterations=1)
		print("filtered img")

		#image_print(filtered_img)
		hsv_img = cv2.cvtColor(filtered_img, cv2.COLOR_RGB2HSV)
		mask = cv2.inRange(hsv_img, light_orange, dark_orange)
		# self.image_print(mask)

		# Find remaning contours, correspond to orange objects
		contours = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
		largest_contour = self.get_largest_contour(contours)

		print("largest contour found")

		# Draw boxes around the contours
		x, y, w, h = cv2.boundingRect(largest_contour)
		bounding_box = ((x, y), (x+w, y+h))

		boxes = []
		for c in contours:
			x, y, w, h = cv2.boundingRect(c)
			boxes.append(((x, y), (x+w, y+h)))

		cv2.rectangle(self.latest_image, bounding_box[0], bounding_box[1], (0, 255, 0),2)
		cv2.imwrite("found_pod.jpg", self.latest_image)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()

		return bounding_box

	def get_sensor_pod_tip_pose(self):
		img = self.get_latest_image()
		bounding_box = self.cd_color_segmentation(img)
		x = (bounding_box[0][0]+bounding_box[1][0])/2
		y = bounding_box[0][1]
		z = 1

		return [x, y, z]
	
	# def pose_to_matrix(pose):
	# 	# Convert the quaternion orientation to a rotation matrix
	# 	q = pose.pose.orientation
	# 	r = np.array([
	# 		[1 - 2*q.y*q.y - 2*q.z*q.z, 2*q.x*q.y - 2*q.z*q.w, 2*q.x*q.z + 2*q.y*q.w],
	# 		[2*q.x*q.y + 2*q.z*q.w, 1 - 2*q.x*q.x - 2*q.z*q.z, 2*q.y*q.z - 2*q.x*q.w],
	# 		[2*q.x*q.z - 2*q.y*q.w, 2*q.y*q.z + 2*q.x*q.w, 1 - 2*q.x*q.x - 2*q.y*q.y]
	# 	])
		
	# 	# Create a homogeneous transformation matrix from the rotation matrix
	# 	t = np.eye(4)
	# 	t[:3,:3] = r
		
	# 	return t
	

	def object_callback(self, msg):
        # Check if we have received the drone pose
		if self.drone_pose is None:
			rospy.logwarn('No drone pose received yet. Cannot transform object position.')
			return

        # Convert the position of the object to a numpy array
		P_camera = np.array(self.get_sensor_pod_tip_pose()) #np.array([msg.point.x, msg.point.y, msg.point.z, 1.0])

        # Convert the quaternion orientation of the drone to a rotation matrix
		q = self.drone_pose.pose.orientation
		r = np.array([
            [1 - 2*q.y*q.y - 2*q.z*q.z, 2*q.x*q.y - 2*q.z*q.w, 2*q.x*q.z + 2*q.y*q.w],
            [2*q.x*q.y + 2*q.z*q.w, 1 - 2*q.x*q.x - 2*q.z*q.z, 2*q.y*q.z - 2*q.x*q.w],
            [2*q.x*q.z - 2*q.y*q.w, 2*q.y*q.z + 2*q.x*q.w, 1 - 2*q.x*q.x - 2*q.y*q.y]
        ])

        # Create a homogeneous transformation matrix from the rotation matrix
		t = np.eye(4)
		t[:3,:3] = r

        # Transform the position of the object from the camera frame to the drone frame
		P_drone = np.dot(np.linalg.inv(t), P_camera)
		P_drone = P_drone[:3] / P_drone[3]

        # Add the position of the drone in the world frame to get the position of the object in the world frame
		P_drone_world = np.array([self.drone_pose.pose.position.x, self.drone_pose.pose.position.y, self.drone_pose.pose.position.z])
		P_world = P_drone[:3] + P_drone_world

		# Create a PointStamped message for the position of the object in the drone frame
		object_msg = PointStamped()
		object_msg.header.frame_id = 'world'
		object_msg.header.stamp = rospy.Time.now()
		object_msg.point.x = P_world[0]
		object_msg.point.y = P_world[1]
		object_msg.point.z = P_world[2]

		# Publish the position of the object in the drone frame
		self.publisher.publish(object_msg)

		rospy.loginfo('Object position in world frame: x = {}, y = {}, z = {}'.format(P_world[0], P_world[1], P_world[2]))


	# def publish_sensor_pod_tip_pose(self):
	# 	pod_tip = self.get_sensor_pod_tip_pose()
	# 	pose_msg = PoseStamped()
	# 	pose_msg.header.frame_id = "world"
	# 	pose_msg.pose.position.x = pod_tip[0]
	# 	pose_msg.pose.position.y = pod_tip[1]
	# 	pose_msg.pose.position.z = pod_tip[2]
	# 	pose_msg.pose.orientation.x = 0.0
	# 	pose_msg.pose.orientation.y = 0.0
	# 	pose_msg.pose.orientation.z = 0.0
	# 	pose_msg.pose.orientation.w = 1.0

	# 	rospy.loginfo(pose_msg)
	# 	self.publisher.publish(pose_msg)


if __name__ == '__main__':
	try:
		rospy.init_node('sensor_pod_identifier')
		sensor_pod_identifier = SensorPodIdentifier()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


	# image_subscriber = SensorPodIdentifier()
	# while not rospy.is_shutdown():
	# 	latest_image = image_subscriber.get_latest_image()
	# 	if latest_image is not None:
    #         # process the latest image
	# 		pass
	# 	else:
	# 		rospy.loginfo('No image received yet.')
	# 	rospy.sleep(0.01)

	# for i in range(10):
	# 	cone_image = cv2.imread(sensor_pod_template_path + str(i) + ".jpg")

	# 	boundary_box = cd_color_segmentation(cone_image)
	# 	cv2.rectangle(cone_image, boundary_box[0], boundary_box[1], (0, 255, 0),2)

	# 	image_print(cone_image)
