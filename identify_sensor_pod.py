import cv2
import numpy as np
import pdb
import sys
import rospy
from sensor_msgs.msg import Image, Range
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import glob
import os
# from apriltag_ros import Detector
#import apriltag
from nav_msgs.msg import Odometry
import tf

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
		self.image_subscriber = rospy.Subscriber("/front_cam/camera/image", Image, self.object_callback)
		self.publisher = rospy.Publisher(POD_LOCATION_TOPIC, PointStamped, queue_size=10)
		self.subscriber = rospy.Subscriber('/ground_truth/state', Odometry, self.odom_callback)
		self.range_sub = rospy.Subscriber('/sonar_height', Range, self.range_callback)
		self.latest_image = None
		self.bridge = CvBridge()
		self.drone_pose = None
		self.ground_dist = 0
		self.K = np.array([[319.9988245765257, 0, 320.5], [0, 319.9988245765257, 240.5], [0, 0, 1]]) # intrinsics Matrix
		self.P_camera_drone = np.array([0.05, 0, -0.25])
		self.R_camera_drone = tf.transformations.euler_matrix(0, np.pi/4, 0)[:3, :3]

		# self.pts_image_plane = np.array([[406.0, 276.0], # airpod_left_bottom_corner
        #            [309.0, 240.0], # green_post-its_left_bottom_corner
        #            [317.0, 295.0], # contact_lenses_right_bottom_corner
        #            [234.0, 252.0]]) # matcha_pocky_box_right_bottom_corner
		
		# self.pts_ground_plane = np.array([[14.375, -3.625],
	    #                 [23.875, 1.3125],
	    #                 [20.9375, 2.625],
	    #                 [20.625, 8.625]])
		
		# self.pts_ground_plane = self.pts_ground_plane
		# self.pts_ground_plane = np.float32(self.pts_ground_plane[:, np.newaxis, :])

		# self.pts_image_plane = self.pts_image_plane * 1.0
		# self.pts_image_plane = np.float32(self.pts_image_plane[:, np.newaxis, :])

		# self.h, err = cv2.findHomography(self.pts_image_plane, self.pts_ground_plane)

	# def callback(self, msg):
	# 	# other setup info 
	# 	img = self.bridge.imgmsg_to_cv2(msg)
	# 	bounding_box = self.cd_color_segmentation(img)
	# 	u = (bounding_box[0][0]+bounding_box[1][0])/2
	# 	v = bounding_box[0][1]
	# 	# w = self.ground_dist

	# 	#Call to main function
	# 	P_world_x, P_world_y = self.transformUvToXy(u, v)
	# 	P_world_z = 0 # solve

	# 	# Create a PointStamped message for the position of the object in the drone frame
	# 	object_msg = PointStamped()
	# 	object_msg.header.frame_id = 'world'
	# 	object_msg.header.stamp = rospy.Time.now()
	# 	object_msg.point.x = P_world_x
	# 	object_msg.point.y = P_world_y
	# 	object_msg.point.z = P_world_z

	# 	# Publish the position of the object in the drone frame
	# 	self.publisher.publish(object_msg)

	# 	rospy.loginfo('Object position in world frame: x = {}, y = {}, z = {}'.format(P_world_x, P_world_y, P_world_z))



	# def transformUvToXy(self, u, v):
	# 	"""
	# 	u and v are pixel coordinates.
	# 	The top left pixel is the origin, u axis increases to right, and v axis
	# 	increases down.

	# 	Returns a normal non-np 1x2 matrix of xy displacement vector from the
	# 	camera to the point on the ground plane.
	# 	Camera points along positive x axis and y axis increases to the left of
	# 	the camera.

	# 	Units are in meters.
	# 	"""
	# 	homogeneous_point = np.array([[u], [v], [1]])
	# 	xy = np.dot(self.h, homogeneous_point)
	# 	scaling_factor = 1.0 / xy[2, 0]
	# 	homogeneous_xy = xy * scaling_factor
	# 	x = homogeneous_xy[0, 0]
	# 	y = homogeneous_xy[1, 0]
	# 	return x, y

	def odom_callback(self, msg: Odometry) -> None:
		"""
        Saves in the odometry for the drone to conduct its planning

        Parameters:
        msg (Odometry): pose information of the drone

        Returns:
        None
        """
		self.drone_pose = msg.pose.pose

	def range_callback(self, msg) -> None:
		self.ground_dist = msg.range

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
				# if w/h < 1.25 and w/h > 0.75:
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
		# light_orange = (40, 100, 105) #(40, 100, 105) #(70, 180, 150)
		# dark_orange = (170, 255, 255) #(170, 255, 255) #(150, 255, 255)
		lower_bound = (40, 40, 40)
		upper_bound = (180, 255, 255)

		# light_red = (252, 1, 2)
		# dark_red = (255, 0, 0)
		kernel = np.ones((7, 7), np.uint8)

		filtered_img = cv2.dilate(cv2.erode(img, kernel, iterations=1), kernel, iterations=1)
		# print("filtered img")

		#image_print(filtered_img)
		hsv_img = cv2.cvtColor(filtered_img, cv2.COLOR_RGB2HSV)
		mask = cv2.inRange(hsv_img, lower_bound, upper_bound)
		# self.image_print(mask)

		# Find remaning contours, correspond to orange objects
		contours = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]

		if not contours:
			return None
		
		else:
			largest_contour = self.get_largest_contour(contours)

			# print("largest contour found")

			# Draw boxes around the contours
			x, y, w, h = cv2.boundingRect(largest_contour)
			bounding_box = ((x, y), (x+w, y+h))

			boxes = []
			for c in contours:
				x, y, w, h = cv2.boundingRect(c)
				boxes.append(((x, y), (x+w, y+h)))

			# cv2.rectangle(self.latest_image, bounding_box[0], bounding_box[1], (0, 255, 0),2)
			# cv2.imwrite("found_pod.jpg", self.latest_image)
			# cv2.waitKey(0)
			# cv2.destroyAllWindows()

			return bounding_box

	def object_callback(self, msg):
		img = self.bridge.imgmsg_to_cv2(msg)
		bounding_box = self.cd_color_segmentation(img)
		# print(bounding_box)
		if bounding_box == None:
			u = np.nan
			v = np.nan
			w = np.nan
		
		else:
			u = (bounding_box[0][0]+bounding_box[1][0])/2 - 320
			v = (bounding_box[0][1]+bounding_box[1][1])/2 - 240
			w = self.ground_dist

		# # Extrinsic matrix
		# R_drone_world = tf.transformations.quaternion_matrix([self.drone_pose.orientation.x, self.drone_pose.orientation.y, self.drone_pose.orientation.z, self.drone_pose.orientation.w])[:3, :3]  # 3x3 rotation matrix

		# # Rotate the original rotation matrix about the y-axis
		# R_cam_world = np.dot(R_drone_world, self.R_camera_drone)
		
		# p_drone_world = np.array([self.drone_pose.position.x, self.drone_pose.position.y, self.drone_pose.position.z])
		# p_camera_world = p_drone_world + R_drone_world @ self.P_camera_drone
		# T = np.concatenate([np.concatenate([R_cam_world, p_camera_world[:, np.newaxis]], axis=1), np.array([[0, 0, 0, 1]])], axis=0)

		# # Normalize pixel coordinates
		# u_norm = (u - 320.5) / 319.9988245765257
		# v_norm = (v - 240.5) / 319.9988245765257

		# # Direction vector in camera coordinates
		# P_c = np.array([u_norm, v_norm, 1])
		# P_c = P_c / np.linalg.norm(P_c)

		# # 3D position in world coordinates
		# P_w = np.dot(np.linalg.inv(T), np.concatenate([P_c, [1]]))
		# P_world = P_w[:3]

		# Create a PointStamped message for the position of the object in the drone frame
		object_msg = PointStamped()
		object_msg.header.frame_id = 'world'
		object_msg.header.stamp = rospy.Time.now()
		object_msg.point.x = u #P_world[0]
		object_msg.point.y = v #P_world[1]
		object_msg.point.z = w #P_world[2]

		# Publish the position of the object in the drone frame
		self.publisher.publish(object_msg)

		# rospy.loginfo('Object position in world frame: x = {}, y = {}, z = {}'.format(u, v, w))

	# def get_sensor_pod_tip_pose(self):
	# 	img = self.get_latest_image()
	# 	bounding_box = self.cd_color_segmentation(img)
	# 	x = (bounding_box[0][0]+bounding_box[1][0])/2
	# 	y = bounding_box[0][1]
	# 	z = 1

	# 	return [x, y, z]
	
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


	# def object_callback(self, msg):
	# 	img = self.bridge.imgmsg_to_cv2(msg)
	# 	bounding_box = self.cd_color_segmentation(img)
	# 	x = (bounding_box[0][0]+bounding_box[1][0])/2
	# 	y = bounding_box[0][1]
	# 	z = self.ground_dist

    #     # Check if we have received the drone pose
	# 	if self.drone_pose is None:
	# 		rospy.logwarn('No drone pose received yet. Cannot transform object position.')
	# 		return

    #     # Convert the position of the object to a numpy array
	# 	P_camera = np.array([x, y, z]) #np.array([msg.point.x, msg.point.y, msg.point.z, 1.0])

    #     # Convert the quaternion orientation of the drone to a rotation matrix
	# 	q = self.drone_pose.pose.orientation
	# 	r = np.array([
    #         [1 - 2*q.y*q.y - 2*q.z*q.z, 2*q.x*q.y - 2*q.z*q.w, 2*q.x*q.z + 2*q.y*q.w],
    #         [2*q.x*q.y + 2*q.z*q.w, 1 - 2*q.x*q.x - 2*q.z*q.z, 2*q.y*q.z - 2*q.x*q.w],
    #         [2*q.x*q.z - 2*q.y*q.w, 2*q.y*q.z + 2*q.x*q.w, 1 - 2*q.x*q.x - 2*q.y*q.y]
    #     ])

    #     # Create a homogeneous transformation matrix from the rotation matrix
	# 	t = np.eye(4)
	# 	t[:3,:3] = r

    #     # Transform the position of the object from the camera frame to the drone frame
	# 	P_drone = np.dot(np.linalg.inv(t), P_camera)
	# 	P_drone = P_drone[:3] / P_drone[3]

    #     # Add the position of the drone in the world frame to get the position of the object in the world frame
	# 	P_drone_world = np.array([self.drone_pose.pose.position.x, self.drone_pose.pose.position.y, self.drone_pose.pose.position.z])
	# 	P_world = P_drone[:3] + P_drone_world

	# 	# Create a PointStamped message for the position of the object in the drone frame
	# 	object_msg = PointStamped()
	# 	object_msg.header.frame_id = 'world'
	# 	object_msg.header.stamp = rospy.Time.now()
	# 	object_msg.point.x = P_world[0]
	# 	object_msg.point.y = P_world[1]
	# 	object_msg.point.z = P_world[2]

	# 	# Publish the position of the object in the drone frame
	# 	self.publisher.publish(object_msg)

	# 	rospy.loginfo('Object position in world frame: x = {}, y = {}, z = {}'.format(P_world[0], P_world[1], P_world[2]))


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
