import cv2
import numpy as np
import pdb
import sys
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import glob
import os

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
		self.subscriber = rospy.Subscriber("/pod_images/sensor_pod_0.jpg", Image, self.callback)
		self.publisher = rospy.Publisher(SensorPodIdentifier.POD_LOCATION_TOPIC, PoseStamped, queue_size=10)
		self.latest_image = None
		self.bridge = CvBridge()

	def callback(self, Image_data):
		self.latest_image = self.bridge.imgmsg_to_cv2(Image_data, "bgr8")

	def get_latest_image(self):
		return self.latest_image

	def image_print(self):
			"""
			Helper function to print out images, for debugging. Pass them in as a list.
			Press any key to continue
			"""
			cv2.imshow("image", self.get_latest_image())
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
		light_orange = (40, 100, 105) #(70, 180, 150)
		dark_orange = (170, 255, 255) #(150, 255, 255)
		kernel = np.ones((7, 7), np.uint8)

		filtered_img = cv2.dilate(cv2.erode(img, kernel, iterations=1), kernel, iterations=1)

		#image_print(filtered_img)
		hsv_img = cv2.cvtColor(filtered_img, cv2.COLOR_RGB2HSV)
		mask = cv2.inRange(hsv_img, light_orange, dark_orange)
		self.image_print(mask)

		# Find remaning contours, correspond to orange objects
		contours = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
		largest_contour = self.get_largest_contour(contours)

		# Draw boxes around the contours
		x, y, w, h = cv2.boundingRect(largest_contour)
		bounding_box = ((x, y), (x+w, y+h))

		boxes = []
		for c in contours:
			x, y, w, h = cv2.boundingRect(c)
			boxes.append(((x, y), (x+w, y+h)))

		return bounding_box
	
	def get_sensor_pod_tip_pose(self):
		img = self.get_latest_image()
		bounding_box = self.cd_color_segmentation(img)
		x = (bounding_box[0][0]+bounding_box[1][0])/2
		y = bounding_box[0][1]
		z = 1

		return (x, y, z)
		
	def publish_sensor_pod_tip_pose(self):
		pod_tip = self.get_sensor_pod_tip_pose()
		pose_msg = PoseStamped()
		pose_msg.header.frame_id = "world"
		pose_msg.pose.position.x = pod_tip[0]
		pose_msg.pose.position.y = pod_tip[1]
		pose_msg.pose.position.z = pod_tip[2]
		pose_msg.pose.orientation.x = 0.0
		pose_msg.pose.orientation.y = 0.0
		pose_msg.pose.orientation.z = 0.0
		pose_msg.pose.orientation.w = 1.0

		self.publisher.publish(pose_msg)


if __name__ == '__main__':

	rospy.init_node('sensor_pod_identifier')
	sensor_pod_identifier = SensorPodIdentifier()
	rospy.spin()

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