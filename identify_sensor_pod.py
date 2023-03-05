import cv2
import numpy as np
import pdb
import sys

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

sensor_pod_template_path = 'sensor_pod_'

def image_print(img):
		"""
		Helper function to print out images, for debugging. Pass them in as a list.
		Press any key to continue
		"""
		cv2.imshow("image", img)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

def filter_contours(contours):
	if len(contours) == 0:
		return None
    
	return [contour for contour in contours if cv2.contourArea(contour) > 1000] # WILL NEED TO CHANGE

def get_largest_contour(contours):
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

def cd_color_segmentation(img, template=None):
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
	image_print(mask)

	# Find remaning contours, correspond to orange objects
	contours = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
	largest_contour = get_largest_contour(contours)

	# Draw boxes around the contours
	x, y, w, h = cv2.boundingRect(largest_contour)
	bounding_box = ((x, y), (x+w, y+h))

	boxes = []
	for c in contours:
		x, y, w, h = cv2.boundingRect(c)
		boxes.append(((x, y), (x+w, y+h)))

	return bounding_box

if __name__ == '__main__':

	for i in range(10):
		# image_print(cone_image)

		cone_image = cv2.imread(sensor_pod_template_path + str(i) + ".jpg")

		boundary_box = cd_color_segmentation(cone_image)
		cv2.rectangle(cone_image, boundary_box[0], boundary_box[1], (0, 255, 0),2)

		# boxes = cd_color_segmentation(cone_image)

		# for b in boxes:
		# 	cv2.rectangle(cone_image, b[0], b[1], (0, 255, 0),2)
		image_print(cone_image)