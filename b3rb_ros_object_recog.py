import rclpy
from rclpy.node import Node
from synapse_msgs.msg import TrafficStatus

import cv2
import numpy as np


from sensor_msgs.msg import CompressedImage

QOS_PROFILE_DEFAULT = 10

#stop_sign_cascade = cv2.CascadeClassifier('cascade_stop_sign.xml')
from ultralytics import YOLO
#model_path = "best(1).pt"  #path to the weights file
import os

#path = os.path.join(os.path.dirname(__file__),'best(1).pt')
path = '/home/gitaansh/cognipilot/cranium/src/b3rb_ros_line_follower/b3rb_ros_line_follower/best(1).pt'
model = YOLO(path)
#current_dir = os.path.dirname(os.path.abspath(__file__))
#path = '~/cognipilot/cranium/src/b3rb_ros_line_follower/b3rb_ros_line_follower/cascade_stop_sign.xml'
# stop_sign_cascade = cv2.CascadeClassifier(cascade_path)

class ObjectRecognizer(Node):
	""" Initializes object recognizer node with the required publishers and subscriptions.

		Returns:
			None
	"""
	def __init__(self):
		super().__init__('object_recognizer')

		# Subscription for camera images.
		self.subscription_camera = self.create_subscription(
			CompressedImage,
			'/camera/image_raw/compressed',
			self.camera_image_callback,
			QOS_PROFILE_DEFAULT)

		# Publisher for traffic status.
		self.publisher_traffic = self.create_publisher(
			TrafficStatus,
			'/traffic_status',
			QOS_PROFILE_DEFAULT)

	""" Analyzes the image received from /camera/image_raw/compressed to detect traffic signs.
		Publishes the existence of traffic signs in the image on the /traffic_status topic.

		Args:
			message: "docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CompressedImage.html"

		Returns:
			None
	"""
	def camera_image_callback(self, message):
		# Convert message to an n-dimensional numpy array representation of image.
		np_arr = np.frombuffer(message.data, np.uint8)
		image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

		traffic_status = TrafficStatus()

		# NOTE: participants need to implement logic for recognizing traffic signs.
		# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		# stop_signs = []
		
		# try:
		# 	stop_signs = stop_sign_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))			
		# except:
		# 	pass
		
		# if len(stop_signs) > 0:
		# 	traffic_status.stop_sign = True
		# 	print(stop_signs)
		# else:
		# 	traffic_status.stop_sign = False

		results = model.predict(source=image, imgsz=640, conf=0.25)
		for result in results:
			if result.boxes:  
				#print(result.names)
				#if result.boxes.conf.tolist() > 0.9:
				#print('Detected Stop Sign')
				print("Confidences:", result.boxes.conf.tolist()[0]) 
				result.boxes.conf
				#if result.boxes.conf.tolist() 
		
		self.publisher_traffic.publish(traffic_status)

'''        # For visualization purposes (optional)
        for (x, y, w, h) in stop_signs:
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(image, 'Stop Sign', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)'''


def main(args=None):
	rclpy.init(args=args)

	object_recognizer = ObjectRecognizer()

	rclpy.spin(object_recognizer)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	object_recognizer.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
