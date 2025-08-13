#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar import pyzbar
from std_msgs.msg import String

class QRCodeReader:
    def __init__(self):
        self.bridge = CvBridge()
        
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback,queue_size=1)
        self.qr_pub = rospy.Publisher("/qr_code_data", String, queue_size=10)
    def image_callback(self, image):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
           
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Detect and decode QR codes
        decoded_objects = pyzbar.decode(cv_image)
        for obj in decoded_objects:
            self.qr_pub.publish(obj.data.decode('utf-8'))


def main():
    rospy.init_node('qr_code_reader', anonymous=True)
    qr_code_reader = QRCodeReader()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
