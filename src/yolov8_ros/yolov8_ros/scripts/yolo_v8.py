#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
import numpy as np
from ultralytics import YOLO
from time import time
from cv_bridge import CvBridge,CvBridgeError
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes


class Yolo_Dect:
    def __init__(self):

        # load parameters
        weight_path = rospy.get_param('~weight_path', '')
        image_topic = rospy.get_param(
            '~image_topic', '/camera/image_raw')
        pub_topic = rospy.get_param('~pub_topic', '/yolov8/BoundingBoxes')
        self.camera_frame = rospy.get_param('~camera_frame', '')
        conf = rospy.get_param('~conf', '0.5')
        self.visualize = rospy.get_param('~visualize', 'True')
        use_cpu = rospy.get_param('/use_cpu', 'false')
        # which device will be used
        if (use_cpu):
            self.device = 'cpu'
        else:
            self.device = 'cuda'
        self.bridge = CvBridge()
        self.model = YOLO(weight_path)
        #self.model.fuse()

        self.model.conf = conf
        self.color_image = Image()
        self.getImageStatus = False

        # Load class color
        self.classes_colors = {}

        # image subscribe
        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback,
                                          queue_size=1 ,buff_size=52428800)
    #buff_size=52428800
        # output publishers
        self.position_pub = rospy.Publisher(
            pub_topic,  BoundingBoxes, queue_size=1)

        self.image_pub = rospy.Publisher(
            '/yolov8/detection_image',  Image, queue_size=1)

        # if no image messages
        while (not self.getImageStatus):
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)

    def image_callback(self, image):

        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header
        self.getImageStatus = True
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)

        #self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)
        #self.color_image = self.bridge.imgmsg_to_cv2(image,"bgr8")
        results = self.model(self.color_image)
        
        self.dectshow(results, image.height, image.width)

        cv2.waitKey(3)

    def dectshow(self, results, height, width):

        self.frame = results[0].plot()
        print(str(results[0].speed['inference']))
        fps = 1000.0/ results[0].speed['inference']
        cv2.putText(self.frame, f'FPS: {int(fps)}', (20,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        # 打印检测到的类别信息
        if len(results[0].boxes) > 0:
            detected_classes = []
            for result in results[0].boxes:
                class_name = results[0].names[result.cls.item()]
                confidence = result.conf.item()
                detected_classes.append(f"{class_name}({confidence:.2f})")
            
            print(f"检测到的类别: {', '.join(detected_classes)}")
        else:
            print("未检测到任何目标")

        for result in results[0].boxes:
            boundingBox = BoundingBox()
            sum1 = np.int64(result.xyxy[0][0].item()/2) + np.int64(result.xyxy[0][2].item()/2)
            sum2 = np.int64(result.xyxy[0][1].item()/2) + np.int64(result.xyxy[0][3].item()/2) 
            boundingBox.xmin = sum1
            boundingBox.ymin = sum2
            boundingBox.xmax = 0
            boundingBox.ymax = 0
            
            boundingBox.Class = results[0].names[result.cls.item()]
            boundingBox.probability = result.conf.item()
            self.boundingBoxes.bounding_boxes.append(boundingBox)
        self.position_pub.publish(self.boundingBoxes)
        self.publish_image(self.frame, height, width)

        if self.visualize :
            cv2.imshow('YOLOv8', self.frame)

    def publish_image(self, imgdata, height, width):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = self.camera_frame
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.image_pub.publish(image_temp)


def main():
    import pkg_resources
    import ultralytics
    rospy.init_node('yolov8_ros', anonymous=True)
    # 输出ultralytics及其依赖包版本
    print("ultralytics version:", ultralytics.__version__)
    print("ultralytics dependencies:")
    try:
        dist = pkg_resources.get_distribution("ultralytics")
        requires = dist.requires()
        for req in requires:
            try:
                dep_dist = pkg_resources.get_distribution(req.project_name)
                print(f"  {req.project_name}: {dep_dist.version}")
            except Exception:
                print(f"  {req.project_name}: not installed")
    except Exception as e:
        print("Could not get ultralytics dependencies:", e)
    yolo_dect = Yolo_Dect()
    rospy.spin()


if __name__ == "__main__":

    main()
