#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

class DoorRectangleDrawer:
    def __init__(self):
        rospy.init_node('door_rectangle_drawer', anonymous=True)
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 门点坐标初始化
        self.left_door_point = 0
        self.right_door_point = 0
        
        # 订阅门点坐标
        rospy.Subscriber('/door_points', Point, self.door_points_callback)
        
        # 订阅图像话题
        rospy.Subscriber('/camera_processor/color/image_raw', Image, self.image_callback)
        
        # 发布处理后的图像到 /door_pub
        self.image_pub = rospy.Publisher('/door_pub', Image, queue_size=10)
        
        rospy.loginfo("Door Rectangle Drawer initialized")
    
    def door_points_callback(self, msg):
        """处理门点坐标的回调函数"""
        self.left_door_point = int(msg.x)
        self.right_door_point = int(msg.y)
        rospy.loginfo(f"Received door points: left={self.left_door_point}, right={self.right_door_point}")
    
    def image_callback(self, msg):
        """处理图像的回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 绘制矩形
            # 左上角点: (left_door_point, 230)
            # 右下角点: (right_door_point, 250)
            pt1 = (self.left_door_point, 230)
            pt2 = (self.right_door_point, 250)
            
            # 绘制绿色矩形，线宽为2
            cv2.rectangle(cv_image, pt1, pt2, (0, 255, 0), 2)
            
            # 将OpenCV图像转换回ROS格式并发布
            processed_img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(processed_img_msg)
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        drawer = DoorRectangleDrawer()
        drawer.run()
    except rospy.ROSInterruptException:
        pass