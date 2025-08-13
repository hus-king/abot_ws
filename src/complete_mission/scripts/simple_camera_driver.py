#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def main(argv):
    rospy.init_node("simple_camera_driver", argv=argv)
    camera_id = rospy.get_param("~camera_id", 0)
    frame_id = rospy.get_param("~frame_id", "mono_camera")
    rate_duration = rospy.get_param("~rate", 10)
    
    image_pub = rospy.Publisher("camera/image_raw", Image, queue_size=1)
    
    # 尝试打开摄像头，如果0打不开就尝试1
    capture = cv2.VideoCapture(camera_id)
    if not capture.isOpened():
        rospy.logwarn(f"摄像头 {camera_id} 无法打开，尝试摄像头 1")
        camera_id = 1
        capture = cv2.VideoCapture(camera_id)
        if not capture.isOpened():
            rospy.logerr("摄像头 0 和 1 都无法打开，程序退出")
            return
        else:
            rospy.loginfo(f"成功打开摄像头 {camera_id}")
    else:
        rospy.loginfo(f"成功打开摄像头 {camera_id}")
    
    # <<< --- 添加：设置摄像头分辨率为 640x480
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 可选：验证是否设置成功
    width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
    rospy.loginfo(f"摄像头分辨率设置为: {int(width)}x{int(height)}")

    # --- >>>
    
    bridge = CvBridge()
    rate = rospy.Rate(rate_duration)
    while not rospy.is_shutdown():
        ret, frame = capture.read()
        if ret:
            try:
                image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                image_msg.header.stamp = rospy.Time.now()
                image_msg.header.frame_id = frame_id
                image_pub.publish(image_msg)
            except CvBridgeError as e:
                rospy.logerr(e)
        rate.sleep()


if __name__ == '__main__':
    main(sys.argv)