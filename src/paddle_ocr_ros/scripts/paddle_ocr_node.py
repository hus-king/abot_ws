#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from paddle_ocr_ros.msg import OCRResult, OCRItem
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from paddleocr import PaddleOCR
import os
os.environ['FLAGS_use_mkldnn'] = '0'  # 禁用 MKLDNN
class PaddleOCRNode:
    def __init__(self):
        rospy.init_node('paddle_ocr_node')
        
        # 初始化参数
        det_model = rospy.get_param('~det_model_dir', 'en_PP-OCRv3_det_infer')
        rec_model = rospy.get_param('~rec_model_dir', 'en_PP-OCRv3_rec_infer')
        image_topic = rospy.get_param('~image_topic', '/image_raw')
        pub_topic = rospy.get_param('~pub_topic', '/ocr')
        ctrl_topic = rospy.get_param('~ctrl_topic', '/ocr_ctrl')
        self.visualize = rospy.get_param('~visualize', False)
        
        # 转换为绝对路径
        det_model = os.path.abspath(det_model)
        rec_model = os.path.abspath(rec_model)
        
        # 验证路径存在
        if not os.path.exists(det_model):
            rospy.logerr(f"检测模型路径不存在: {det_model}")
        if not os.path.exists(rec_model):
            rospy.logerr(f"识别模型路径不存在: {rec_model}")
        
        rospy.loginfo(f"使用检测模型: {det_model}")
        rospy.loginfo(f"使用识别模型: {rec_model}")
        
        # 初始化OCR (使用更新后的参数名)
        self.ocr = PaddleOCR(
            use_textline_orientation=False,  # 替代 use_angle_cls
            lang='en',
            det_model_dir=det_model,  # 替代 det_model_dir text_detection_model_dir
            rec_model_dir=rec_model,  # 替代 rec_model_dir text_recognition_model_dir
            enable_mkldnn=False  # 显式禁用 MKLDNN
        )
        
        # 初始化ROS组件
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.ctrl_sub = rospy.Subscriber(ctrl_topic, String, self.ctrl_callback)
        self.ocr_pub = rospy.Publisher(pub_topic, OCRResult, queue_size=10)
        self.ctrl_flag = False
        rospy.loginfo("PaddleOCR节点已启动，等待图像输入...")
        
    def ctrl_callback(self, msg):
        if msg.data.lower() == "true":
            self.ctrl_flag = True
        else:
            self.ctrl_flag = False
            if self.visualize:
                cv2.destroyAllWindows()  # 停止可视化时关闭所有窗口
    
    def image_callback(self, msg):
        if not self.ctrl_flag:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("图像转换失败: %s", str(e))
            return

        results = self.ocr.ocr(cv_image, cls=False)
        ocr_result = OCRResult()
        ocr_result.header = msg.header

        if results is not None and results[0] is not None and isinstance(results, list):
            for result in results[0]:
                if not isinstance(result, list) or len(result) < 2:
                    rospy.logwarn("OCR结果结构异常: %s", result)
                    continue

                points, text_info = result

                if not isinstance(points, (list, np.ndarray)) or len(points) != 4:
                    continue

                if not isinstance(text_info, (tuple, list)) or len(text_info) < 2:
                    continue

                item = OCRItem()
                item.text = text_info[0]
                item.confidence = float(text_info[1])

                # 将小c转换为大C
                if item.text == "c":
                    item.text = "C"
                    
                # confidence 阈值过滤（可选）
                # if item.confidence < 0.5:
                    #continue
                
                for i in range(4):
                    
                    pt = points[i]
                    rospy.logwarn(pt)
                    point = Point()
                    point.x = float(pt[0])
                    point.y = float(pt[1])
                    point.z = 0.0
                    item.polygon[i] = point
                    
                rospy.logwarn(f"{points}")
                rospy.logwarn(f"{item.polygon}")
                ocr_result.items.append(item)

                # 可视化调试信息（可选）
                if self.visualize:
                    pts = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
                    cv2.polylines(cv_image, [pts], True, (0, 255, 0), 2)
                    cv2.putText(cv_image, f"{item.text} ({item.confidence:.2f})",
                            (int(points[0][0]), int(points[0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                            
        if self.visualize:
            cv2.imshow("ocr",cv_image)
            cv2.waitKey(1)

        self.ocr_pub.publish(ocr_result)
        

if __name__ == '__main__':
    try:
        node = PaddleOCRNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
