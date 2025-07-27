#!/usr/bin/env python
#pin17-3.3v,pin34-gnd,pin33-pwm
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO
import time 

LASER_PIN = 33
# 声明全局变量跟踪GPIO模式设置状态
GPIO_MODE_SET = False
SHOOT_MODE = False

def setup_gpio():
    """设置GPIO模式并初始化引脚"""
    global GPIO_MODE_SET
    if not GPIO_MODE_SET:
        try:
            GPIO.setmode(GPIO.BOARD)  # 使用BOARD编号模式
            GPIO.setup(LASER_PIN, GPIO.OUT, initial=GPIO.LOW)
            time.sleep(1)
            GPIO.output(LASER_PIN, GPIO.LOW)
            time.sleep(1)
            GPIO_MODE_SET = True
            rospy.loginfo("GPIO模式已设置，激光笔引脚初始化完成")
        except Exception as e:
            rospy.logerr(f"GPIO设置失败: {e}")

def shoot_control_callback(msg):
    """回调函数，处理接收到的舵机控制命令"""
    rospy.loginfo(f"接收到舵机控制命令: {msg.data}")
    
    if msg.data:
        rospy.loginfo("shooting...")
        SHOOT_MODE = True
    else:
        SHOOT_MODE = False
        
        
    # 不要在每次回调后都cleanup，这会重置GPIO设置
    # GPIO.cleanup()

def shoot_control_subscriber():
    """初始化ROS节点并订阅舵机控制话题"""
    # 初始化ROS节点，命名为'servo_control_subscriber'
    rospy.init_node('shoot_control_subscriber', anonymous=True)
    
    # 创建一个订阅者，订阅'servo_control_topic'话题，消息类型为String
    rospy.Subscriber('shoot_control_topic', Bool, shoot_control_callback)
    
    # 进入循环等待消息
    rospy.loginfo("shoot控制节点已启动，等待命令...")
    while True:
        if SHOOT_MODE:
            GPIO.output(LASER_PIN, GPIO.HIGH)
            print("open")
            time.sleep(1)
            GPIO.output(LASER_PIN, GPIO.LOW)
            time.sleep(1)
            print("close")
        else:
            GPIO.output(LASER_PIN, GPIO.LOW)
            time.sleep(1)
            rospy.loginfo("未知命令，忽略")
        rospy.spinOnce()

if __name__ == '__main__':
    try:
        setup_gpio()  # 在主程序中先设置GPIO
        shoot_control_subscriber()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点关闭")
    finally:
        # 程序退出时清理GPIO
        if GPIO_MODE_SET:
            GPIO.cleanup()
            rospy.loginfo("GPIO资源已清理")
