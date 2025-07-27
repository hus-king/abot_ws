#!/usr/bin/env python
#pin1-3.3v,pin14-gnd,pin15-pwm
import rospy
from std_msgs.msg import String
import Jetson.GPIO as GPIO
import time 

SERVO_PIN = 15
PWM_FREQ = 50

def angle_to_duty(angle):
	angle = max(0,min(180,angle))
	return 2.5 + (angle/180)*10
	


def servo_control_callback(msg):
    """回调函数，处理接收到的舵机控制命令"""
    rospy.loginfo(f"接收到舵机控制命令: {msg.data}")
    
    # 在这里添加舵机控制的代码
    if msg.data.lower() == "open":
        rospy.loginfo("启动舵机打开任务...")
        pwm.ChangeDutyCycle(angle_to_duty(135))
    else:
        rospy.loginfo("未知命令，忽略")

def servo_control_subscriber():
    """初始化ROS节点并订阅舵机控制话题"""
    # 初始化ROS节点，命名为'servo_control_subscriber'
    rospy.init_node('servo_control_subscriber', anonymous=True)
    
    
    # 创建一个订阅者，订阅'servo_control_topic'话题，消息类型为String
    # 接收到消息后调用servo_control_callback函数进行处理
    rospy.Subscriber('servo_control_topic', String, servo_control_callback)
    
    # 进入循环等待消息
    rospy.loginfo("舵机控制节点已启动，等待命令...")
    rospy.spin()

if __name__ == '__main__':
    try:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SERVO_PIN,GPIO.OUT)

        pwm = GPIO.PWM(SERVO_PIN,PWM_FREQ)
        pwm.start(0)
        pwm.ChangeDutyCycle(angle_to_duty(0))
        servo_control_subscriber()
        
        pwm.stop()
        GPIO.cleanup()

    except rospy.ROSInterruptException:
        rospy.loginfo("节点关闭")
