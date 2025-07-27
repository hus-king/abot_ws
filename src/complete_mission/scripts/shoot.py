#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO
import threading
import time

LASER_PIN = 33
# 声明全局变量跟踪GPIO模式设置状态
GPIO_MODE_SET = False
# 添加线程锁确保状态一致性
STATE_LOCK = threading.Lock()

def setup_gpio():
    """设置GPIO模式并初始化引脚"""
    global GPIO_MODE_SET
    if not GPIO_MODE_SET:
        try:
            GPIO.setmode(GPIO.BOARD)  # 使用BOARD编号模式
            GPIO.setup(LASER_PIN, GPIO.OUT, initial=GPIO.LOW)
            # 确保初始状态为低电平
            GPIO.output(LASER_PIN, GPIO.LOW)
            GPIO_MODE_SET = True
            rospy.loginfo("GPIO模式已设置，激光笔引脚初始化完成")
        except Exception as e:
            rospy.logerr(f"GPIO设置失败: {e}")

class LaserController:
    def __init__(self):
        rospy.init_node("laser_driver_node")

        self.control_sub = rospy.Subscriber("/shoot_control_topic", Bool, self.control_callback)
        setup_gpio()

        self.shot_count = 0  # 记录发射次数
        self.max_shots = 5   # 最大发射次数
        self.shot_interval = 0.5  # 发射间隔(秒)
        self.shooting = False  # 是否正在发射
        self.shooting_thread = None  # 发射线程
        
        rospy.on_shutdown(self.cleanup)

    def control_callback(self, msg):
        with STATE_LOCK:
            if msg.data and not self.shooting and self.shot_count < self.max_shots:
                rospy.loginfo("接收到打靶开启信号，开始发射激光")
                self.shooting = True
                self.shooting_thread = threading.Thread(target=self.shoot_sequence)
                self.shooting_thread.daemon = True
                self.shooting_thread.start()

    def shoot_sequence(self):
        """执行五次激光发射序列"""
        try:
            while self.shot_count < self.max_shots and not rospy.is_shutdown():
                self.turn_on_laser()
                time.sleep(0.1)  # 激光开启时间
                self.turn_off_laser()
                self.shot_count += 1
                rospy.loginfo(f"完成第 {self.shot_count}/{self.max_shots} 次激光发射")
                if self.shot_count < self.max_shots:
                    time.sleep(self.shot_interval)  # 间隔时间
            self.shooting = False
            rospy.loginfo("激光发射序列完成，永久关闭激光")
        except Exception as e:
            rospy.logerr(f"激光发射序列异常: {e}")
            self.shooting = False

    def turn_on_laser(self):
        try:
            GPIO.output(LASER_PIN, GPIO.HIGH)
            self.laser_on = True
        except Exception as e:
            rospy.logerr(f"打开激光失败: {e}")

    def turn_off_laser(self):
        try:
            GPIO.output(LASER_PIN, GPIO.LOW)
            self.laser_on = False
        except Exception as e:
            rospy.logerr(f"关闭激光失败: {e}")

    def cleanup(self):
        with STATE_LOCK:
            self.shooting = False
            self.turn_off_laser()
            if GPIO_MODE_SET:
                GPIO.cleanup()
                rospy.loginfo("GPIO资源已清理")

if __name__ == '__main__':
    LaserController()
    rospy.spin()