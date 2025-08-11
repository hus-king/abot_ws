#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <yolov8_ros_msgs/BoundingBoxes.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <deque>
#include <image_transport/image_transport.h>
#include <camera_processor/PointDepth.h>

using namespace std;


#define ALTITUDE 0.7
#define VIEW_ALTITUDE 1.6
#define DOOR_ALTITUDE 0.3
#define LANDING_ALTITUDE -0.15
#define CATAPULT_ALTITUDE 0.05
#define QR_ALTITUDE 0.40
#define RING_HEIGHT 1.50
#define TANK_ALTITUDE 1.9
#define PUT_TANK_ALTITUDE 1.5
float camera_offset_body_x = 0.0; // 相机安装偏移量，单位米

mavros_msgs::PositionTarget setpoint_raw;
ros::Publisher planner_goal_pub;
ros::Publisher finish_ego_pub;
ros::Publisher ego_planner_mode_pub;
ros::Publisher door_points_pub;
std_msgs::Bool finish_ego_flag;
std_msgs::Int32 ego_planner_mode;
int now_mode = -1 ;

int spin_flag = 0 ;

float map_size_z = 0.8;
float ground_height = 0.8;
float ego_now_x = 0;
float ego_now_y = 0;
yolov8_ros_msgs::BoundingBox cb;
float camera_height;
float square_yaw_cb;
float now_target_x = 0;
float now_target_y = 0;
float door_step = 2;
float door_shred = 1;
int door_adjust_range = 70;
float landing_gear;

string target_1 = "car";
string target_2 = "bridge";
string target_3 = "pillbox";

bool target_1_found = false;
bool target_2_found = false;
bool target_3_found = false;

double calculate_abs_distance(double x1,double y1,double x2,double y2)
{
	return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

/************************************************************************
函数功能1：无人机状态回调函数
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
	current_state = *msg;
	//	std::cout << "ok " << std::endl;
}

/************************************************************************
函数功能2：回调函数接收无人机的里程计信息
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
tf::Quaternion quat;
nav_msgs::Odometry local_pos;
double roll, pitch, yaw;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	local_pos = *msg;
	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	square_yaw_cb = yaw;
	if (flag_init_position == false && (local_pos.pose.pose.position.z != 0))
	{
		init_position_x_take_off = local_pos.pose.pose.position.x;
		init_position_y_take_off = local_pos.pose.pose.position.y;
		init_position_z_take_off = local_pos.pose.pose.position.z;
		init_yaw_take_off = yaw;
		flag_init_position = true;
	}
}


/************************************************************************
函数功能3：自主巡航，发布目标位置，控制无人机到达目标，采用local坐标系位置控制
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
bool mission_pos_cruise_flag = false;
bool mission_pos_cruise(float x, float y, float z, float yaw, float error_max);
bool mission_pos_cruise(float x, float y, float z, float yaw, float error_max)
{
	if (mission_pos_cruise_flag == false)
	{
		mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
		mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
		mission_pos_cruise_flag = true;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = x + init_position_x_take_off;
	setpoint_raw.position.y = y + init_position_y_take_off;
	setpoint_raw.position.z = z + init_position_z_take_off;
	setpoint_raw.yaw = yaw;

	ROS_INFO("now_position:(%f,%f)  Flying to ( %.2f, %.2f )", local_pos.pose.pose.position.x ,local_pos.pose.pose.position.y,x, y);
	ROS_INFO("target_yaw: %.2f", yaw * 180.0 / M_PI);
	ROS_INFO("now_yaw: %.2f", square_yaw_cb * 180.0 / M_PI);

	if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max && fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < error_max)
	{
		ROS_INFO("到达目标点，巡航点任务完成");
		mission_pos_cruise_flag = false;
		return true;
	}
	return false;
}



/************************************************************************
函数功能4：自主巡航，发布目标位置，控制无人机到达目标，采用机体坐标系位置控制
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float current_position_cruise_last_position_x = 0;
float current_position_cruise_last_position_y = 0;
bool current_position_cruise_flag = false;
bool current_position_cruise(float x, float y, float z, float yaw, float error_max);
bool current_position_cruise(float x, float y, float z, float yaw, float error_max)
{
	if (current_position_cruise_flag == false)
	{
		current_position_cruise_last_position_x = local_pos.pose.pose.position.x;
		current_position_cruise_last_position_y = local_pos.pose.pose.position.y;
		current_position_cruise_flag = true;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 + 1024 /*+ 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = current_position_cruise_last_position_x + x;
	setpoint_raw.position.y = current_position_cruise_last_position_y + y;
	setpoint_raw.position.z = z;
	double current_yaw, a, b;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(a, b, current_yaw);
	ROS_WARN("current_yaw: %.2f", current_yaw * 180.0 / M_PI);
	float yaw_diff = yaw - current_yaw;
	if (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
	else if (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
	setpoint_raw.yaw_rate = yaw_diff * 0.5; // 设置yaw_rate，P控制
	if (fabs(local_pos.pose.pose.position.x - current_position_cruise_last_position_x - x) < error_max && fabs(local_pos.pose.pose.position.y - current_position_cruise_last_position_y - y) < error_max && fabs(local_pos.pose.pose.position.z - z) < error_max && fabs(current_yaw - yaw) < 0.1)
	{
		ROS_INFO("到达目标点，巡航点任务完成");
		current_position_cruise_flag = false;
		setpoint_raw.yaw_rate = 0.0; // 重置yaw_rate
		setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 + /*1024 + */2048;
		setpoint_raw.yaw = yaw; // 确保最终yaw角度正确
		return true;
	}
	return false;
}


/************************************************************************
函数功能: yolov8
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
bool start_checking = false;  // YOLO检测启动标志：true=开始目标检测，false=停止检测
bool only_tank = false; // 仅检测tank目标标志：true=只检测tank，false=检测target_1和target_2和target_3
bool found = false;           // 固定目标发现标志：累计检测到target_1或target_2或target_3达到阈值次数时为true
bool found_false = false;     // 错误目标发现标志：连续检测到非期望目标达到阈值次数时为true
bool found_tank = false;      // 移动目标发现标志：累计检测到tank达到阈值次数时为true

std::string have_found;       // 最近检测到的目标类别名称
float box_target_x;           // 目标在世界坐标系中的X坐标（米）
float box_target_y;           // 目标在世界坐标系中的Y坐标（米）
float fx = 531.256452402983;
float fy = 531.062457146004;
float cx = 331.091420250469;
float cy = 248.782850955389;

std::string target_data[3] = {target_1,target_2,target_3};  // 目标检测类别数组，只检测车辆和桥梁和碉堡三种目标

// 累计检测计数器 - 用于消除误检，提高检测可靠性
int target_1_detect_count = 0;        // 车辆目标累计检测次数计数器
int target_2_detect_count = 0;     // 桥梁目标累计检测次数计数器
int target_3_detect_count = 0;    // 碉堡目标累计检测次数计数器
int tank_detect_count = 0;       // 坦克目标累计检测次数计数器（用于移动目标跟踪）
int false_detect_count = 0;      // 错误目标检测计数器（检测到非期望目标时递增）
const int DETECT_THRESHOLD = 10; // 目标确认阈值：需要累计检测10次才确认找到目标（消除误检）
const int FALSE_THRESHOLD = 10;  // 错误检测阈值：连续10次错误检测则found_false为true（避开干扰目标）

void yolo_ros_cb(const yolov8_ros_msgs::BoundingBoxes::ConstPtr &msg){    
    if(!start_checking) {
        // 重置检测计数器
        target_1_detect_count = 0;
        target_2_detect_count = 0;
		target_3_detect_count = 0;
        tank_detect_count = 0;
        false_detect_count = 0;
        return;
    }
	
    // 临时标记本次是否检测到目标
    bool target_1_detected_this_frame = false;
    bool target_2_detected_this_frame = false;
	bool target_3_detected_this_frame = false;
    bool tank_detected_this_frame = false;
    
    for(yolov8_ros_msgs::BoundingBox bounding_box:msg->bounding_boxes)
    {        
		if(bounding_box.probability < 0.85) continue; //保险误发送，重新检验置信度
    	std::cout<<"CLASS: "<<bounding_box.Class<<std::endl;
		cb.Class = bounding_box.Class;
		std::cout<<"probability: "<<bounding_box.probability<<std::endl;
		cb.probability = bounding_box.probability;
		if(bounding_box.Class.empty()) continue; // 如果类别为空，跳过该框
        
        // 检查是否为target_1或target_2或target_3且之前未找到
		bool is_target_found = false; // 标记本次是否检测到期望目标
		for(int i = 0; i < 3; i++){
			if(only_tank) break; // 如果仅检测tank，跳过其他目标
            string target = target_data[i];
			if(target == bounding_box.Class)
			{
                // 检查是否已经找到过该目标
                if((target == target_1 && target_1_found) || (target == target_2 && target_2_found) || (target == target_3 && target_3_found)) {
                    std::cout << "已找到过" << target << "，忽略该检测" << std::endl;
                    is_target_found = false;
                    break;  // 已找到过，不处理
                }
                
				have_found = target;
                
                // 计算目标位置（考虑飞机yaw角度）
				float center_x = bounding_box.xmin;
				float center_y = bounding_box.ymin;
				
				// 相机坐标系下的偏移量（相对于相机光心）
				float camera_offset_x = (cy - center_y) * (local_pos.pose.pose.position.z + camera_height - init_position_z_take_off + landing_gear) / fy + camera_offset_body_x;
				float camera_offset_y = (cx - center_x) * (local_pos.pose.pose.position.z + camera_height - init_position_z_take_off + landing_gear) / fx;
				
				// 考虑飞机yaw角度，将相机坐标系转换到世界坐标系
				float cos_yaw = cos(yaw);
				float sin_yaw = sin(yaw);
				
				// 世界坐标系下的目标位置
				box_target_x = local_pos.pose.pose.position.x + camera_offset_x * cos_yaw - camera_offset_y * sin_yaw;
				box_target_y = local_pos.pose.pose.position.y + camera_offset_x * sin_yaw + camera_offset_y * cos_yaw;
				std::cout << "center_x = " << center_x << ", center_y = " << center_y << std::endl;
				std::cout << "calculate_box_target_x = " << box_target_x << ", calculate_box_target_y = " << box_target_y << std::endl;
				
                // 标记本次检测到该目标
                if(target == target_1) {
                    target_1_detected_this_frame = true;
                } 
				else if(target == target_2) {
                    target_2_detected_this_frame = true;
                }
				else if(target == target_3) {
					target_3_detected_this_frame = true;
				}
                
                is_target_found = true;
				break;
			}
		}
		
		// 单独检测tank（tank不算作有效目标，但需要单独计数）
		if(bounding_box.Class == "tank") {
		    tank_detected_this_frame = true;
			have_found = "tank";
			// 计算目标位置（考虑飞机yaw角度）
			float center_x = bounding_box.xmin;
			float center_y = bounding_box.ymin;
			
			// 相机坐标系下的偏移量（相对于相机光心）
			float camera_offset_x = (cy - center_y) * (local_pos.pose.pose.position.z + camera_height - init_position_z_take_off + landing_gear) / fy;
			float camera_offset_y = (cx - center_x) * (local_pos.pose.pose.position.z + camera_height - init_position_z_take_off + landing_gear) / fx;
			
			// 考虑飞机yaw角度，将相机坐标系转换到世界坐标系
			float cos_yaw = cos(yaw);
			float sin_yaw = sin(yaw);
			
			// 世界坐标系下的目标位置
			box_target_x = local_pos.pose.pose.position.x + camera_offset_x * cos_yaw - camera_offset_y * sin_yaw;
			box_target_y = local_pos.pose.pose.position.y + camera_offset_x * sin_yaw + camera_offset_y * cos_yaw;
			
			std::cout << "tank center_x = " << center_x << ", center_y = " << center_y << std::endl;
			std::cout << "tank calculate_box_target_x = " << box_target_x << ", calculate_box_target_y = " << box_target_y << std::endl;
		}		// 如果不是target_1或target_2或target_3，增加错误检测计数（包括tank）
		if(!is_target_found) {
		    false_detect_count++;
		    std::cout << "本次检测到非目标类别: " << bounding_box.Class << "，错误检测次数: " << false_detect_count << "/" << FALSE_THRESHOLD << std::endl;
		}
    }
    
    // 更新检测计数器
    if(target_1_detected_this_frame) {
        target_1_detect_count++;
		std::cout<< "本次检测到"<<target_1<<",累计次数: " << target_1_detect_count << "/" << DETECT_THRESHOLD << std::endl;
    } else if(target_2_detected_this_frame) {
        target_2_detect_count++;
        std::cout<< "本次检测到"<<target_2<<",累计次数: " << target_2_detect_count << "/" << DETECT_THRESHOLD << std::endl;
    } else if(target_3_detected_this_frame) {
		target_3_detect_count++;
		std::cout<< "本次检测到"<<target_3<<",累计次数: " << target_3_detect_count << "/" << DETECT_THRESHOLD << std::endl;
	}
    // 单独更新tank检测计数器
    if(tank_detected_this_frame) {
        tank_detect_count++;
        std::cout<< "本次检测到tank,累计次数: " << tank_detect_count << "/" << DETECT_THRESHOLD << std::endl;
    }
    
    // 注意：tank仍算作错误目标
    
    // 判断是否达到检测阈值
    if(target_1_detect_count >= DETECT_THRESHOLD && !target_1_found) {
        target_1_found = true;
        found = true;
        std::cout << "累计检测到" << target_1 << "  " << DETECT_THRESHOLD << " 次，确认找到" << target_1 << "!" << std::endl;
    }
    
    if(target_2_detect_count >= DETECT_THRESHOLD && !target_2_found) {
        target_2_found = true;
        found = true;
        std::cout << "累计检测到 " << target_2 << "  " << DETECT_THRESHOLD << " 次，确认找到" << target_2 << "!" << std::endl;
    }

	if(target_3_detect_count >= DETECT_THRESHOLD && !target_3_found) {
		target_3_found = true;
		found = true;
		std::cout << "累计检测到" << target_3 << "  " << DETECT_THRESHOLD << " 次，确认找到" << target_3 << "!" << std::endl;
	}
    
    // 单独判断tank检测
    if(tank_detect_count >= DETECT_THRESHOLD) {
        found_tank = true;
        std::cout << "累计检测到tank " << DETECT_THRESHOLD << " 次，确认找到tank!" << std::endl;
    }
    
    // 判断是否达到错误检测阈值
    if(false_detect_count >= FALSE_THRESHOLD) {
        found_false = true;
        std::cout << "连续检测到错误目标 " << FALSE_THRESHOLD << " 次，found_false设为true!" << std::endl;
    }
}

/************************************************************************
函数功能: kalman滤波器
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
bool kalman_start_flag = false;  // 卡尔曼滤波器启动标志，用于初始化滤波器
bool kalman_miss_flag = false;   // 卡尔曼滤波器跟踪丢失标志，当目标丢失过多帧时设为true
double t = 0;                    // 时间变量，用于卡尔曼滤波器时间计算

int step = 0;                    // 卡尔曼滤波器处理步骤：0=未开始，1=学习阶段，2=预测阶段
ros::Time kalman_time;           // 卡尔曼滤波器时间戳，用于计算时间间隔
double kalman_predict_x = 0.;    // 卡尔曼滤波器预测的目标X坐标（米）
double kalman_predict_y = 0.;    // 卡尔曼滤波器预测的目标Y坐标（米）
double tank_time = 2.0;      // 坦克运动预测时间（秒）
double tank_shred = 0.2;     // 坦克命中阈值距离（米）
double sigma_a = 0.1;        // 加速度噪声标准差（m/s²）

class KalmanFilter {
private:
    // 状态向量 [x, y, vx, vy]
    Eigen::Vector4d x;
    // 状态协方差矩阵
    Eigen::Matrix4d P;
    // 状态转移矩阵（基础模板）
    Eigen::Matrix4d F_template;
    // 测量矩阵
    Eigen::Matrix<double, 2, 4> H;
    // 过程噪声协方差矩阵（基础模板）
    Eigen::Matrix4d Q_template;
    // 测量噪声协方差矩阵
    Eigen::Matrix2d R;
    // 单位矩阵
    Eigen::Matrix4d I;
    // 上一次更新时间
    ros::Time lastUpdateTime;
    
    // 是否初始化
    bool initialized;
    
    // 检测丢失计数器
    int missedDetectionCount;
    
    // 最大允许连续丢失次数
    int maxMissedDetections;

public:
    // 构造函数
    KalmanFilter(double initialDt = 0.1, int maxMissed = 10) : 
        initialized(false), missedDetectionCount(0), maxMissedDetections(maxMissed) {
        
        // 初始化状态向量
        x.setZero();
        
        // 初始化状态协方差矩阵
        P = Eigen::Matrix4d::Identity() * 1000.0;
        
        // 初始化状态转移矩阵模板 (dt=1作为基准)
        F_template = Eigen::Matrix4d::Identity();
        F_template(0, 2) = 1.0;
        F_template(1, 3) = 1.0;
        
        // 初始化测量矩阵 (只测量位置)
        H.setZero();
        H(0, 0) = 1.0;
        H(1, 1) = 1.0;
        
        // 初始化过程噪声协方差矩阵模板 (dt=1作为基准)
        
        Q_template.setZero();
        Q_template(0, 0) = 1.0/4 * sigma_a*sigma_a;
        Q_template(0, 2) = 1.0/2 * sigma_a*sigma_a;
        Q_template(1, 1) = 1.0/4 * sigma_a*sigma_a;
        Q_template(1, 3) = 1.0/2 * sigma_a*sigma_a;
        Q_template(2, 0) = 1.0/2 * sigma_a*sigma_a;
        Q_template(2, 2) = 1.0 * sigma_a*sigma_a;
        Q_template(3, 1) = 1.0/2 * sigma_a*sigma_a;
        Q_template(3, 3) = 1.0 * sigma_a*sigma_a;
        
        // 初始化测量噪声协方差矩阵
        R = Eigen::Matrix2d::Identity() * 0.1;
        
        // 初始化单位矩阵
        I = Eigen::Matrix4d::Identity();
        
        // 记录初始时间
        lastUpdateTime = ros::Time::now();
    }
    
    // 计算时间间隔并更新状态转移矩阵和过程噪声矩阵
    double updateTimeDependentMatrices() {
        ros::Time currentTime =ros::Time::now();
        ros::Duration duration = currentTime - lastUpdateTime;
        double dt = duration.toSec();
        lastUpdateTime = currentTime;
        
        
        return dt;
    }

    
    // 预测步骤
    void predict() {
        double dt = updateTimeDependentMatrices();
        
        // 构建状态转移矩阵
        Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
        F(0, 2) = dt;
        F(1, 3) = dt;
        
        // 构建过程噪声矩阵
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        
        Eigen::Matrix4d Q;
        Q.setZero();
        Q(0, 0) = dt4/4 * Q_template(0, 0);
        Q(0, 2) = dt3/2 * Q_template(0, 2);
        Q(1, 1) = dt4/4 * Q_template(1, 1);
        Q(1, 3) = dt3/2 * Q_template(1, 3);
        Q(2, 0) = dt3/2 * Q_template(2, 0);
        Q(2, 2) = dt2 * Q_template(2, 2);
        Q(3, 1) = dt3/2 * Q_template(3, 1);
        Q(3, 3) = dt2 * Q_template(3, 3);
        
        // 预测状态: x̂ = F·x
        x = F * x;
        
        // 预测协方差: P = F·P·F^T + Q
        P = F * P * F.transpose() + Q;
        
        // 增加检测丢失计数
        missedDetectionCount++;
    }
    
	// 预测步骤
    Eigen::Vector4d predict_next(double dt) {
        
        // 构建状态转移矩阵
        Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
        F(0, 2) = dt;
        F(1, 3) = dt;
        
        // 构建过程噪声矩阵
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        
        Eigen::Matrix4d Q;
        Q.setZero();
        Q(0, 0) = dt4/4 * Q_template(0, 0);
        Q(0, 2) = dt3/2 * Q_template(0, 2);
        Q(1, 1) = dt4/4 * Q_template(1, 1);
        Q(1, 3) = dt3/2 * Q_template(1, 3);
        Q(2, 0) = dt3/2 * Q_template(2, 0);
        Q(2, 2) = dt2 * Q_template(2, 2);
        Q(3, 1) = dt3/2 * Q_template(3, 1);
        Q(3, 3) = dt2 * Q_template(3, 3);
        
        
		return F * x;
        
        
    }

	Eigen::Vector4d get_x()
	{
		return x;
	}

    // 更新步骤
    bool update(const Eigen::Vector2d& z) {
        // 预测步骤（包含时间更新）
        predict();
        
        // 重置检测丢失计数
        missedDetectionCount = 0;
        
        // 计算残差: y = z - H·x
        Eigen::Vector2d y = z - H * x;
        
        // 计算残差协方差: S = H·P·H^T + R
        Eigen::Matrix2d S = H * P * H.transpose() + R;
        
        // 计算卡尔曼增益: K = P·H^T·S^(-1)
        Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();
        
        // 更新状态估计: x̂ = x̂ + K·y
        x += K * y;
        
        // 更新协方差矩阵: P = (I - K·H)·P
        P = (I - K * H) * P;
        
        // 如果未初始化，标记为已初始化
        if (!initialized) {
            initialized = true;
        }
        
        return true;
    }
    
    // 无观测值时的预测
    bool predictOnly() {
        // 只执行预测步骤
        predict();
        
        // 检查是否超出最大丢失次数
        return (missedDetectionCount <= maxMissedDetections);
    }
    
    // 设置初始状态
    void setInitialState(const Eigen::Vector4d& initialState) {
        x = initialState;
        initialized = true;
    }
    
    // 获取当前状态估计
    Eigen::Vector4d getState() const {
        return x;
    }
    
    // 检查滤波器是否已初始化
    bool isInitialized() const {
        return initialized;
    }
    
    // 检查是否因多次丢失检测而失效
    bool isTrackValid() const {
        return (missedDetectionCount <= maxMissedDetections);
    }
};

KalmanFilter *kf;

bool Kalman_prediction()
{
  if(kalman_start_flag)
  {
  	t = 0;
	step = 1;
  	kf = new KalmanFilter();
	kalman_start_flag = false;
	kalman_time = ros::Time::now();
  }
  Eigen::Vector2d detection;
  bool newDetectionAvailable;
  if(found_tank)
  {
  	newDetectionAvailable = true;
	detection(0) = box_target_x;
	detection(1) = box_target_y;
  }
  else
  {
  	newDetectionAvailable = false;
  }

  if(newDetectionAvailable) 
  {
	kf->update(detection);
  }
  else
  {
	bool trackingValid = kf->predictOnly();
    if (!trackingValid) {
        kalman_miss_flag = true;
        ROS_WARN("Tracking lost!"); 
            // 可以在这里实现重新初始化逻辑
        }
  }

  if(step == 1)
  {
	Eigen::Vector4d x = kf->get_x();
	if(calculate_abs_distance(x(0),x(1),local_pos.pose.pose.position.x,local_pos.pose.pose.position.y) < calculate_abs_distance(kalman_predict_x,kalman_predict_y,local_pos.pose.pose.position.x,local_pos.pose.pose.position.y))
  	{
		kalman_predict_x = x(0);
		kalman_predict_y = x(1);
	}
    if(ros::Time::now() - kalman_time >= ros::Duration(15))
	{
	    step = 2;
		now_target_x = kalman_predict_x;
		now_target_y = kalman_predict_y;
		kalman_time = ros::Time::now();
	}
	return false;
  }
  else if(step == 2)
  {
	 if(ros::Time::now() - kalman_time <= ros::Duration(3.0)) return false;
     Eigen::Vector4d tank = kf->predict_next(tank_time);
	 kalman_predict_x = tank(0);
	 kalman_predict_y = tank(1);
	 if(calculate_abs_distance(kalman_predict_x,kalman_predict_y,local_pos.pose.pose.position.x,local_pos.pose.pose.position.y) < tank_shred)
	 {
		return true;
	 }
	 return false;
  }
}



/************************************************************************
函数功能5: 获取yolo识别目标的位置信息
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
geometry_msgs::PointStamped object_pos;
double position_detec_x = 0;
double position_detec_y = 0;
double position_detec_z = 0;
void object_pos_cb(const geometry_msgs::PointStamped::ConstPtr &msg);
void object_pos_cb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	object_pos = *msg;
}

/************************************************************************
函数功能6: 目标识别，采用速度控制运动，任务是识别到目标后，保持无人机正对着目标
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float object_recognize_track_vel_last_time_position_x = 0;
float object_recognize_track_vel_last_time_position_y = 0;
bool object_recognize_track_vel_flag = false;
bool object_recognize_track_vel(string str, float yaw, float altitude, float speed, float error_max);
bool object_recognize_track_vel(string str, float yaw, float altitude, float speed, float error_max)
{
	// 此处false主要是为了获取任务初始时候的位置信息
	if (object_recognize_track_vel_flag == false)
	{
		// 获取初始位置，防止无人机飘
		object_recognize_track_vel_last_time_position_x = local_pos.pose.pose.position.x;
		object_recognize_track_vel_last_time_position_y = local_pos.pose.pose.position.y;
		object_recognize_track_vel_flag = true;
		ROS_INFO("开始识别目标并且保持跟踪");
	}
	// 此处首先判断是否识别到指定的目标
	if (object_pos.header.frame_id == str)
	{
		// 此处实时更新当前位置，防止目标丢失的时候导致无人机漂移
		object_recognize_track_vel_last_time_position_x = local_pos.pose.pose.position.x;
		object_recognize_track_vel_last_time_position_y = local_pos.pose.pose.position.y;
		// 获取到目标物体相对摄像头的坐标
		position_detec_x = object_pos.point.x;
		position_detec_y = object_pos.point.y;
		position_detec_z = object_pos.point.z;
		if (fabs(position_detec_x - 320) < error_max && fabs(position_detec_y - 240) < error_max)
		{
			ROS_INFO("到达目标的正上/前方，在允许误差范围内保持");
			return true;
		}
		else
		{
			// 摄像头朝下安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向，Z对应上下
			if (position_detec_x - 320 >= error_max)
			{
				setpoint_raw.velocity.y = -speed;
			}
			else if (position_detec_x - 320 <= -error_max)
			{
				setpoint_raw.velocity.y = speed;
			}
			else
			{
				setpoint_raw.velocity.y = 0;
			}
			// 无人机前后移动速度控制
			if (position_detec_y - 240 >= error_max)
			{
				setpoint_raw.velocity.x = -speed;
			}
			else if (position_detec_y - 240 <= -error_max)
			{
				setpoint_raw.velocity.x = speed;
			}
			else
			{
				setpoint_raw.velocity.x = 0;
			}
			// 机体坐标系下发送xy速度期望值以及高度z期望值至飞控（输入：期望xy,期望高度）
			setpoint_raw.type_mask = 1 + 2 + /* 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 + 1024 + 2048;
			setpoint_raw.coordinate_frame = 8;
			setpoint_raw.position.z = init_position_z_take_off + altitude;
		}
	}
	else
	{
		setpoint_raw.position.x = object_recognize_track_vel_last_time_position_x;
		setpoint_raw.position.y = object_recognize_track_vel_last_time_position_y;
		setpoint_raw.type_mask = /*1 + 2 + 4*/ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024*/ + 2048;
		setpoint_raw.coordinate_frame = 1;
		setpoint_raw.position.z = init_position_z_take_off + altitude;
		setpoint_raw.yaw = yaw;
	}
	return false;
}

/************************************************************************
函数功能7: //无人机追踪，仅水平即Y方向，通常用于无人机穿越圆框、方框等使用
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float object_recognize_track_last_time_position_x = 0;
float object_recognize_track_last_time_position_y = 0;
bool object_recognize_track_flag = false;
bool object_recognize_track(string str, float yaw, int reverse, float altitude, float error_max, float ctrl_coef);
bool object_recognize_track(string str, float yaw, int reverse, float altitude, float error_max, float ctrl_coef)
{
	if (object_recognize_track_flag == false)
	{
		object_recognize_track_last_time_position_x = local_pos.pose.pose.position.x;
		object_recognize_track_last_time_position_y = local_pos.pose.pose.position.y;
		object_recognize_track_flag = true;
		ROS_INFO("开始识别目标并且保持跟踪");
	}
	// 此处首先判断是否识别到指定的目标
	if (object_pos.header.frame_id == str)
	{
		// 此处实时更新当前位置，防止目标丢失的时候导致无人机漂移
		object_recognize_track_last_time_position_x = local_pos.pose.pose.position.x;
		object_recognize_track_last_time_position_y = local_pos.pose.pose.position.y;
		// 获取到目标物体相对摄像头的坐标
		position_detec_x = object_pos.point.x;
		position_detec_y = object_pos.point.y;
		position_detec_z = object_pos.point.z;

		// 摄像头向下或者向前安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向，Z对应上下
		if (position_detec_x - 320 >= error_max)
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y - ctrl_coef * reverse;
		}
		else if (position_detec_x - 320 <= -error_max)
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y + ctrl_coef * reverse;
		}
		else
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y;
		}

		if (fabs(position_detec_x - 320) < error_max)
		{
			ROS_INFO("到达目标的正上/前方，在允许误差范围内保持");
			return true;
		}
	}
	else
	{
		setpoint_raw.position.x = object_recognize_track_last_time_position_x;
		setpoint_raw.position.y = object_recognize_track_last_time_position_y;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = object_recognize_track_last_time_position_x;
	setpoint_raw.position.z = 0 + altitude;
	setpoint_raw.yaw = yaw;
	return false;
}

/************************************************************************
函数功能8: //无人机追踪，仅水平即Y方向，通常用于无人机穿越圆框、方框等使用
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float object_recognize_track_omni_last_time_position_x = 0;
float object_recognize_track_omni_last_time_position_y = 0;
bool object_recognize_track_omni_flag = false;
bool object_recognize_track_omni(string str, float yaw, int reverse, float altitude, float error_max, float ctrl_coef);
bool object_recognize_track_omni(string str, float yaw, int reverse, float altitude, float error_max, float ctrl_coef)
{
	if (object_recognize_track_omni_flag == false)
	{
		object_recognize_track_omni_last_time_position_x = local_pos.pose.pose.position.x;
		object_recognize_track_omni_last_time_position_y = local_pos.pose.pose.position.y;
		object_recognize_track_omni_flag = true;
		ROS_INFO("开始识别目标并且保持跟踪");
	}
	// 此处首先判断是否识别到指定的目标
	if (object_pos.header.frame_id == str)
	{
		object_recognize_track_omni_last_time_position_x = local_pos.pose.pose.position.x;
		object_recognize_track_omni_last_time_position_y = local_pos.pose.pose.position.y;
		// 获取到目标物体相对摄像头的坐标
		position_detec_x = object_pos.point.x;
		position_detec_y = object_pos.point.y;
		position_detec_z = object_pos.point.z;

		// 摄像头向下或者向前安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向，Z对应上下
		if (position_detec_x - 320 >= error_max)
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y - ctrl_coef * reverse;
		}
		else if (position_detec_x - 320 <= -error_max)
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y + ctrl_coef * reverse;
		}
		else
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y;
		}
		if (position_detec_y - 240 >= error_max)
		{
			setpoint_raw.position.x = local_pos.pose.pose.position.x - ctrl_coef * reverse;
		}
		else if (position_detec_y - 240 <= -error_max)
		{
			setpoint_raw.position.x = local_pos.pose.pose.position.x + ctrl_coef * reverse;
		}
		else
		{
			setpoint_raw.position.x = local_pos.pose.pose.position.x;
		}

		if (fabs(position_detec_x - 320) < error_max && fabs(position_detec_y - 240) < error_max)
		{
			ROS_INFO("到达目标的正上/前方，在允许误差范围内保持");
			return true;
		}
	}
	else
	{
		setpoint_raw.position.x = object_recognize_track_omni_last_time_position_x;
		setpoint_raw.position.y = object_recognize_track_omni_last_time_position_y;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.z = 0 + altitude;
	setpoint_raw.yaw = yaw;
	return false;
}



/************************************************************************
函数功能11: move_base速度转换
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
bool mission_obs_avoid_flag = false;
bool mission_obs_avoid(float x, float y, float z, float yaw)
{

	MoveBaseClient ac("move_base", true);
	while (!ac.waitForServer(ros::Duration(1.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	if (mission_obs_avoid_flag == false)
	{
		mission_obs_avoid_flag = true;
		ROS_INFO("发布避障导航目标点 x = %f, y = %f, z = %f, yaw = %f", x, y, z, yaw);
		move_base_msgs::MoveBaseGoal first_goal;
		first_goal.target_pose.header.frame_id = "map";
		first_goal.target_pose.header.stamp = ros::Time::now();
		first_goal.target_pose.pose.position.x = x;
		first_goal.target_pose.pose.position.y = y;
		first_goal.target_pose.pose.position.z = z;
		first_goal.target_pose.pose.orientation.w = 1;
		ac.sendGoal(first_goal);
	}

	if (fabs(local_pos.pose.pose.position.x - x) < 0.2 && fabs(local_pos.pose.pose.position.y - y) < 0.2)
	{
		ROS_INFO("到达目标点，自主导航任务完成");
		mission_obs_avoid_flag = false;
		return true;
	}
	return false;
}

/************************************************************************
函数功能12: move_base
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
void cmd_to_px4(const geometry_msgs::Twist &msg);
void cmd_to_px4(const geometry_msgs::Twist &msg)
{
	if (mission_obs_avoid_flag)
	{
		setpoint_raw.type_mask = 1 + 2 + /* 4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 /*+ 2048*/;
		setpoint_raw.coordinate_frame = 8;
		setpoint_raw.velocity.x = msg.linear.x;
		setpoint_raw.velocity.y = msg.linear.y;
		setpoint_raw.position.z = ALTITUDE;
		//		setpoint_raw.yaw_rate = 0;
		setpoint_raw.yaw_rate = msg.angular.z;
		ROS_INFO("move_base working, vel_x:%.2f, vel_y:%.2f, yaw_rate:%.2f", msg.linear.x, msg.linear.y, msg.angular.z);
	}
}

/************************************************************************
函数功能13: 穿越圆框，发布圆框后方目标点，效果良好，有时间以后可以继续优化
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float target_through_init_position_x = 0;
float target_through_init_position_y = 0;
bool target_through_init_position_flag = false;
bool target_through(float pos_x, float pos_y, float z, float yaw);
bool target_through(float pos_x, float pos_y, float z, float yaw)
{
	// 初始化位置一次即可，用于获取无人机初始位置
	if (!target_through_init_position_flag)
	{
		target_through_init_position_x = local_pos.pose.pose.position.x;
		target_through_init_position_y = local_pos.pose.pose.position.y;
		target_through_init_position_flag = true;
	}
	setpoint_raw.position.x = target_through_init_position_x + pos_x;
	setpoint_raw.position.y = target_through_init_position_y + pos_y;
	setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.z = z;
	setpoint_raw.yaw = yaw;

	double current_yaw, a, b;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(a, b, current_yaw);
	if (fabs(local_pos.pose.pose.position.x - pos_x - target_through_init_position_x) < 0.1 && fabs(local_pos.pose.pose.position.y - pos_y - target_through_init_position_y) < 0.1 && fabs(current_yaw - yaw) < 0.1)
	{
		target_through_init_position_flag = false;
		ROS_INFO("到达目标点，穿越圆框任务完成");
		return true;
	}
	return false;
}

/************************************************************************
函数功能:depth_image
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float horizon_depth[640];
bool depth_flag = false;
int left_door_point = -1; // 左侧门的起始点
int right_door_point = -1; // 右侧门的起始点
float door_distance = 0; // 门的距离
int door_direction = 0; // 门的方向，1向左，2表示向右，0表示不调整
void depth_image_cb(const camera_processor::PointDepthConstPtr &msg)
{
	int depth_error = 10;
    if (!depth_flag) return;
    
    // 检查数据长度是否匹配（640*480）
    if (msg->depths.size() != 640 * 480) {
        ROS_WARN("Depth data size mismatch! Expected %d, got %zu", 
                 640*480, msg->depths.size());
        return;
    }
	float sum = 0;
	float num = 0;
    
    // 将二维数组转换为一维数组（y行x列）
    for (int x = 0; x < 640; ++x) {
		sum = 0;
		num = 0;
        for (int y = 220; y < 260; ++y){
            // 原始数据存储格式：y*640 + x（与发布时一致）
            size_t index = y * 640 + x;
			if(msg->depths[index]==0) continue;
			sum += msg->depths[index];
			num ++;
        }
		if(num == 0) horizon_depth[x] = depth_error++;
		else{
			horizon_depth[x] = sum / num;
			if(horizon_depth[x] > 2) horizon_depth[x] = depth_error++; // 限制最大值为2
		}
    }
}

//1:left 2:right
int judge_door()
{
	float last_average = 0;
	float now_average = 0;
	float door_sum = 0;
	bool door_flag = true;
	for(int i = door_step ; i < 640-door_step ; i += door_step + 1)
	{
		door_sum = 0;
		for(int j = i - door_step; j <= i + door_step; j++)
		{
			if(horizon_depth[j] == 10) 
			{
				door_flag = false;
				break;
			}
			door_sum += horizon_depth[j];
		}
		if(!door_flag)
		{
			door_flag = true;
			continue;
		}
		now_average = door_sum/(2*door_step + 1);
		if(now_average <= 0) continue;
		cout << i <<": now_average = " << now_average << "last_average = " << last_average << endl;
		if(last_average!=0 && abs(last_average - now_average) >= door_shred)
		{
			if(last_average - now_average >= 0) return 1;
			else if(last_average - now_average < 0) return 2;
		}
		last_average = now_average;
	}
	return 0;
}
struct {
	int left;
	int right;
	int width;
	float distance;
} line[640];

float line_shred = 0.1;//线段分割的阈值
int line_key = 0; //找到的线段数量

void find_line(){
	for(int i = 0; i < 640; i++){
		line[i].left = -1;
		line[i].right = -1;
		line[i].width = 0;
	}
	line[0].left = 0;
	line_key = 0;
	for(int i = 0; i < 640; i++){
		if( abs(horizon_depth[i] - horizon_depth[line[line_key].left]) < line_shred){
			line[line_key].right = i;
			line[line_key].width++;
		}else{
			line[line_key].right = i - 1;
			line_key++;
			line[line_key].left = i;
		}
	}
}
void print_line(){
	int valid_line_count = 0;
	for(int i = 0; i <= line_key; i++){
		if(line[i].width > 60){
			valid_line_count++;
		}
	}
	// ROS_INFO("找到的线段数量: %d, 长度大于20的线段数量: %d", line_key, valid_line_count);
	for(int i = 0; i <= line_key; i++){
		if(line[i].width > 60){
			line[i].distance = (horizon_depth[line[i].left] + horizon_depth[line[i].right]) / 2;
			// ROS_INFO("Line %d: left = %d, right = %d, width = %d, distance = %.2f", i, line[i].left, line[i].right, line[i].width, line[i].distance);
		}
	}
}
void find_door() {
    ROS_INFO("开始门检测，将连续检测5次");

    int direction_votes[3] = {0}; // 统计 door_direction: 0=中, 1=左, 2=右
    int valid_detection_count = 0; // 成功检测次数

    // 用于调试：记录每次检测结果
    int detection_results[5];
    float distances[5];

    // 连续检测5次
    for (int detection_count = 0; detection_count < 5; detection_count++) {
        ROS_INFO("第 %d 次检测", detection_count + 1);

        find_line();
        print_line();

        int first_line = -1, second_line = -1;

        // 找最长和第二长的线段
        for (int i = 0; i <= line_key; i++) {
            if (line[i].width > 60) {
                if (first_line == -1 || line[i].width > line[first_line].width) {
                    second_line = first_line;
                    first_line = i;
                } else if (second_line == -1 || line[i].width > line[second_line].width) {
                    second_line = i;
                }
            }
        }

        // 调整顺序：first_line 在左，second_line 在右
        if (first_line != -1 && second_line != -1) {
            if (line[first_line].left > line[second_line].left) {
                int temp = first_line;
                first_line = second_line;
                second_line = temp;
            }
        }

        // 初始化门参数
        left_door_point = -1;
        right_door_point = -1;
        door_distance = -1;
        int current_direction = -1; // 本次检测的方向

        // 分析情况
        if (first_line != -1 && second_line != -1) {
            ROS_INFO("两侧均有挡板");
            if (fabs(line[first_line].distance - line[second_line].distance) < 0.3) {
                ROS_INFO("平行挡板");
                left_door_point = line[first_line].right;
                right_door_point = line[second_line].left;
                door_distance = (line[first_line].distance + line[second_line].distance) / 2.0;
            } else {
                ROS_INFO("非平行挡板");
                int closer_line = (line[first_line].distance < line[second_line].distance) ? first_line : second_line;
                if ((line[closer_line].left + line[closer_line].right) / 2 > 320) {
                    ROS_INFO("单侧挡板在右侧");
                    right_door_point = line[closer_line].left;
                    door_distance = line[closer_line].distance;
                    left_door_point = 0;
                    for (int i = 0; i < line[closer_line].left; i++) {
                        if (fabs(horizon_depth[i] - line[closer_line].distance) < 0.1) {
                            left_door_point = i;
                            break;
                        }
                    }
                } else {
                    ROS_INFO("单侧挡板在左侧");
                    left_door_point = line[closer_line].right;
                    door_distance = line[closer_line].distance;
                    right_door_point = 639;
                    for (int i = 639; i > line[closer_line].right; i--) {
                        if (fabs(horizon_depth[i] - line[closer_line].distance) < 0.1) {
                            right_door_point = i;
                            break;
                        }
                    }
                }
            }
        }
        else if (first_line != -1) {
            ROS_INFO("仅有一个挡板");
            if ((line[first_line].left + line[first_line].right) / 2 > 320) {
                ROS_INFO("挡板在右侧");
                right_door_point = line[first_line].left;
                door_distance = line[first_line].distance;
                left_door_point = 0;
                for (int i = 0; i < line[first_line].left; i++) {
                    if (fabs(horizon_depth[i] - line[first_line].distance) < 0.1) {
                        left_door_point = i;
                        break;
                    }
                }
            } else {
                ROS_INFO("挡板在左侧");
                left_door_point = line[first_line].right;
                door_distance = line[first_line].distance;
                right_door_point = 639;
                for (int i = 639; i > line[first_line].right; i--) {
                    if (fabs(horizon_depth[i] - line[first_line].distance) < 0.1) {
                        right_door_point = i;
                        break;
                    }
                }
            }
        }

        // 如果成功提取出门的边界
        if (left_door_point != -1 && right_door_point != -1 && door_distance > 0) {
            // ROS_INFO("检测到门: left=%d, right=%d, center=%.1f, distance=%.2f",
            //          left_door_point, right_door_point,
            //          (left_door_point + right_door_point) / 2.0, door_distance);

            // 计算本次方向
            double center = (left_door_point + right_door_point) / 2.0;
            if (center < (320 - door_adjust_range)) {
                current_direction = 1; // 左
            } else if (center > (320 + door_adjust_range)) {
                current_direction = 2; // 右
            } else {
                current_direction = 0; // 中
            }

            // 投票
            direction_votes[current_direction]++;
            valid_detection_count++;

            // 保存距离用于后续平均（可选）
            distances[detection_count] = door_distance;
            detection_results[detection_count] = current_direction;


            geometry_msgs::Point door_points_msg;
            door_points_msg.x = left_door_point;
            door_points_msg.y = right_door_point;
            door_points_msg.z = center;
            door_points_pub.publish(door_points_msg);
        } else {
            // ROS_WARN("第 %d 次检测失败：未找到有效门边界", detection_count + 1);
            detection_results[detection_count] = -1; // 标记为无效
        }

        // ros::Duration(0.1).sleep(); // 短暂延时避免处理过快
    }

    // === 所有5次检测完成，开始投票决策 ===
    ROS_INFO("5次检测完成，开始汇总结果...");

    int final_direction = -1;
    int max_votes = 0;
    for (int i = 0; i < 3; i++) {
        ROS_INFO("方向 %d 投票数: %d", i, direction_votes[i]);
        if (direction_votes[i] > max_votes) {
            max_votes = direction_votes[i];
            final_direction = i;
        }
    }

    // 如果没有足够有效检测，尝试使用最后一次有效方向
    if (valid_detection_count == 0) {
		door_direction = 0; // 默认中间
        ROS_ERROR("5次检测均未成功，无法确定门方向！");
        return;
    }
    // 设置最终 door_direction
    door_direction = final_direction;

    // 计算平均距离：仅使用方向与最终决策一致的检测
    double avg_distance = 0.0;
    int consistent_count = 0;

    for (int i = 0; i < 5; i++) {
        // 只有方向匹配最终决策，且该次检测有效
        if (detection_results[i] != -1 && detection_results[i] == final_direction) {
            avg_distance += distances[i];
            consistent_count++;
        }
    }

    if (consistent_count > 0) {
        avg_distance /= consistent_count;
        door_distance = avg_distance; // 更新门的距离
        ROS_INFO("使用 %d 次方向一致的检测，平均距离: %.2f 米", consistent_count, door_distance);
    } else {
        // 极端情况：没有一次检测的方向与最终投票结果一致（理论上不太可能）
        door_distance = 1.0; // 设为默认值
    }

    // 输出最终决策
    switch (door_direction) {
        case 0:
            ROS_INFO("最终决策：门在中间，距离: %.2f 米", avg_distance);
            break;
        case 1:
            ROS_INFO("最终决策：门在左侧，距离: %.2f 米", avg_distance);
            break;
        case 2:
            ROS_INFO("最终决策：门在右侧，距离: %.2f 米", avg_distance);
            break;
    }
}

/************************************************************************
函数功能14:降落
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool precision_land_init_position_flag = false;
void precision_land();
void precision_land()
{
	if (!precision_land_init_position_flag)
	{
		precision_land_init_position_x = local_pos.pose.pose.position.x;
		precision_land_init_position_y = local_pos.pose.pose.position.y;
		precision_land_init_position_flag = true;
	}
	setpoint_raw.position.x = precision_land_init_position_x;
	setpoint_raw.position.y = precision_land_init_position_y;
	setpoint_raw.position.z = -0.15;
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
}

/************************************************************************
函数功能:ego_planner导航
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
quadrotor_msgs::PositionCommand ego_sub;
void ego_sub_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
void ego_sub_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
	ego_sub = *msg;
}

/************************************************************************
函数功能: ego_planner是否规划出航线
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
bool rec_traj_flag = false;
void rec_traj_cb(const std_msgs::Bool::ConstPtr &msg);
void rec_traj_cb(const std_msgs::Bool::ConstPtr &msg)
{
	rec_traj_flag = msg->data;
}

void PI_attitude_control()
{                                                      
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.type_mask = 0b101111100011; // vx vy z yaw
	setpoint_raw.velocity.x = 0.85 * ego_sub.velocity.x + (ego_sub.position.x - local_pos.pose.pose.position.x) * 1;
	setpoint_raw.velocity.y = 0.85 * ego_sub.velocity.y + (ego_sub.position.y - local_pos.pose.pose.position.y) * 1;
	if(now_mode == 1)//穿门模式
	{
		setpoint_raw.position.z = DOOR_ALTITUDE;
	}
	else setpoint_raw.position.z = ego_sub.position.z;
	setpoint_raw.yaw = ego_sub.yaw;

	ROS_INFO("ego: vel_x = %.2f, vel_y = %.2f, z = %.2f, yaw = %.2f", ego_sub.velocity.x, ego_sub.velocity.y, ego_sub.position.z, ego_sub.yaw);
	ROS_INFO("ego: x = %.2f, y = %.2f, z = %.2f, yaw = %.2f", ego_sub.position.x, ego_sub.position.y, ego_sub.position.z, ego_sub.yaw);
	ROS_INFO("已触发控制器: vel_x = %.2f, vel_y = %.2f, z = %.2f, yaw = %.2f", setpoint_raw.velocity.x, setpoint_raw.velocity.y, setpoint_raw.position.z, setpoint_raw.yaw);
	ROS_INFO("now_yaw: %.2f", square_yaw_cb * 180.0 / M_PI);
	ROS_INFO("ego_target_x = %.2f, ego_target_y = %.2f", ego_now_x, ego_now_y);
	ROS_INFO("now_mode = %d", now_mode);
}

/************************************************************************
函数功能: 发布ego planner模式
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
void publish_ego_planner_mode(int mode);
void publish_ego_planner_mode(int mode)
{
	ego_planner_mode.data = mode;
	ego_planner_mode_pub.publish(ego_planner_mode);
	ROS_INFO("发布ego planner模式: %d", mode);
}


/************************************************************************
函数功能:ego_planner发布目标点函数
//mode0为正常巡航模式
//mode1为激进穿门模式
*************************************************************************/
float before_ego_pose_x = 0;
float before_ego_pose_y = 0;
float before_ego_pose_z = 0;
bool pub_ego_goal_flag = false;
bool pub_ego_goal(float x, float y, float z, float err_max, int first_target = 0,int mode = 0);
bool pub_ego_goal(float x, float y, float z, float err_max, int first_target,int mode)
{
	ego_planner_mode.data = mode;
	ego_planner_mode_pub.publish(ego_planner_mode);
	ROS_INFO("发布ego planner模式: %d", mode);
	now_mode = mode;
	before_ego_pose_x = local_pos.pose.pose.position.x;
	before_ego_pose_y = local_pos.pose.pose.position.y;
	before_ego_pose_z = local_pos.pose.pose.position.z;

	double current_yaw, a, b;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(a, b, current_yaw);

	geometry_msgs::PoseStamped target_point;
	if (pub_ego_goal_flag == false)
	{
		pub_ego_goal_flag = true;
		target_point.pose.position.x = x;
		target_point.pose.position.y = y;
		target_point.pose.position.z = z;
		ego_now_x = x;
		ego_now_y = y;

		planner_goal_pub.publish(target_point);
		ROS_INFO("发送目标点( %f, %f, %f )", x, y, z);

		finish_ego_flag.data = false;
		finish_ego_pub.publish(finish_ego_flag);
	}

	if (rec_traj_flag == true)
	{
		PI_attitude_control();
		if(now_mode == 0)
		{
			if (ego_sub.position.z > map_size_z)
			{
				setpoint_raw.position.z = map_size_z;
				std::cout << "exceed map_size_z" << std::endl;
			}
			else if (ego_sub.position.z < ground_height)
			{
				setpoint_raw.position.z = ground_height;
				std::cout << "lower than ground height" << std::endl;
			}
		}
        if (first_target == 1)
        {
            std::cout << "get traj" << std::endl;
            std::cout << "ego_x : " << ego_sub.position.x << std::endl;
            std::cout << "ego_y : " << ego_sub.position.y << std::endl;
            std::cout << "ego_z : " << ego_sub.position.z << std::endl;
            std::cout << "ego_yaw : " << ego_sub.yaw << std::endl;
            setpoint_raw.type_mask = 0b101111000000; // 101 111 000 000  vx vy　vz x y z+ yaw
            setpoint_raw.coordinate_frame = 1;
            setpoint_raw.position.x = ego_sub.position.x;
            setpoint_raw.position.y = ego_sub.position.y;
            setpoint_raw.position.z = ego_sub.position.z;
            setpoint_raw.velocity.x = ego_sub.velocity.x;
            setpoint_raw.velocity.y = ego_sub.velocity.y;
            setpoint_raw.velocity.z = ego_sub.velocity.z;
            setpoint_raw.yaw = 0;
        }
        if (first_target == 2)
        {
            std::cout << "get traj" << std::endl;
            std::cout << "ego_x : " << ego_sub.position.x << std::endl;
            std::cout << "ego_y : " << ego_sub.position.y << std::endl;
            std::cout << "ego_z : " << ego_sub.position.z << std::endl;
            std::cout << "ego_yaw : " << ego_sub.yaw << std::endl;
            setpoint_raw.type_mask = 0b101111000000; // 101 111 000 000  vx vy　vz x y z+ yaw
            setpoint_raw.coordinate_frame = 1;
            setpoint_raw.position.x = ego_sub.position.x;
            setpoint_raw.position.y = ego_sub.position.y;
            setpoint_raw.position.z = ego_sub.position.z;
            setpoint_raw.velocity.x = ego_sub.velocity.x;
            setpoint_raw.velocity.y = ego_sub.velocity.y;
            setpoint_raw.velocity.z = ego_sub.velocity.z;
            setpoint_raw.yaw = 3.14;
        }
	}
	else
	{
		std::cout << "not rec_tarj" << std::endl;
		setpoint_raw.position.x = before_ego_pose_x;
		setpoint_raw.position.y = before_ego_pose_y;
		setpoint_raw.position.z = before_ego_pose_z;
		setpoint_raw.yaw = current_yaw;
		setpoint_raw.type_mask = 0b101111111000; // 101 111 111 000  x y z+ yaw
		setpoint_raw.coordinate_frame = 1;
	}

	setpoint_raw.header.stamp = ros::Time::now();

	if (fabs(local_pos.pose.pose.position.x - x) < err_max && fabs(local_pos.pose.pose.position.y - y) < err_max)
	{
		ROS_INFO("到达目标点, ego_planner导航任务完成");
		pub_ego_goal_flag = false;

		finish_ego_flag.data = true;
		finish_ego_pub.publish(finish_ego_flag);

		return true;
	}
	return false;
}

float calculate_yaw(float target_x, float target_y){
	float delta_x = target_x - local_pos.pose.pose.position.x;
	float delta_y = target_y - local_pos.pose.pose.position.y;
	float cal_yaw = atan2(delta_y, delta_x);
	return cal_yaw;
}
/************************************************************************
函数功能1：gjs_lidar
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
geometry_msgs::PointStamped door_pos;
float sum_door_x = 0;
float sum_door_y = 0;
float sum_door_z = 0;

float avg_door_x = 0;
float avg_door_y = 0;
float avg_door_z = 0;

float door_x = 0;
float door_y = 0;
float door_z = 0;
int check_num = 0;
int door_check_num = 10;
uint32_t door_seq = 0;
float door_thred = 0.2;
bool start_check_door_flag = false;
bool finish_check_door_flag = false;

void door_lidar_cb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	if(!start_check_door_flag) return;
	door_pos = *msg;

	if(door_pos.header.seq == door_seq) return;

	door_seq = door_pos.header.seq;
	sum_door_x += door_pos.point.x;
	sum_door_y += door_pos.point.y;
	sum_door_z += door_pos.point.z;
	check_num++;
	avg_door_x = sum_door_x / check_num;
	avg_door_y = sum_door_y / check_num;
	avg_door_z = sum_door_z / check_num;
	if(abs(door_pos.point.x - avg_door_x) > door_thred || abs(door_pos.point.y - avg_door_y) > door_thred)
	{
		check_num = 0;
		sum_door_x = 0;
		sum_door_y = 0;
		sum_door_z = 0;
		avg_door_x = 0;
		avg_door_y = 0;
		avg_door_z = 0;
		return;
	}

	if(check_num >= door_check_num)
	{
		door_x = avg_door_x;
		door_y = avg_door_y;
		door_z = avg_door_z;
		finish_check_door_flag = true;
		start_check_door_flag = false;
	}
	ROS_INFO("door_pos_x:%f,door_pos_y:%f,door_pos_z:%f",door_pos.point.x,door_pos.point.y,door_pos.point.z);
}



