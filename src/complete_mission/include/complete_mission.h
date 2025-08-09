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
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <circle_detector/CirclePosition.h>
#include <yolov8_ros_msgs/BoundingBoxes.h>

using namespace std;


#define ALTITUDE 1.2
#define VIEW_ALTITUDE 1.0
#define QR_ALTITUDE 0.40
#define RING_HEIGHT 1.50

mavros_msgs::PositionTarget setpoint_raw;
ros::Publisher planner_goal_pub;
ros::Publisher finish_ego_pub;
std_msgs::Bool finish_ego_flag;

int spin_flag = 0 ;

float map_size_z = 1.1;
float ground_height = 0.9;

float calculate(float x1,float y1,float x2,float y2)
{
	return abs(sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)));
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

	if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max && fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < error_max)
	{
		ROS_INFO("到达目标点，巡航点任务完成");
		mission_pos_cruise_flag = false;
		return true;
	}
	return false;
}

/************************************************************************
函数功能：qr
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
std::string qr_data[3];
int qr_index = 0;
std::string word;
std::string read_data[3];
void qr_cb(const std_msgs::String::ConstPtr& msg){
	qr_index =0;
	for(auto& ch : msg->data)
	{
		if(ch == ',')
		{
			qr_data[qr_index] = word;
			qr_index++;
			word.clear(); 
		}
		else
		{
			word +=ch;
		}
	}
	qr_data[qr_index] = word;
	qr_index = 0;
	word.clear();
	
}

bool qr_success_flag = true;

bool qr_mission(float x,float y,float z,float yaw);
bool qr_mission(float x,float y,float z,float yaw){
	setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = x + init_position_x_take_off;
	setpoint_raw.position.y = y + init_position_y_take_off;
	setpoint_raw.position.z = z + init_position_z_take_off;
	setpoint_raw.yaw = yaw;
	
	qr_success_flag = true;
	for(int i=0;i<3;i++){
		if(qr_data[i].empty()){
			continue;
		}
		read_data[i] = qr_data[i];
	}
	
	for(int i=0;i<3;i++){
		if(read_data[i].empty()){
			qr_success_flag = false;
			break;
		}
	}
	if(qr_success_flag){
		return true;
	}
	else{ 
		return false;
	}
}

/************************************************************************
函数功能：circle_detection_cb 圆环检测回调函数
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
int latest_circle_x = 320;  // 默认值为图像中心 (1280/2)
int latest_circle_y = 240;  // 默认值为图像中心 (960/2)
bool circle_data_valid = false; // 数据是否实时更新

void circle_detection_cb(const circle_detector::CirclePosition::ConstPtr& msg);
void circle_detection_cb(const circle_detector::CirclePosition::ConstPtr& msg) {
    // 仅当检测到有效圆环时更新数据
    if (msg->x != 0 || msg->y != 0) {
        latest_circle_x = msg->x;
        latest_circle_y = msg->y;
        circle_data_valid = true; // 标记为实时更新数据
    } else {
        circle_data_valid = false; // 标记为无效数据（使用上一次的值）
    }
}

/************************************************************************
函数功能：ring_mission
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
int dx = 0 ;

int ring_mission(float align_threshold);
int ring_mission(float align_threshold){

	dx = latest_circle_x - 320;

	if (abs(dx) > align_threshold) {
		if( dx > 0 ) {
			return 1 ;//向右移动
		}
		else {
			return 2 ;//向左移动
		}
	}
	else {
		return 0 ;
	}
	
}

/************************************************************************
函数功能31：自主巡航，发布目标位置，控制无人机到达目标，采用local坐标系位置控制
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/

/************************************************************************
函数功能4：自主巡航，发布目标位置，控制无人机到达目标，采用机体坐标系位置控制
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float current_position_cruise_last_position_x = 0;
float current_position_cruise_last_position_y = 0;
float current_position_cruise_last_position_yaw = 0;
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
	setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = current_position_cruise_last_position_x + x;
	setpoint_raw.position.y = current_position_cruise_last_position_y + y;
	setpoint_raw.position.z = z;
	setpoint_raw.yaw = yaw;
	double current_yaw, a, b;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(a, b, current_yaw);

	std::cout << "x: " << local_pos.pose.pose.position.x << std::endl;

	if (fabs(local_pos.pose.pose.position.x - current_position_cruise_last_position_x - x) < error_max && fabs(local_pos.pose.pose.position.y - current_position_cruise_last_position_y - y) < error_max && fabs(current_yaw - yaw) < 0.1)
	{
		ROS_INFO("到达目标点，巡航点任务完成");
		current_position_cruise_flag = false;
		return true;
	}
	return false;
}

/************************************************************************
函数功能: black_square
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/

float fx = 531.256452402983;
float fy = 531.062457146004;
float cx = 331.091420250469;
float cy = 248.782850955389;

float black_target_x = 0;
float black_target_y = 0;
float black_dx = 0;
float black_dy = 0;

void black_square_cb(const geometry_msgs::Point::ConstPtr &msg){

	if(msg->x<0 ||msg->y <0)
	{
		return;
	}
	black_dx = (cy - msg->y) * (local_pos.pose.pose.position.z + 0.03)/fy;
	black_dy = (cx - msg->x) * (local_pos.pose.pose.position.z + 0.03)/fx;
	black_target_x = (cy - msg->y) * (local_pos.pose.pose.position.z + 0.03)/fy + local_pos.pose.pose.position.x - 0.08;
    black_target_y = (cx - msg->x) * (local_pos.pose.pose.position.z + 0.03)/fx + local_pos.pose.pose.position.y;
	return;
}



/************************************************************************
函数功能: yolov8
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
bool start_checking = false;//是否开始yolo检测
bool found = false;//是否发现目标
float box_target_x = 0;
float box_target_y = 0;
// std::map<std::string, int> class_map = {
// 	{"apple", 0}, {"aquarium_fish", 1}, {"baby", 2}, {"bear", 3}, {"beaver", 4},
// 	{"bed", 5}, {"bee", 6}, {"beetle", 7}, {"bicycle", 8}, {"bottle", 9},
// 	{"bowl",10}, {"boy", 11},{"bridge", 12}, {"bus", 13}, {"butterfly", 14}, {"camel", 15},
// 	{"can", 16}, {"castle", 17}, {"caterpillar", 18}, {"cattle", 19}, {"chair", 20},
// 	{"chimpanzee", 21}, {"clock", 22}, {"cloud", 23}, {"cockroach", 24},
// 	{"couch", 25}, {"crab", 26}, {"crocodile", 27}, {"cup", 28}, {"dinosaur", 29},
// 	{"dolphin", 30}, {"elephant", 31}, {"flatfish", 32}, {"forest", 33},
// 	{"fox", 34}, {"girl", 35}, {"hamster", 36}, {"house", 37}, {"kangaroo", 38},
// 	{"keyboard", 39}, {"lamp", 40}, {"lawn_mower", 41}, {"leopard", 42},
// 	{"lion", 43}, {"lizard", 44}, {"lobster", 45}, {"man", 46}, {"maple_tree", 47},
// 	{"motorcycle", 48}, {"mountain", 49}, {"mouse", 50}, {"mushroom", 51},
// 	{"oak_tree", 52}, {"orange", 53}, {"orchid", 54}, {"otter", 55}, {"palm_tree", 56},
// 	{"pear", 57}, {"pickup_truck", 58}, {"pine_tree", 59}, {"plain", 60},
// 	{"plate", 61}, {"poppy", 62}, {"porcupine", 63}, {"possum", 64}, {"rabbit", 65},
// 	{"raccoon", 66}, {"ray", 67}, {"road", 68}, {"rocket", 69}, {"rose", 70},
// 	{"sea", 71}, {"seal", 72}, {"shark", 73}, {"shrew", 74}, {"skunk", 75},
// 	{"skyscraper", 76}, {"snail", 77}, {"snake",78 },{"spider",79 },{"squirrel",80 },
// 	{"streetcar",81 },{"sunflower",82 },{"sweet_pepper",83 },{"table",84 },
// 	{"tank",85 },{"telephone",86 },{"television",87 },{"tiger",88 },{"tractor",89 },
// 	{"train",90 },{"trout",91 },{"tulip",92 },{"turtle",93 },{"wardrobe",94 },
// 	{"whale",95 },{"willow_tree",96 },{"wolf",97 },{"woman", 98}, {"worm", 99}
// }

// float windows[100] = {0};
// int win_count = 0;
std::string have_found;

// void reset_yolo(){
// 	start_checking = false;
// 	found = false;
// 	for(int i=0;i<100;i++){
// 		windows[i] = 0;
// 	}
// 	win_count = 0;
// 	have_found = "";
// }

// void yolo_ros_cb(const yolov8_ros_msgs::BoundingBoxes::ConstPtr &msg){    
//     if(!start_checking) return;
// 	found = false;   
//     for(yolov8_ros_msgs::BoundingBox bounding_box:msg->bounding_boxes)
//     {        
//     	std::cout<<"CLASS: "<<bounding_box.Class<<std::endl;
// 		if(bounding_box.Class.empty()) continue; // 如果类别为空，跳过该框
// 		else {
// 			for(int i=0;i<100;i++){
// 				if(bounding_box.Class == classes[i]){
// 					windows[i]+=bounding_box.confidence;
// 					break;
// 				}
// 			}
// 		}
//     }

// }


float dx0 = 0;
float dy = 0;

float center_x = 0;
float center_y = 0;

float yolo_x = 0;
float yolo_y = 0;


void yolo_ros_cb(const yolov8_ros_msgs::BoundingBoxes::ConstPtr &msg){    
    if(!start_checking) return;
	found = false;   
    for(yolov8_ros_msgs::BoundingBox bounding_box:msg->bounding_boxes)
    {        
    	std::cout<<"CLASS: "<<bounding_box.Class<<std::endl;
		if(bounding_box.Class.empty()) continue; // 如果类别为空，跳过该框
		for(string target:read_data){
			if(target == bounding_box.Class)
			{
				have_found = target;
				found = true;
				center_x = bounding_box.xmin;
				center_y = bounding_box.ymin;
				dx0 = (cy - center_y) * (local_pos.pose.pose.position.z + 0.03)/fy;
				dy = (cx - center_x) * (local_pos.pose.pose.position.z + 0.03)/fx;
				yolo_x = bounding_box.xmin;
				yolo_y = bounding_box.ymin;

				// box_target_x = (cy - center_y) * (local_pos.pose.pose.position.z + 0.03)/fy + local_pos.pose.pose.position.x - 0.08;
				// box_target_y = (cx - center_x) * (local_pos.pose.pose.position.z + 0.03)/fx + local_pos.pose.pose.position.y;
				break;
			}

		}
    }

}

float black_shred = 0.3;

bool judge_box_target(float target_x,float target_y)
{
	float centerbox_x = 0,centerbox_y = 0;
	// if(black_square_x < 0 || black_square_y < 0)
	// {
	// 	centerbox_x = yolo_x;
	// 	centerbox_y = yolo_y;
	// 	ROS_WARN("using yolo to calculate 3D point");
	// }
	// else
	// {
	// 	centerbox_x = black_square_x;
	// 	centerbox_y = black_square_y;
	// 	ROS_WARN("using black_square_method to calculate 3D point");
	// }
	// box_target_x = (cy - centerbox_y) * (local_pos.pose.pose.position.z + 0.03)/fy + local_pos.pose.pose.position.x - 0.08;
    // box_target_y = (cx - centerbox_x) * (local_pos.pose.pose.position.z + 0.03)/fx + local_pos.pose.pose.position.y;

	//test_open
	if(calculate(target_x,target_y,black_target_x,black_target_y) > black_shred)
	{
		box_target_x = local_pos.pose.pose.position.x;
		box_target_y = local_pos.pose.pose.position.y;
		return false;
	}

	
	if(black_target_x < 0 || black_target_y < 0)
	{
		box_target_x = local_pos.pose.pose.position.x;
		box_target_y = local_pos.pose.pose.position.y;
		ROS_WARN("AUTO-------------------");
		return false;
	}
	else
	{
		box_target_x = black_target_x;
    	box_target_y = black_target_y;
		ROS_WARN("BLACK_SQUARE------");
		ROS_WARN("SQUARE_TARGET:black_dx:%f,black_dy:%f,black_target_x:%f,black_target_dy:%f",black_dx,black_dy,black_target_x,black_target_y);
		ROS_WARN("SQUARE_TARGET:black_dx:%f,black_dy:%f,black_target_x:%f,black_target_dy:%f",black_dx,black_dy,black_target_x,black_target_y);
		ROS_WARN("SQUARE_TARGET:black_dx:%f,black_dy:%f,black_target_x:%f,black_target_dy:%f",black_dx,black_dy,black_target_x,black_target_y);
		return true;
	}
}

bool mission_view(float x,float y,float z,float yaw);
bool mission_view(float x,float y,float z,float yaw){
	setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = x + init_position_x_take_off;
	setpoint_raw.position.y = y + init_position_y_take_off;
	setpoint_raw.position.z = z + init_position_z_take_off;
	setpoint_raw.yaw = yaw;
	if(found){
		for(string& target:read_data)
		{
			if(target == have_found)
			{
				target = "";
			}
		}
		return true;
	}
	else{ 
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
	setpoint_raw.position.z = ego_sub.position.z;
	setpoint_raw.yaw = ego_sub.yaw;

	ROS_INFO("ego: vel_x = %.2f, vel_y = %.2f, z = %.2f, yaw = %.2f", ego_sub.velocity.x, ego_sub.velocity.y, ego_sub.position.z, ego_sub.yaw);
	ROS_INFO("ego: x = %.2f, y = %.2f, z = %.2f, yaw = %.2f", ego_sub.position.x, ego_sub.position.y, ego_sub.position.z, ego_sub.yaw);
	ROS_INFO("已触发控制器: vel_x = %.2f, vel_y = %.2f, z = %.2f, yaw = %.2f", setpoint_raw.velocity.x, setpoint_raw.velocity.y, setpoint_raw.position.z, setpoint_raw.yaw);
}

/************************************************************************
函数功能:ego_planner发布目标点函数
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float before_ego_pose_x = 0;
float before_ego_pose_y = 0;
float before_ego_pose_z = 0;
bool pub_ego_goal_flag = false;
bool pub_ego_goal(float x, float y, float z, float err_max, int first_target = 0);
bool pub_ego_goal(float x, float y, float z, float err_max, int first_target)
{
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

		planner_goal_pub.publish(target_point);
		ROS_INFO("发送目标点( %f, %f, %f )", x, y, z);

		finish_ego_flag.data = false;
		finish_ego_pub.publish(finish_ego_flag);
	}

	if (rec_traj_flag == true)
	{
		PI_attitude_control();

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



