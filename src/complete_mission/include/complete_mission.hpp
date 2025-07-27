#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <mavros_msgs/CommandLong.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <mavros_msgs/AttitudeTarget.h>

using namespace std;

#define ALTITUDE 0.5

mavros_msgs::PositionTarget setpoint_raw;
mavros_msgs::AttitudeTarget setpoint_att;
ros::Publisher planner_goal_pub;
ros::Publisher finish_ego_pub;
ros::Publisher goal_pub;
std_msgs::Bool finish_ego_flag;

float map_size_z = 0.8;
float ground_height = 0.4;

bool get_ipc_goal_flag = false;
bool takeoff_flag = false;
bool reach_ipc_flag = false;

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
	setpoint_raw.position.x = init_position_x_take_off + x;
	setpoint_raw.position.y = init_position_y_take_off + y;
	setpoint_raw.position.z = init_position_z_take_off + z;
	setpoint_raw.yaw = yaw;
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
float current_position_cruise_last_position_z = 0;
float current_position_cruise_last_position_yaw = 0;
double current_position_cruise_last_yaw = 0;
bool current_position_cruise_flag = false;
bool current_position_cruise(float x, float y, float z, float yaw, float error_max);
bool current_position_cruise(float x, float y, float z, float yaw, float error_max)
{
	if (current_position_cruise_flag == false)
	{
		double yaw, a, b;
		tf::Quaternion quat;
		tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
		tf::Matrix3x3(quat).getRPY(a, b, yaw);
		current_position_cruise_last_position_x = local_pos.pose.pose.position.x;
		current_position_cruise_last_position_y = local_pos.pose.pose.position.y;
		current_position_cruise_last_position_z = local_pos.pose.pose.position.z;
		current_position_cruise_last_yaw = yaw;
		current_position_cruise_flag = true;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = current_position_cruise_last_position_x + x;
	setpoint_raw.position.y = current_position_cruise_last_position_y + y;
	setpoint_raw.position.z = current_position_cruise_last_position_z + z;
	setpoint_raw.yaw = current_position_cruise_last_yaw + yaw;
	// setpoint_raw.yaw = 0;
	double current_yaw, a, b;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(a, b, current_yaw);

	if (fabs(local_pos.pose.pose.position.x - current_position_cruise_last_position_x - x) < error_max && fabs(local_pos.pose.pose.position.y - current_position_cruise_last_position_y - y) < error_max && fabs(current_yaw - current_position_cruise_last_yaw - yaw) < 0.1)
	{
		// ROS_INFO("到达目标点，巡航点任务完成");
		// current_position_cruise_flag = false;
		return true;
	}
	return false;
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
bool pub_ego_goal(float x, float y, float z, float err_max);
bool pub_ego_goal(float x, float y, float z, float err_max)
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
		ROS_INFO("发送目标点: (%f, %f, %f)", x, y, z);

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

		setpoint_raw.header.stamp = ros::Time::now();

		// setpoint_raw.type_mask = 0b100111000000; // 100 111 000 000  vx vy　vz x y z+ yaw
		// setpoint_raw.coordinate_frame = 1;
		// setpoint_raw.position.x = ego_sub.position.x;
		// setpoint_raw.position.y = ego_sub.position.y;
		// setpoint_raw.position.z = ego_sub.position.z;
		// setpoint_raw.velocity.x = ego_sub.velocity.x;
		// setpoint_raw.velocity.y = ego_sub.velocity.y;
		// setpoint_raw.velocity.z = ego_sub.velocity.z;

		std::cout << "get traj" << std::endl;
		std::cout << "ego_x : " << ego_sub.position.x << std::endl;
		std::cout << "ego_y : " << ego_sub.position.y << std::endl;
		std::cout << "ego_z : " << ego_sub.position.z << std::endl;
		std::cout << "ego_yaw : " << ego_sub.yaw << std::endl;
	}
	else
	{
	std:
		cout << "no rec_traj" << std::endl;
		setpoint_raw.position.x = before_ego_pose_x;
		setpoint_raw.position.y = before_ego_pose_y;
		setpoint_raw.position.z = before_ego_pose_z;
		setpoint_raw.yaw = current_yaw;
		setpoint_raw.type_mask = 0b100111111000; // 100 111 111 000  x y z+ yaw
		setpoint_raw.coordinate_frame = 1;
	}

	if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < err_max && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < err_max)
	{
		ROS_INFO("到达目标点, ego_planner导航任务完成");
		pub_ego_goal_flag = false;

		finish_ego_flag.data = true;
		finish_ego_pub.publish(finish_ego_flag);

		return true;
	}
	return false;
}

/**
 * 函数功能: 转发来自rviz的设定目标点给IPC
 */
geometry_msgs::PoseStamped goal_for_ipc;
void goal_cb(const geometry_msgs::PoseStampedConstPtr &msg)
{
	get_ipc_goal_flag = true;
	goal_for_ipc.pose.position.x = msg->pose.position.x;
	goal_for_ipc.pose.position.y = msg->pose.position.y;
	// goal_for_ipc.pose.position.z = msg->pose.position.z;
	goal_for_ipc.pose.position.z = ALTITUDE;
	goal_pub.publish(goal_for_ipc);
}

/**
 * 函数功能：hover
 */
void hover()
{
	double current_yaw, a, b;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(a, b, current_yaw);

	setpoint_raw.type_mask = 0b101111000111; // vx vy z yaw
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.velocity.x = 0;
	setpoint_raw.velocity.y = 0;
	setpoint_raw.velocity.z = 0;
	setpoint_raw.yaw = 0;

	ROS_INFO("hover");
}

/**
 * 函数功能：得到IPC规划结果
 */
// mavros_msgs::AttitudeTarget IPC_result;
quadrotor_msgs::PositionCommand IPC_result;
void IPC_cb(const quadrotor_msgs::PositionCommandConstPtr &msg)
{
	// get_ipc_flag = true;
	IPC_result.position.x = msg->position.x;
	IPC_result.position.y = msg->position.y;
	IPC_result.position.z = msg->position.z;
	IPC_result.velocity.x = msg->velocity.x;
	IPC_result.velocity.y = msg->velocity.y;
	IPC_result.velocity.z = msg->velocity.z;
	IPC_result.acceleration.x = msg->acceleration.x;
	IPC_result.acceleration.y = msg->acceleration.y;
	IPC_result.acceleration.z = msg->acceleration.z;
	IPC_result.yaw = msg->yaw;
}

// void IPC_cb(const mavros_msgs::AttitudeTargetConstPtr &msg)
// {
// 	// get_ipc_flag = true;
// 	IPC_result.type_mask = msg->type_mask;
// 	IPC_result.body_rate.x = msg->body_rate.x;
// 	IPC_result.body_rate.y = msg->body_rate.y;
// 	IPC_result.body_rate.z = msg->body_rate.z;
// 	IPC_result.thrust = msg->thrust;
// }

/**
 * 函数功能：IPC控制
 */
void IPC_control()
{
	// setpoint_att.type_mask = IPC_result.type_mask;
	// setpoint_att.body_rate.x = IPC_result.body_rate.x;
	// setpoint_att.body_rate.y = IPC_result.body_rate.y;
	// setpoint_att.body_rate.z = IPC_result.body_rate.z;
	// setpoint_att.thrust = IPC_result.thrust;

	// ROS_INFO("x: %.2f, y: %.2f, z: %.2f, thrust: %.2f", setpoint_att.body_rate.x, setpoint_att.body_rate.y, setpoint_att.body_rate.z, setpoint_att.thrust);

	setpoint_raw.type_mask = 0b101111000000; // 
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = IPC_result.position.x;
	setpoint_raw.position.y = IPC_result.position.y;
	setpoint_raw.position.z = IPC_result.position.z;
	setpoint_raw.velocity.x = IPC_result.velocity.x;
	setpoint_raw.velocity.y = IPC_result.velocity.y;
	setpoint_raw.velocity.z = IPC_result.velocity.z;
	setpoint_raw.acceleration_or_force.x = IPC_result.acceleration.x;
	setpoint_raw.acceleration_or_force.y = IPC_result.acceleration.y;
	setpoint_raw.acceleration_or_force.z = IPC_result.acceleration.z;
	setpoint_raw.yaw = IPC_result.yaw;

	if(IPC_result.position.z > map_size_z)
	{
		setpoint_raw.position.z = map_size_z;
		setpoint_raw.velocity.z = 0;
	}


	ROS_INFO("fly to (%.2f, %.2f, %.2f)", goal_for_ipc.pose.position.x, goal_for_ipc.pose.position.y, goal_for_ipc.pose.position.z);
	ROS_INFO("px: %.2f, py: %.2f, pz: %.2f, vx: %.2f, vy: %.2f, vz: %.2f, ax: %.2f, ay: %.2f, az: %.2f, yaw: %.2f", setpoint_raw.position.x, setpoint_raw.position.y, setpoint_raw.position.z, setpoint_raw.velocity.x, setpoint_raw.velocity.y, setpoint_raw.velocity.z, setpoint_raw.acceleration_or_force.x, setpoint_raw.acceleration_or_force.y, setpoint_raw.acceleration_or_force.z, setpoint_raw.yaw);

	ROS_INFO("ipc_control");

	if (fabs(local_pos.pose.pose.position.x - goal_for_ipc.pose.position.x - init_position_x_take_off) < 0.2 && fabs(local_pos.pose.pose.position.y - goal_for_ipc.pose.position.y - init_position_y_take_off) < 0.2)
	{
		ROS_INFO("到达目标点，巡航点任务完成");
		get_ipc_goal_flag = false;
	}
}