#include <complete_mission.hpp>

int mission_num = 0;

int main(int argc, char **argv)
{
	// 防止中文输出乱码
	setlocale(LC_ALL, "");

	ros::init(argc, argv, "complete_mission");

	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

	ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

	ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("goal", 10, goal_cb);

	// ros::Subscriber IPC_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("IPC_result", 100, IPC_cb);
	ros::Subscriber IPC_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("planning/pos_cmd", 100, IPC_cb);

	goal_pub = nh.advertise<geometry_msgs::PoseStamped>("goal_for_ipc", 100);

	ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 100);

	// attitude control
	ros::Publisher mavros_setpoint_att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 100);

	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	// 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
	ros::Rate rate(20);

	// 等待连接到飞控
	while (ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
		rate.sleep();
	}
	// 设置无人机的期望位置
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = 0;
	setpoint_raw.position.y = 0;
	setpoint_raw.position.z = ALTITUDE;
	setpoint_raw.yaw = 0;

	// send a few setpoints before starting
	for (int i = 100; ros::ok() && i > 0; --i)
	{
		mavros_setpoint_pos_pub.publish(setpoint_raw);
		ros::spinOnce();
		rate.sleep();
	}

	// 定义客户端变量，设置为offboard模式
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	// 定义客户端变量，请求无人机解锁
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	// 记录当前时间，并赋值给变量last_request
	ros::Time last_request = ros::Time::now();

	while (ros::ok())
	{
		if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
		{
			if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
			{
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		}
		else
		{
			if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
			{
				if (arming_client.call(arm_cmd) && arm_cmd.response.success)
				{
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}
		// 当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
		if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.2)
		{
			if (ros::Time::now() - last_request > ros::Duration(3.0))
			{
				mission_num = 1;
				break;
			}
		}
		setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
		setpoint_raw.coordinate_frame = 1;
		setpoint_raw.position.x = 0;
		setpoint_raw.position.y = 0;
		setpoint_raw.position.z = ALTITUDE;
		setpoint_raw.yaw = 0;
		mavros_setpoint_pos_pub.publish(setpoint_raw);
		ros::spinOnce();
		rate.sleep();
	}

	goal_for_ipc.pose.position.x = 0;
	goal_for_ipc.pose.position.y = 0;
	goal_for_ipc.pose.position.z = 1;

	while (ros::ok())
	{
		if (get_ipc_goal_flag == false)
		{
			// current_position_cruise(0, 0, 0, 0, 0.2);
			mission_pos_cruise(goal_for_ipc.pose.position.x, goal_for_ipc.pose.position.y, goal_for_ipc.pose.position.z, 0, 0.2);
			// hover();
			mavros_setpoint_pos_pub.publish(setpoint_raw);
			ROS_INFO("HOVER");
		}
		else
		{
			IPC_control();
			// mavros_setpoint_att_pub.publish(setpoint_att);
			mavros_setpoint_pos_pub.publish(setpoint_raw);
		}
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
