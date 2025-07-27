#include <complete_mission.h>
#include <lib_library.h>
#include <collision_avoidance.hpp>

int mission_num = 0;

int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "complete_mission");

  // 创建节点句柄
  ros::NodeHandle nh;

  // 订阅ego_planner规划出来的结果
  ros::Subscriber ego_sub = nh.subscribe("/position_cmd", 100, ego_sub_cb);

  // 发布ego_planner目标
  planner_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal", 100);

  // 订阅二维码实时位置信息
  ros::Subscriber ar_pos_sub = nh.subscribe("/ar_pose_marker", 100, ar_marker_cb);

  ros::Subscriber rec_traj_sub = nh.subscribe("/rec_traj", 100, rec_traj_cb);

  // 创建一个Subscriber订阅者，订阅名为/mavros/state的topic，注册回调函数state_cb
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

  // 创建一个Subscriber订阅者，订阅名为/mavros/local_position/odom的topic，注册回调函数local_pos_cb
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);

  ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, lidar_cb);

  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

  // 创建一个服务客户端，连接名为/mavros/cmd/arming的服务，用于请求无人机解锁
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

  // 创建一个服务客户端，连接名为/mavros/set_mode的服务，用于请求无人机进入offboard模式
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // 请求控制舵机客户端
  ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  // collision
  nh.param<float>("scan_distance_max", scan_distance_max, 1.1);
  nh.param<float>("scan_distance_max_cross_door", scan_distance_max_cross_door, 0.7);
  nh.param<float>("scan_distance_min", scan_distance_min, 0.1);

  nh.param<float>("barrier_distance_max", barrier_distance_max, 0.6);
  nh.param<float>("barrier_distance_max_cross_door", barrier_distance_max_cross_door, 0.5);

  nh.param<float>("angle_resolution", angle_resolution, 1.0);
  nh.param<float>("sector_value", sector_value, 30);
  nh.param<int>("print_time", print_time, 5);

  nh.param<float>("vel", vel, 0.5);
  nh.param<float>("vel_cross_door", vel_cross_door, 0.5);

  nh.param<float>("vel_max_normal", vel_max_normal, 0.4);
  nh.param<float>("vel_max_hit", vel_max_hit, 0.03);
  nh.param<float>("vel_max_cross_door", vel_max_cross_door, 0.2);

  nh.param<std::vector<float>>("divid_line", divid_line, std::vector<float>());
  nh.param<std::vector<float>>("divid_line_cross_door", divid_line_cross_door, std::vector<float>());
  nh.param<float>("speed_mul", speed_mul, 3);

  nh.param<int>("range_min", range_min, 0.0);
  nh.param<int>("range_max", range_max, 0.0);
  nh.param<float>("lidar_max_distance", lidar_max_distance, 0.0);

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

  // 此处控制两路舵机，通道5和6，int型0-100代表占空比
  lib_pwm_control(0, 0);
  ctrl_pwm_client.call(lib_ctrl_pwm);

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
    if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.1)
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
    mission_pos_cruise(0, 0, ALTITUDE, 0, 0.2);
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }

  // 定义任务计时标志位
  bool tmp_time_record_start_flag = false;
  // 定义任务计时变量
  ros::Time tmp_mission_success_time_record;

  float tmp_x = 0;
  float tmp_y = 0;
  int tmp_mission = 1;

  while (ros::ok())
  {
    if (tmp_mission != mission_num)
    {
      tmp_mission = mission_num;
      printf("mission_num = %d\r\n", mission_num);
    }

    switch (mission_num)
    {
    case 1:
      if (mission_pos_cruise(0, 0, ALTITUDE, 0, 0.2))
      {
        if (lib_time_record_func(1.0, ros::Time::now()))
          mission_num = 2;
      }
      break;

    case 2:
      if (pub_ego_goal(3.3, 0.0, ALTITUDE, 0.2))
      {
        if (lib_time_record_func(1.0, ros::Time::now()))
          mission_num = 100;
      }
      break;

    case 3:
      if (pub_ego_goal(3.3, 2.1, ALTITUDE, 0.2))
      {
        if (lib_time_record_func(1.0, ros::Time::now()))
          mission_num = 4;
      }
      break;
    case 100:
      if (ar_lable_land(0.1, marker.id, ALTITUDE))
      {
        if (tmp_time_record_start_flag == false)
        {
          tmp_time_record_start_flag = true;
          lib_pwm_control(100, 100);
          ctrl_pwm_client.call(lib_ctrl_pwm);
          ROS_INFO("开启激光笔");
          tmp_mission_success_time_record = ros::Time::now();
        }
        if (ros::Time::now() - tmp_mission_success_time_record > ros::Duration(1.0))
        {
          lib_pwm_control(0, 0);
          ctrl_pwm_client.call(lib_ctrl_pwm);
          ROS_INFO("关闭激光笔");
          tmp_time_record_start_flag = false;
          mission_num = 3;
        }
      }
      else
      {
        if (lib_time_record_func(20.0, ros::Time::now()))
        {
          mission_num = 3;
        }
      }
      break;

    case 4:
      ROS_INFO("AUTO.LAND");
      offb_set_mode.request.custom_mode = "AUTO.LAND";
      set_mode_client.call(offb_set_mode);
      mission_num = -1;
      break;
    }
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
