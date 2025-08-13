// 说明：舵机方向根据实际调整，这里用的占空比0%表示闭合，100%表示全部打开
#include <lib_library.h>
#include <complete_mission.h>

int mission_num = 0;
int tmp_marker_id = 1;

// the first door
float target1_x = 0, target1_y = 0;
// four ar targets
float target2_x = 0, target2_y = 0;
float target3_x = 0, target3_y = 0;
float target4_x = 0, target4_y = 0;
float target5_x = 0, target5_y = 0;
// the second door
float target6_x = 0, target6_y = 0;
// land ar target
float target7_x = 0, target7_y = 0;
// between 4 and 5 ar
float tmp_target_x = 0, tmp_target_y = 0;
// land target
float land_target_x = 0, land_target_y = 0;
// ar track err_max
float ar_track_err_max = 0;

float fly_height = 0.75;

int land_marker_id = 0;

void print_param()
{
  std::cout << "target1 : ( " << target1_x << ", " << target1_y << " )" << std::endl;
  std::cout << "target2 : ( " << target2_x << ", " << target2_y << " )" << std::endl;
  std::cout << "target3 : ( " << target3_x << ", " << target3_y << " )" << std::endl;
  std::cout << "target4 : ( " << target4_x << ", " << target4_y << " )" << std::endl;
  std::cout << "tmp_target : ( " << tmp_target_x << ", " << tmp_target_y << " )" << std::endl;
  std::cout << "target5 : ( " << target5_x << ", " << target5_y << " )" << std::endl;
  std::cout << "target6 : ( " << target6_x << ", " << target6_y << " )" << std::endl;
  std::cout << "target7 : ( " << target7_x << ", " << target7_y << " )" << std::endl;
  std::cout << "land_target : ( " << land_target_x << ", " << land_target_y << " )" << std::endl;
  std::cout << "land_ar_id : " << land_marker_id << std::endl;
  std::cout << "ar_track_err_max : " << ar_track_err_max << std::endl;
  std::cout << "track_speed : " << track_speed << std::endl;
}

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

  ros::Subscriber rec_traj_sub = nh.subscribe("/rec_traj", 100, rec_traj_cb);

  // 创建一个Subscriber订阅者，订阅名为/mavros/state的topic，注册回调函数state_cb
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

  // 创建一个Subscriber订阅者，订阅名为/mavros/local_position/odom的topic，注册回调函数local_pos_cb
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  // 订阅move_base规划出来的速度，用于避障
  // ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmd_to_px4);

  // 订阅二维码实时位置信息
  ros::Subscriber ar_pos_sub = nh.subscribe("/ar_pose_marker", 100, ar_marker_cb);

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

  //  参数读取
  nh.param<float>("target1_x", target1_x, 0);
  nh.param<float>("target1_y", target1_y, 0);
  nh.param<float>("target2_x", target2_x, 0);
  nh.param<float>("target2_y", target2_y, 0);
  nh.param<float>("target3_x", target3_x, 0);
  nh.param<float>("target3_y", target3_y, 0);
  nh.param<float>("target4_x", target4_x, 0);
  nh.param<float>("target4_y", target4_y, 0);
  nh.param<float>("target5_x", target5_x, 0);
  nh.param<float>("target5_y", target5_y, 0);
  nh.param<float>("target6_x", target6_x, 0);
  nh.param<float>("target6_y", target6_y, 0);
  nh.param<float>("target7_x", target7_x, 0);
  nh.param<float>("target7_y", target7_y, 0);
  nh.param<float>("tmp_target_x", tmp_target_x, 0);
  nh.param<float>("tmp_target_y", tmp_target_y, 0);
  nh.param<float>("track_speed", track_speed, 0);
  nh.param<float>("ar_track_err_max", ar_track_err_max, 0);
  nh.param<int>("land_marker_id", land_marker_id, 0);
  nh.param<float>("land_target_x", land_target_x, 0);
  nh.param<float>("land_target_y", land_target_y, 0);
  print_param();

  int choice = 0;
  std::cout << "1 to go on , else to quit" << std::endl;
  std::cin >> choice;
  if (choice != 1)
    return 0;

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
      printf("change mission_num = %d\r\n", mission_num);
    }

    printf("mission_num = %d\r\n", mission_num);
    switch (mission_num)
    {
    case 1:
      if (mission_pos_cruise(0, 0, ALTITUDE, 0, 0.2))
      {
        if (lib_time_record_func(1.0, ros::Time::now()))
          mission_num = 2;
      }
      break;

    // cross the first door
    case 2:
      if (pub_ego_goal(target1_x, target1_y, ALTITUDE, 0.2, 1))
      {
        // if (lib_time_record_func(1.0, ros::Time::now()))
        // {
          finish_ego = false;
          mission_num = 3;
        // }
      }
      break;

    // 1 ar_target
    case 3:
      if (mission_pos_cruise(target2_x, target2_y, fly_height, 0, 0.2))
      {
        if (lib_time_record_func(1, ros::Time::now()))
          mission_num = 4;
      }
      else
      {
        if (lib_time_record_func(8.0, ros::Time::now()))
        {
          std::cout << "timeout" << std::endl;
          mission_num = 4;
        }
      }
      break;

      // case 31:
      //   if (current_position_cruise(0, 0, ALTITUDE, 3.14 / 2, 0.2))
      //   {
      //     if (lib_time_record_func(1.0, ros::Time::now()))
      //       mission_num = 4;
      //   }
      //   break;

    case 4:
      if (ar_lable_land(ar_track_err_max, marker.id, fly_height))
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
          mission_num = 5;
          ar_lable_land_init_position_flag = false;
        }
      }
      else
      {
        if (lib_time_record_func(20.0, ros::Time::now()))
        {
          if (tmp_time_record_start_flag == false)
          {
            tmp_time_record_start_flag = true;
            lib_pwm_control(100, 100);
            ctrl_pwm_client.call(lib_ctrl_pwm);
            ROS_INFO("cs开启激光笔");
            tmp_mission_success_time_record = ros::Time::now();
          }
          if (ros::Time::now() - tmp_mission_success_time_record > ros::Duration(1.0))
          {
            lib_pwm_control(0, 0);
            ctrl_pwm_client.call(lib_ctrl_pwm);
            ROS_INFO("cs关闭激光笔");
            tmp_time_record_start_flag = false;
            ar_lable_land_init_position_flag = false;
            mission_num = 5;
          }
        }
      }
      break;

    // 2 ar_target
    case 5:
      if (pub_ego_goal(target3_x, target3_y, fly_height, 0.2))
      {
        // if (lib_time_record_func(1.0, ros::Time::now()))
        // {
          finish_ego = false;
          mission_num = 51;
        // }
      }
      break;

    case 51:
      if (current_position_cruise(0, 0, fly_height, 3.14/2, 0.2))
      {
        if (lib_time_record_func(1.0, ros::Time::now()))
          mission_num = 52;
      }
      else
      {
        if (lib_time_record_func(5.0, ros::Time::now()))
        {
          std::cout << "timeout" << std::endl;
          mission_num = 52;
        }
      }
      break;

    case 52:
      if (current_position_cruise(0, 0, fly_height, 0, 0.2))
      {
        if (lib_time_record_func(1.0, ros::Time::now()))
          mission_num = 53;
      }
      else
      {
        if (lib_time_record_func(5.0, ros::Time::now()))
        {
          std::cout << "timeout" << std::endl;
          mission_num = 53;
        }
      }
      break;

    case 53:
      if (mission_pos_cruise(target3_x, target3_y, fly_height, 0, 0.2))
      {
        if (lib_time_record_func(1, ros::Time::now()))
          mission_num = 6;
      }
      else
      {
        if (lib_time_record_func(8.0, ros::Time::now()))
        {
          std::cout << "timeout" << std::endl;
          mission_num = 6;
        }
      }
      break;

    case 6:
      if (ar_lable_land(ar_track_err_max, marker.id, fly_height))
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
          mission_num = 7;
          ar_lable_land_init_position_flag = false;
        }
      }
      else
      {
        if (lib_time_record_func(20.0, ros::Time::now()))
        {
          if (tmp_time_record_start_flag == false)
          {
            tmp_time_record_start_flag = true;
            lib_pwm_control(100, 100);
            ctrl_pwm_client.call(lib_ctrl_pwm);
            ROS_INFO("cs开启激光笔");
            tmp_mission_success_time_record = ros::Time::now();
          }
          if (ros::Time::now() - tmp_mission_success_time_record > ros::Duration(1.0))
          {
            lib_pwm_control(0, 0);
            ctrl_pwm_client.call(lib_ctrl_pwm);
            ROS_INFO("cs关闭激光笔");
            tmp_time_record_start_flag = false;
            mission_num = 7;
            ar_lable_land_init_position_flag = false;
          }
          
        }
      }
      break;

    // 3 ar_target开启激光笔
    case 7:
      if (pub_ego_goal(target4_x, target4_y, fly_height, 0.2))
      {
        // if (lib_time_record_func(1.0, ros::Time::now()))
        // {
          finish_ego = false;
          mission_num = 71;
        // }
      }
      break;

    case 71:
      if (current_position_cruise(0, 0, fly_height, 0, 0.2))
      {
        if (lib_time_record_func(1.0, ros::Time::now()))
          mission_num = 72;
      }
      else
      {
        if (lib_time_record_func(5.0, ros::Time::now()))
        {
          std::cout << "timeout" << std::endl;
          mission_num = 72;
        }
      }
      break;

    case 72:
      if (mission_pos_cruise(target4_x, target4_y, fly_height, 0, 0.2))
      {
        if (lib_time_record_func(1, ros::Time::now()))
          mission_num = 8;
      }
      else
      {
        if (lib_time_record_func(8.0, ros::Time::now()))
        {
          std::cout << "timeout" << std::endl;
          mission_num = 8;
        }
      }
      break;

    case 8:
      if (ar_lable_land(ar_track_err_max, marker.id, fly_height))
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
          mission_num = 90;
          ar_lable_land_init_position_flag = false;
        }
      }
      else
      {
        if (lib_time_record_func(20.0, ros::Time::now()))
        {
          if (tmp_time_record_start_flag == false)
          {
            tmp_time_record_start_flag = true;
            lib_pwm_control(100, 100);
            ctrl_pwm_client.call(lib_ctrl_pwm);
            ROS_INFO("cs开启激光笔");
            tmp_mission_success_time_record = ros::Time::now();
          }
          if (ros::Time::now() - tmp_mission_success_time_record > ros::Duration(1.0))
          {
            lib_pwm_control(0, 0);
            ctrl_pwm_client.call(lib_ctrl_pwm);
            ROS_INFO("cs关闭激光笔");
            tmp_time_record_start_flag = false;
            mission_num = 90;
            ar_lable_land_init_position_flag = false;
          }
         
        }
      }
      break;

    // temp target
    case 90:
      if (mission_pos_cruise(tmp_target_x, tmp_target_y, fly_height, 0, 0.2))
      {
        //   if (lib_time_record_func(1.0, ros::Time::now()))
        mission_num = 9;
      }
      else
      {
        if (lib_time_record_func(10.0, ros::Time::now()))
        {
          std::cout << "timeout" << std::endl;
          mission_num = 9;
        }
      }
      break;

    // 4 ar_target
    case 9:
      if (pub_ego_goal(target5_x, target5_y, fly_height, 0.2))
      {
        // if (lib_time_record_func(1.0, ros::Time::now()))
        // {
          finish_ego = false;
          mission_num = 100;
        // }
      }
      break;

    case 100:
      if (current_position_cruise(0, 0, fly_height, 3.14/2, 0.2))
      {
        if (lib_time_record_func(1.0, ros::Time::now()))
          mission_num = 101;
      }
      else
      {
        if (lib_time_record_func(5.0, ros::Time::now()))
        {
          mission_num = 101;
        }
      }
      break;

    case 101:
      if (current_position_cruise(0, 0, fly_height, 3.14, 0.2))
      {
        if (lib_time_record_func(1.0, ros::Time::now()))
          mission_num = 102;
      }
      else
      {
        if (lib_time_record_func(5.0, ros::Time::now()))
        {
          mission_num = 102;
        }
      }
      break;

    case 102:
      if (mission_pos_cruise(target5_x, target5_y, fly_height, 3.14, 0.2))
      {
        if (lib_time_record_func(1, ros::Time::now()))
          mission_num = 10;
      }
      else
      {
        if (lib_time_record_func(5.0, ros::Time::now()))
        {
          mission_num = 10;
        }
      }
      break;

    case 10:
      if (ar_lable_land(ar_track_err_max, marker.id, fly_height, 1))
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
          mission_num = 11;
          ar_lable_land_init_position_flag = false;
        }
      }
      else
      {
        if (lib_time_record_func(20.0, ros::Time::now()))
        {
          if (tmp_time_record_start_flag == false)
          {
            tmp_time_record_start_flag = true;
            lib_pwm_control(100, 100);
            ctrl_pwm_client.call(lib_ctrl_pwm);
            ROS_INFO("cs开启激光笔");
            tmp_mission_success_time_record = ros::Time::now();
          }
          if (ros::Time::now() - tmp_mission_success_time_record > ros::Duration(1.0))
          {
            lib_pwm_control(0, 0);
            ctrl_pwm_client.call(lib_ctrl_pwm);
            ROS_INFO("cs关闭激光笔");
            tmp_time_record_start_flag = false;
            mission_num = 11;
            ar_lable_land_init_position_flag = false;
          }
          
        }
      }
      break;

    // case 110:
    //   if (current_position_cruise(0, 0, ALTITUDE, 3.14, 0.2))
    //   {
    //     if (lib_time_record_func(1.0, ros::Time::now()))
    //       mission_num = 11;
    //   }
    //   break;

    // before the second door
    case 11:
      if (mission_pos_cruise(target6_x, target6_y, ALTITUDE, 3.14, 0.2))
      {
        if (lib_time_record_func(1.0, ros::Time::now()))
          mission_num = 12;
      }
      break;

    // cross the second door, the target ar
    case 12:
      if (pub_ego_goal(target7_x, target7_y, ALTITUDE, 0.2, 2))
      {
        // if (lib_time_record_func(1.0, ros::Time::now()))
        // {
          finish_ego = false;
          mission_num = 1000;
        // }
      }
      break;

    case 1000:
      if (mission_pos_cruise(land_target_x, land_target_y, fly_height, 3.14, 0.2))
      {
        if (lib_time_record_func(1.0, ros::Time::now()))
        {
          mission_num = 2000;
        }
      }
      else
      {
        if (lib_time_record_func(20.0, ros::Time::now()))
        {
          std::cout << "timeout" << std::endl;
          mission_num = 2000;
        }
      }
      break;

    case 2000:
      if (current_position_cruise(0, 0, fly_height, 3.14/2, 0.2))
      {
        if (lib_time_record_func(1.0, ros::Time::now()))
          mission_num = 3000;
      }
      else
      {
        if (lib_time_record_func(5.0, ros::Time::now()))
        {
          std::cout << "timeout" << std::endl;
          mission_num = 3000;
        }
      }
      break;

    case 3000:
      if (current_position_cruise(0, 0, fly_height, 0, 0.2))
      {
        if (lib_time_record_func(1.0, ros::Time::now()))
          mission_num = 13;
      }
      else
      {
        if (lib_time_record_func(5.0, ros::Time::now()))
        {
          std::cout << "timeout" << std::endl;
          mission_num = 13;
        }
      }
      break;


    case 13:
      if (ar_lable_land(ar_track_err_max, land_marker_id, fly_height, 1))
      {
        if (lib_time_record_func(0.1, ros::Time::now()))
        {
          mission_num = 14;
          ar_lable_land_init_position_flag = false;
        }
      }
      else
      {
        if (lib_time_record_func(20.0, ros::Time::now()))
        {
          mission_num = 14;
          ar_lable_land_init_position_flag = false;
        }
      }
      break;

    case 14:
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
