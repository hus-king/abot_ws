//0603-14:30 case8，case9之前保存版本

// 说明：舵机方向根据实际调整，这里用的占空比0%表示闭合，100%表示全部打开
// RCAIC version-1
#include <lib_library.h>
#include <complete_mission.h>


int mission_num = 0;   //mission_flag
int tmp_marker_id = 1;
int target_num = 0;
float target_x[4] = {};
float target_y[4] = {};

float target1_x = 0, target1_y = 0;
float target2_x = 0, target2_y = 0;
float target3_x = 0, target3_y = 0;
float target4_x = 0, target4_y = 0;


// the four picture
float square_size = 0;

float err_max = 0;

float fly_height = 0.75;

void print_param()
{
  std::cout << "square_size" << square_size << std::endl;

  
  std::cout << "err_max : " << err_max << std::endl;
  std::cout << "fly_height : " << fly_height << std::endl;
  std::cout << "target1 : ( " << target_x[0] << ", " << target_y[0] << " )" << std::endl;
  std::cout << "target2 : ( " << target_x[1] << ", " << target_y[1] << " )" << std::endl;
  std::cout << "target3 : ( " << target_x[2] << ", " << target_y[2] << " )" << std::endl;
  std::cout << "target4 : ( " << target_x[3] << ", " << target_y[3] << " )" << std::endl;
}

int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "video_test_node");

  // 创建节点句柄
  ros::NodeHandle nh;

  // 订阅ego_planner规划出来的结果
  //ros::Subscriber ego_sub = nh.subscribe("/position_cmd", 100, ego_sub_cb);

  // 发布ego_planner目标
  //planner_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal", 100);

  //ros::Subscriber rec_traj_sub = nh.subscribe("/rec_traj", 100, rec_traj_cb);

  //finish_ego_pub = nh.advertise<std_msgs::Bool>("/finish_ego", 1);

  // 创建一个Subscriber订阅者，订阅名为/mavros/state的topic，注册回调函数state_cb
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

  // 创建一个Subscriber订阅者，订阅名为/mavros/local_position/odom的topic，注册回调函数local_pos_cb
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  // 订阅move_base规划出来的速度，用于避障
  // ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmd_to_px4);

  ros::Subscriber qr_sub = nh.subscribe("/qr_code_detection/qrcode_data", 10 , qr_cb);

  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

  // 创建一个服务客户端，连接名为/mavros/cmd/arming的服务，用于请求无人机解锁
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

  // 创建一个服务客户端，连接名为/mavros/set_mode的服务，用于请求无人机进入offboard模式
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // 请求控制舵机客户端
  ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  ros::Publisher catapult_pub_box1 = nh.advertise<std_msgs::Empty>("servo/front_left/open", 1);
  ros::Publisher catapult_pub_box2 = nh.advertise<std_msgs::Empty>("servo/front_right/open", 1);
  ros::Publisher catapult_pub_box_large = nh.advertise<std_msgs::Empty>("servo/all/open", 1);

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  //  参数读取
  nh.param<float>("square_size", square_size, 0);

  nh.param<float>("target1_x", target1_x, 0);
  nh.param<float>("target1_y", target1_y, 0);
  nh.param<float>("target2_x", target2_x, 0);
  nh.param<float>("target2_y", target2_y, 0);
  nh.param<float>("target3_x", target3_x, 0);
  nh.param<float>("target3_y", target3_y, 0);
  nh.param<float>("target4_x", target4_x, 0);
  nh.param<float>("target4_y", target4_y, 0);

  nh.param<float>("err_max", err_max, 0);
  nh.param<float>("fly_height", fly_height, 0);
  target_x[0] = target1_x;
  target_y[0] = target1_y;
  
  target_x[1] = target2_x;
  target_y[1] = target2_y;
  
  target_x[2] = target3_x;
  target_y[2] = target3_y;
  
  target_x[3] = target4_x;
  target_y[3] = target4_y;
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
  //设置无人机的期望位置
 
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
  std::cout<<"ok"<<std::endl;

  // 定义客户端变量，设置为offboard模式
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // 定义客户端变量，请求无人机解锁
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  // 记录当前时间，并赋值给变量last_request
  ros::Time last_request = ros::Time::now();

  // 此处控制两路舵机，通道5和6，int型0-100代表占空比
  //lib_pwm_control(0, 0);
  //ctrl_pwm_client.call(lib_ctrl_pwm);

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
      if (ros::Time::now() - last_request > ros::Duration(1.0))
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
  ros::Time last_time;
  bool last_time_flag = false;
  bool box_flag = false;

  float tmp_x = 0;
  float tmp_y = 0;

  int tmp_mission = 1;
  std::string read_data[3];
  std_msgs::Empty catapult_msg;

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
      //mission1:recognize qr_code
      case 1:
        if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
        {
          mission_num = 2;
        }
        break;

      case 2:
        if (mission_pos_cruise(target1_x, target1_y, ALTITUDE, 0, err_max))
        {
          if(lib_time_record_func(5, ros::Time::now()))
          {
            mission_num = 3;
          }
        }
        break;
      
      case 3:
        if (mission_pos_cruise(target2_x, target2_y, ALTITUDE, 0, err_max))
        {
          if(lib_time_record_func(5, ros::Time::now()))
          {
            mission_num = 4;
          }
        }
        break;

      case 4:
        if (mission_pos_cruise(target3_x, target3_y, ALTITUDE, 0, err_max))
        {
          if(lib_time_record_func(5, ros::Time::now()))
          {
            mission_num = 5;
          }
        }
        break;

      case 5:
        if (mission_pos_cruise(target4_x, target4_y, ALTITUDE, 0, err_max))
        {
          if(lib_time_record_func(5, ros::Time::now()))
          {
            mission_num = 100;
          }
        }
        break;

      case 100:
        arm_cmd.request.value = false;
        if(mission_pos_cruise(0,0,0.01,0,err_max)){
          if (arming_client.call(arm_cmd) && arm_cmd.response.success)
          {
            ROS_INFO("Vehicle disarmed");
            mission_num = -1;
          }
          break;
        }
    }
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
    if(mission_num == -1) break;
  }
  return 0;
}
