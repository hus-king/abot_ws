#include <lib_library.h>
#include <gjs.h>

// 全局变量定义
int mission_num = 0;        // 任务标志位
int tmp_marker_id = 1;      // 临时标记ID
int box_number = 0;         // 投放箱子数量
float now_yaw = 0;          // 当前偏航角

// 目标点坐标数组
vector<float> target_array_x;
vector<float> target_array_y;

// 各个目标点坐标
float target1_x = 0, target1_y = 0;
float target2_x = 0, target2_y = 0;
float target3_x = 0, target3_y = 0;
float target4_x = 0, target4_y = 0;
float target5_x = 0, target5_y = 0;
float target6_x = 0, target6_y = 0;
float target7_x = 0, target7_y = 0;
float target8_x = 0, target8_y = 0;
float target9_x = 0, target9_y = 0;
float if_debug = 0; // 是否开启调试模式

// 注意：只投放car，bridge，跳过pillbox，tank，tent
float track_distance;           // 追踪距离
float temp_x = 0, temp_y = 0;   // 临时坐标
float err_max = 0;              // 最大误差
float err_max_ego = 0;          // ego规划器最大误差

// 投射器相关变量
float check_catapult_x;
float check_catapult_y;
float catapult_shred;
bool ego_check = false; //是否完成进入ego前参数自检

void print_param()
{
  std::cout << "=== 目标点参数 ===" << std::endl;
  std::cout << "target1: (" << target1_x << ", " << target1_y << ")" << std::endl;
  std::cout << "target2: (" << target2_x << ", " << target2_y << ")" << std::endl;
  std::cout << "target3: (" << target3_x << ", " << target3_y << ")" << std::endl;
  std::cout << "target4: (" << target4_x << ", " << target4_y << ")" << std::endl;
  std::cout << "target5: (" << target5_x << ", " << target5_y << ")" << std::endl;
  std::cout << "target6: (" << target6_x << ", " << target6_y << ")" << std::endl;
  std::cout << "target7: (" << target7_x << ", " << target7_y << ")" << std::endl;
  std::cout << "target8: (" << target8_x << ", " << target8_y << ")" << std::endl;
  std::cout << "target9: (" << target9_x << ", " << target9_y << ")" << std::endl;
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "err_max_ego: " << err_max_ego << std::endl;
  std::cout << "track_distance: " << track_distance << std::endl;
  std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
  std::cout << "VIEW_ALTITUDE: " << VIEW_ALTITUDE << std::endl;
  
  std::cout << "=== 投射器参数 ===" << std::endl;
  std::cout << "check_catapult_x: " << check_catapult_x << std::endl;
  std::cout << "check_catapult_y: " << check_catapult_y << std::endl;
  std::cout << "catapult_shred: " << catapult_shred << std::endl;
  std::cout << "camera_height: " << camera_height << std::endl;
  std::cout << "camera_offset_body_x: " << camera_offset_body_x << std::endl;
  std::cout << "if_debug: " << if_debug << std::endl;
  std::cout << "door_step: " << door_step << std::endl;
  std::cout << "door_shred: " << door_shred << std::endl;
}


int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  // 初始化ROS节点
  ros::init(argc, argv, "gjs");
  ros::NodeHandle nh;

  // 订阅ego_planner规划出来的结果
  ros::Subscriber ego_sub = nh.subscribe("/position_cmd", 100, ego_sub_cb);
  ros::Subscriber rec_traj_sub = nh.subscribe("/rec_traj", 100, rec_traj_cb);

  // 发布ego_planner目标和完成信号
  planner_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal", 100);
  finish_ego_pub = nh.advertise<std_msgs::Bool>("/finish_ego", 1);
  ego_planner_mode_pub = nh.advertise<std_msgs::Int32>("/ego_planner_mode", 10);

  // 订阅mavros相关话题
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

  // 创建服务客户端
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  // 投射器相关发布者和订阅者
  ros::Publisher catapult_pub_box1 = nh.advertise<std_msgs::Empty>("servo/back_right/open", 1);
  ros::Publisher catapult_pub_box2 = nh.advertise<std_msgs::Empty>("servo/back_left/open", 1);
  ros::Publisher catapult_pub_box3 = nh.advertise<std_msgs::Empty>("servo/front_right/open", 1);
  // ros::Publisher catapult_pub_box_large = nh.advertise<std_msgs::Empty>("servo/all/open", 1);
  ros::Publisher catapult_pub_box_large = nh.advertise<std_msgs::Empty>("servo/all/open", 1);
  
  ros::Subscriber yolo_ros_box_sub = nh.subscribe<yolov8_ros_msgs::BoundingBoxes>("/object_position", 1, yolo_ros_cb);
  
  ros::Subscriber depth_sub = nh.subscribe<camera_processor::PointDepth>("/camera_processor/depth/points", 1, depth_image_cb);

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  // 参数读取

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
  nh.param<float>("target8_x", target8_x, 0);
  nh.param<float>("target8_y", target8_y, 0);
  nh.param<float>("target9_x", target9_x, 0);
  nh.param<float>("target9_y", target9_y, 0);

  nh.param<float>("track_distance", track_distance, 0); 
  nh.param<float>("err_max", err_max, 0);
  nh.param<float>("err_max_ego", err_max_ego, 0.3);

  nh.param<float>("check_catapult_x", check_catapult_x, 0);
  nh.param<float>("check_catapult_y", check_catapult_y, 0);
  
  nh.param<float>("catapult_shred", catapult_shred, 0);

  nh.param<double>("sigma_a", sigma_a, 0.1);
  nh.param<double>("tank_time", tank_time, 0.0);
  nh.param<double>("tank_shred", tank_shred, 0.0);

  nh.param<float>("camera_height", camera_height, 0);
  nh.param<float>("camera_offset_body_x", camera_offset_body_x, 0.0);
  nh.param<float>("if_debug", if_debug, 0);

  nh.param<float>("door_step", door_step, 2);
  nh.param<float>("door_shred", door_shred, 1);

  target_array_x.push_back(target1_x);
  target_array_y.push_back(target1_y);
  target_array_x.push_back(target2_x);
  target_array_y.push_back(target2_y);
  target_array_x.push_back(target3_x);
  target_array_y.push_back(target3_y);
  target_array_x.push_back(target4_x);
  target_array_y.push_back(target4_y);


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
      if(if_debug == 1)
      {
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
      }
      else
      {
        ROS_INFO("Waiting for OFFBOARD mode");
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
    mission_pos_cruise(0, 0, ALTITUDE, 0, err_max); 
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
  
  //用于追踪目标是否在靠近飞机
  float tmp_x = 0;
  float tmp_y = 0;
  // float average_x = 0;
  // float average_y = 0;
  // float point_num = 0;
  // int track_time = 0;
  // int catapult_shred = 0;
  // float last_average_x = 0;
  // float last_average_y = 0;
  bool catapult_flag = false;

  float now_target_x = 0;
  float now_target_y = 0;
  float now_check_catapult_x = 0;
  float now_check_catapult_y = 0;
  int index = 0;


  int tmp_mission = 1;
  std_msgs::Empty catapult_msg;

  float cos_yaw = 1;
  float sin_yaw = 0;

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
      // mission1: 起飞
      case 1:
        if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
        {
          if(lib_time_record_func(0.5, ros::Time::now()))
          {
            mission_num = 2;
            now_yaw = yaw;
            last_request = ros::Time::now();
          } 
        }
        break;
        
      case 2:
        if (mission_pos_cruise(1, 0, ALTITUDE, 0, err_max))
        {
          if(lib_time_record_func(0.5, ros::Time::now()))
          {
            mission_num = 3;
            now_yaw = yaw;
            last_request = ros::Time::now();
          } 
        }
        break;

      case 3:
        if (mission_pos_cruise(1, 0, ALTITUDE, 0, err_max))
        {
          if(lib_time_record_func(2.0,ros::Time::now()))
          {
            mission_num = 4;
            last_request = ros::Time::now();
          }
          catapult_pub_box1.publish(catapult_msg);
        }
        break;

      case 4:
        if (mission_pos_cruise(1, 0, ALTITUDE, 0, err_max))
        {
          if(lib_time_record_func(2.0,ros::Time::now()))
          {
            mission_num = 5;
            last_request = ros::Time::now();
          }
          catapult_pub_box2.publish(catapult_msg);
        }
        break;

      case 5:
        if (mission_pos_cruise(1, 0, ALTITUDE, 0, err_max))
        {
          if(lib_time_record_func(2.0,ros::Time::now()))
          {
            mission_num = 6;
            last_request = ros::Time::now();
          }
          catapult_pub_box_large.publish(catapult_msg);
        }
        break;

      case 6:
        if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
        {
          if(lib_time_record_func(0.5, ros::Time::now()))
          {
            mission_num = 6;
            now_yaw = yaw;
            last_request = ros::Time::now();
          } 
        }
        break;

        case 10: // 在起飞点降落
        arm_cmd.request.value = false;
        if(mission_pos_cruise(0, 0, -0.02, 0, err_max))
        {
          if (lib_time_record_func(5.0, ros::Time::now()))
          {
            mission_num = -1; // 任务结束
          }
        }
        break;
    }
    
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
    
    if(mission_num == -1) 
    {
      exit(0);
    }
  }
  
  return 0;
}


