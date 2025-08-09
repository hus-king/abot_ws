#include <lib_library.h>
#include <gjs.h>

// 全局变量定义
int mission_num = 0;        // 任务标志位
int tmp_marker_id = 1;      // 临时标记ID
int box_number = 0;         // 投放箱子数量
float now_yaw = 0;          // 当前偏航角


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
bool ego_check = false; //是否完成进入ego前参数自检
float if_debug = 0; // 是否开启调试模式

float err_max = 0;              // 最大误差
float err_max_ego = 0;          // ego规划器最大误差

int door_num = 0; // 记录穿门次数


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
  
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "err_max_ego: " << err_max_ego << std::endl;
  std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
  
  std::cout << "=== 穿门参数 ===" << std::endl;
  std::cout << "line_shred: " << line_shred << std::endl;
  std::cout << "door_adjust_range: " << door_adjust_range << std::endl;
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
  door_points_pub = nh.advertise<geometry_msgs::Point>("/door_points", 10);

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
  ros::Publisher catapult_pub_box1 = nh.advertise<std_msgs::Empty>("servo/front_left/open", 1);
  ros::Publisher catapult_pub_box2 = nh.advertise<std_msgs::Empty>("servo/front_right/open", 1);
  ros::Publisher catapult_pub_box3 = nh.advertise<std_msgs::Empty>("servo/back_right/open", 1);
  ros::Subscriber door_lidar_sub = nh.subscribe<geometry_msgs::PointStamped>("/door_center", 1, door_lidar_cb);

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

  nh.param<float>("err_max", err_max, 0.1);
  nh.param<float>("err_max_ego", err_max_ego, 0.3);

  nh.param<float>("if_debug", if_debug, 0);
  nh.param<float>("line_shred", line_shred, 0.1);

  nh.param<int>("door_adjust_range", door_adjust_range, 70);



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


  float adjust_x = 0;
  float adjust_y = 0;


  int tmp_mission = 1;
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
      // mission1: 起飞
      case 1:
        if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
        {
          if(lib_time_record_func(0.5, ros::Time::now()))
          {
            mission_num = 70;
            last_request = ros::Time::now();
          } 
        }
        break;
        
      case 70: // 门前拐角处
        if(ego_check == false){
          now_yaw = calculate_yaw(target8_x, target8_y);
          while(!current_position_cruise(0, 0, ALTITUDE, now_yaw, err_max))
          {
            mavros_setpoint_pos_pub.publish(setpoint_raw);
            ros::spinOnce();
            rate.sleep();
            cout<<"checking"<<endl;
          }
          ego_check = true;
        }
        else{
          if(pub_ego_goal(target8_x, target8_y, ALTITUDE, err_max_ego, 0, 0))
          {
            if (lib_time_record_func(0.3, ros::Time::now()))
            {
              mission_num = 7;
              ego_check = false; // 重置ego_check状态
              last_request = ros::Time::now();
              now_yaw = yaw;
            }
          }
        }
        break;

      case 7: // 门前
        if(ego_check == false){
          now_yaw = calculate_yaw(target6_x, target6_y);//
          while(!current_position_cruise(0, 0, ALTITUDE, now_yaw, err_max))
          {
            mavros_setpoint_pos_pub.publish(setpoint_raw);
            ros::spinOnce();
            rate.sleep();
            cout<<"checking"<<endl;
          }
          ego_check = true;
        }
        if(pub_ego_goal(target6_x, target6_y, ALTITUDE, err_max, 0, 1))
        {
          if (lib_time_record_func(0.3, ros::Time::now()))
          {
            mission_num = 700;
            target6_y = target6_y - 0.8;
            ego_check = false; // 重置ego_check状态
            last_request = ros::Time::now();
            now_yaw = yaw;
          }
        }
        break;
      case 700: // 向前靠近门
        if (mission_pos_cruise(target6_x, target6_y, ALTITUDE, -1.57, err_max))
        {
          if(lib_time_record_func(0.5, ros::Time::now()))
          {
            start_check_door_flag = true;
            mission_num = 701;
            last_request = ros::Time::now();
          } 
        }
        break;

        break;

      case 701:
        mission_pos_cruise(target6_x,target6_y ,ALTITUDE,-1.57,err_max);
        if(finish_check_door_flag)
        {
          door_num ++;
          finish_check_door_flag = false;
          mission_num = 702;
          last_request = ros::Time::now();
        }
        break;
      
      case 702:
        if(mission_pos_cruise(door_x,target6_y,ALTITUDE,-1.57,err_max))
        {
          if(lib_time_record_func(0.5, ros::Time::now()))
          {
            mission_num = 703;
            last_request = ros::Time::now();
          }
        }
        break;

        case 703:
        if(mission_pos_cruise(door_x,door_y,ALTITUDE,-1.57,err_max))
        {
          if(lib_time_record_func(0.5, ros::Time::now()))
          {
            mission_num = 704;
            last_request = ros::Time::now();
          }
        }
        break;

        case 704:
        if(mission_pos_cruise(door_x,door_y - 0.5 ,ALTITUDE,-1.57,err_max))
        {
          if(lib_time_record_func(0.5, ros::Time::now()))
          {
            if(door_num >= 2)
            {
              mission_num = 6;
              last_request = ros::Time::now();
            }
            else
            {
              start_check_door_flag = true;
              mission_num = 701;
              target6_y = door_y - 0.5;
              last_request = ros::Time::now();
            }
          }
        }
        break;
      case 6: // 降落
        arm_cmd.request.value = false;
        if(mission_pos_cruise(door_x, door_y - 0.5, -0.05, 0, err_max))
        {
          ROS_INFO("Vehicle disarmed");
          mission_num = -1;
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


