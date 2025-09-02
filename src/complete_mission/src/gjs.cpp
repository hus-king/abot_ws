#include <lib_library.h>
#include <gjs.h>

// 全局变量定义
int mission_num = 0;        // 任务标志位
int tmp_marker_id = 1;      // 临时标记ID
int box_number = 0;         // 投放箱子数量
float now_yaw = 0;          // 当前偏航角
float adjust_x = 0;          // 调整X坐标
float adjust_y = 0;          // 调整Y坐标
float offset_door_x = 0; // 穿门时的X坐标偏移
int if_realsence = 0; // 是否使用RealSense相机穿门
float gravity_offset_x = 0.1; // 重心偏移X坐标

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
float check_door_1_y = 0;
float check_door_2_y = 0;
float park_x = 0, park_y = 0; // 停机坪坐标
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

std_msgs::Int32 catapult_ctrl;

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
  std::cout << "check_door_1_y: " << check_door_1_y << std::endl;
  std::cout << "check_door_2_y: " << check_door_2_y << std::endl;
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "err_max_ego: " << err_max_ego << std::endl;
  std::cout << "track_distance: " << track_distance << std::endl;
  std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
  std::cout << "VIEW_ALTITUDE: " << VIEW_ALTITUDE << std::endl;
  std::cout << "LANDING_ALTITUDE: " << LANDING_ALTITUDE << std::endl;
  std::cout << "CATAPULT_ALTITUDE: " << CATAPULT_ALTITUDE << std::endl;
  std::cout << "TANK_ALTITUDE: " << TANK_ALTITUDE << std::endl;
  std::cout << "DOOR_ALTITUDE: " << DOOR_ALTITUDE << std::endl;
  std::cout << "if_debug: " << if_debug << std::endl;
  if(if_debug == 1) cout << "自动offboard" << std::endl;
  else cout << "遥控器offboard" << std::endl;
  std::cout << "=== 投射器参数 ===" << std::endl;
  std::cout << "check_catapult_x: " << check_catapult_x << std::endl;
  std::cout << "check_catapult_y: " << check_catapult_y << std::endl;
  std::cout << "catapult_shred: " << catapult_shred << std::endl;
  std::cout << "camera_height: " << camera_height << std::endl;
  std::cout << "camera_offset_body_x: " << camera_offset_body_x << std::endl;
  std::cout << "landing_gear: " << landing_gear << std::endl;
  std::cout << "gravity_offset_x: " << gravity_offset_x << std::endl;
  std::cout << "target_1: " << target_1 << std::endl;
  std::cout << "target_2: " << target_2 << std::endl;
  std::cout << "target_3: " << target_3 << std::endl;
  std::cout << "=== 穿门参数 ===" << std::endl;
  std::cout << "if_realsence: " << if_realsence << std::endl;
  if(if_realsence == 1) cout << "使用RealSense相机穿门" << std::endl;
  else cout << "使用3D雷达穿门" << std::endl;
  std::cout << "door_adjust_range: " << door_adjust_range << std::endl;
  cout << "range = " << 320 - door_adjust_range << " to " << 320 + door_adjust_range << endl;
  cout << "door_check_num: " << door_check_num << endl;
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
  // ros::Publisher catapult_pub_box1 = nh.advertise<std_msgs::Empty>("servo/back_right/open", 1);
  // ros::Publisher catapult_pub_box2 = nh.advertise<std_msgs::Empty>("servo/back_left/open", 1);
  // ros::Publisher catapult_pub_box3 = nh.advertise<std_msgs::Empty>("servo/front_right/open", 1);

  ros::Publisher catapult_pub = nh.advertise<std_msgs::Int32>("/catapult_num",10);

  ros::Subscriber yolo_ros_box_sub = nh.subscribe<yolov8_ros_msgs::BoundingBoxes>("/object_position", 1, yolo_ros_cb);

  ros::Subscriber yolo_ros_box_H_sub = nh.subscribe<yolov8_ros_msgs::BoundingBoxes>("/object_position_H", 1, yolo_ros_H_cb);
  
  // ros::Subscriber depth_sub = nh.subscribe<camera_processor::PointDepth>("/camera_processor/depth/points", 1, depth_image_cb);
  ros::Subscriber door_lidar_sub = nh.subscribe<geometry_msgs::PointStamped>("/door_center",1,door_lidar_cb); //3D雷达
  door_points_pub = nh.advertise<geometry_msgs::Point>("/door_points", 10);  //深度相机

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  // 参数读取
  for(int i=0;i<10;i++)
  {
    catapult_ctrl.data = 3;
    catapult_pub.publish(catapult_ctrl);
    ros::spinOnce();
    rate.sleep();
  }

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
  nh.param<float>("check_door_1_y", check_door_1_y, 0);
  nh.param<float>("check_door_2_y", check_door_2_y, 0);

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
  nh.param<int>("door_adjust_range", door_adjust_range, 70);

  nh.param<int>("if_realsence", if_realsence, 0);

  nh.param<int>("door_check_num", door_check_num, 5);

  nh.param<float>("landing_gear", landing_gear, 0);
  nh.param<float>("gravity_offset_x", gravity_offset_x, 0.1);

  nh.param<string>("target_1", target_1, "car");
  nh.param<string>("target_2", target_2, "bridge");
  nh.param<string>("target_3", target_3, "pillbox");

  target_array_x.push_back(target1_x);
  target_array_y.push_back(target1_y);
  target_array_x.push_back(target2_x);
  target_array_y.push_back(target2_y);
  target_array_x.push_back(target3_x);
  target_array_y.push_back(target3_y);
  target_array_x.push_back(target4_x);
  target_array_y.push_back(target4_y);


  print_param();
  ros::spinOnce();
  rate.sleep();
  while(local_pos.pose.pose.position.z == 0){
  ros::spinOnce();
  rate.sleep();
  }
  ROS_WARN("%f",init_position_z_take_off);

  
  int choice = 0;
  std::cout << "1 to go on , else to quit" << std::endl;
  ros::spinOnce();
  rate.sleep();
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
 	last_request = ros::Time::now();
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

  int door_num = 0;


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
	      else if(ros::Time::now() - last_request >= ros::Duration(2.0))
        {
          mission_num = 2;
          last_request = ros::Time::now();
          lib_time_init_flag = true;
        }
        break;
        
      case 2: // 确保进入ego_planner时的高度正确，角度正确
        if(ego_check == false){
          now_yaw = yaw;
          while(!current_position_cruise(0, 0, ALTITUDE, now_yaw, err_max))
          {
            mavros_setpoint_pos_pub.publish(setpoint_raw);
            ros::spinOnce();
            rate.sleep();
            cout<<"checking"<<endl;
          }
          now_yaw = calculate_yaw(target_array_x[index], target_array_y[index]);
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
          if (pub_ego_goal(target_array_x[index], target_array_y[index], ALTITUDE, err_max_ego, 0, 0))
          {
            if(lib_time_record_func(0.5, ros::Time::now()))
            {
              if(box_number >= 3)
              {
                if(index == 3)
                {
                  mission_num = 7; // 
                  ego_check = false; // 重置ego_check状态
                  last_request = ros::Time::now();
                }
                else
                {
                  mission_num = 2; // 投放完成直接跳过
                  ego_check = false; // 重置ego_check状态
                  last_request = ros::Time::now();
                  index++;
                }  
              }
              else
              {
                now_yaw = yaw;
                mission_num = 21; // 识别
                ego_check = false; // 重置ego_check状态
                last_request = ros::Time::now();
              }
            } 
          }
        }
        break;

      case 21: // 悬停
        if(mission_pos_cruise(target_array_x[index], target_array_y[index], ALTITUDE, now_yaw, err_max))
        {
          if(lib_time_record_func(0.5, ros::Time::now()))
          {
            mission_num = 3;
            last_request = ros::Time::now();
          }
        }
        else if(ros::Time::now() - last_request >= ros::Duration(3.0))
        {
          mission_num = 3;
          last_request = ros::Time::now();
          lib_time_init_flag = true;
        }
        break;

      case 3: // 上升到视觉识别高度
        mission_pos_cruise(target_array_x[index], target_array_y[index], VIEW_ALTITUDE, now_yaw, err_max);
        if(start_checking == false && local_pos.pose.pose.position.z > (VIEW_ALTITUDE - 0.2))
        {
          start_checking = true; // 上升至VIEW_ALTITUDE高度开始识别
          ROS_WARN("开始识别目标，请稍等...");
        }
        if(found) // 确定找到
        {
          mission_num = 4; // 投放
          now_target_x = box_target_x;
          now_target_y = box_target_y;
          start_checking = false;
          box_number++;
          last_request = ros::Time::now();
        }
        else if(found_false) // 确定找到不需要投放的靶标
        {
          mission_num = 2; // 去下一个点
          start_checking = false;
          found_false = false; // 重置found_false状态
          index++; // 走完一个点
          last_request = ros::Time::now();
        }
        else if(ros::Time::now() - last_request >= ros::Duration(3.0))
        {
          mission_num = 31; // 没找到开始画正方形找
          last_request = ros::Time::now();
        }
        break;

      // 正方形搜索模式
      case 31: // 左下
        mission_pos_cruise(target_array_x[index] - track_distance, target_array_y[index] + track_distance, VIEW_ALTITUDE, now_yaw, err_max);
        if(start_checking == false && local_pos.pose.pose.position.z > (VIEW_ALTITUDE - 0.2))
        {
          start_checking = true; // 上升至VIEW_ALTITUDE高度开始识别
          ROS_WARN("开始识别目标，请稍等...");
        }
        if(found)
        {
          mission_num = 4;
          now_target_x = box_target_x;
          now_target_y = box_target_y;
          start_checking = false;
          box_number++;
          last_request = ros::Time::now();
        }
        else if(found_false)
        {
          mission_num = 2;
          now_yaw = yaw;
          start_checking = false;
          found_false = false; // 重置found_false状态
          index++; // 走完一个点
          last_request = ros::Time::now();
        }
        else if(ros::Time::now() - last_request >= ros::Duration(1.0))
        {
          mission_num = 32;
          last_request = ros::Time::now();
        }
        break;
      

      case 32: // 左上
        mission_pos_cruise(target_array_x[index] + track_distance, target_array_y[index] + track_distance, VIEW_ALTITUDE, now_yaw, err_max);
        if(start_checking == false && local_pos.pose.pose.position.z > (VIEW_ALTITUDE - 0.2))
        {
          start_checking = true; // 上升至VIEW_ALTITUDE高度开始识别
          ROS_WARN("开始识别目标，请稍等...");
        }
        if(found)
        {
          mission_num = 4;
          now_target_x = box_target_x;
          now_target_y = box_target_y;
          start_checking = false;
          box_number++;
          last_request = ros::Time::now();
        }
        else if(found_false)
        {
          mission_num = 2;
          now_yaw = yaw;
          start_checking = false;
          found_false = false; // 重置found_false状态
          index++; // 走完一个点
          last_request = ros::Time::now();
        }
        else if(ros::Time::now() - last_request >= ros::Duration(1.0))
        {
          mission_num = 33;
          last_request = ros::Time::now();
        }
        break;
      
      case 33: // 右上
        mission_pos_cruise(target_array_x[index] + track_distance, target_array_y[index] - track_distance, VIEW_ALTITUDE, now_yaw, err_max);
        if(start_checking == false && local_pos.pose.pose.position.z > (VIEW_ALTITUDE - 0.2))
        {
          start_checking = true; // 上升至VIEW_ALTITUDE高度开始识别
          ROS_WARN("开始识别目标，请稍等...");
        }
        if(found)
        {
          mission_num = 4; // 投放
          now_target_x = box_target_x;
          now_target_y = box_target_y;
          start_checking = false;
          box_number++;
          last_request = ros::Time::now();
        }
        else if(found_false)
        {
          mission_num = 2; // 跳过
          start_checking = false;
          found_false = false; // 重置found_false状态
          index++; // 走完一个点
          last_request = ros::Time::now();
        }
        else if(ros::Time::now() - last_request >= ros::Duration(1.0))
        {
          mission_num = 34;
          last_request = ros::Time::now();
        }
        break;
      
      case 34: // 右下
        mission_pos_cruise(target_array_x[index] - track_distance, target_array_y[index] - track_distance, VIEW_ALTITUDE, now_yaw, err_max);
                if(start_checking == false && local_pos.pose.pose.position.z > (VIEW_ALTITUDE - 0.2))
        {
          start_checking = true; // 上升至VIEW_ALTITUDE高度开始识别
          ROS_WARN("开始识别目标，请稍等...");
        }
        if(found)
        {
          mission_num = 4;
          now_target_x = box_target_x;
          now_target_y = box_target_y;
          start_checking = false;
          box_number++;
          last_request = ros::Time::now();
        }
        else if(found_false)
        {
          mission_num = 2;
          now_yaw = yaw;
          start_checking = false;
          found_false = false; // 重置found_false状态
          index++; // 走完一个点
          last_request = ros::Time::now();
        }
        else if(ros::Time::now() - last_request >= ros::Duration(1.0))
        {
          if(index < 3)
          {
            mission_num = 2;
            start_checking = false;
            index++; // 走完一个点
            last_request = ros::Time::now();
          }
          else
          {
            mission_num = 7;
            start_checking = false; // 重置start_checking状态
            last_request = ros::Time::now();
          }
        }
        break;
      
      case 4: // 投放任务
        cout << "bounding_box.Class: " << cb.Class << endl;
        cout << "bounding_box.probability: " << cb.probability << endl;
        cout << "box_target_x = " << box_target_x << ", box_target_y = " << box_target_y << endl;
        found = false; // 重置found状态
      // box_target_x = local_pos.pose.pose.position.x + camera_offset_x * cos_yaw - camera_offset_y * sin_yaw;
			// box_target_y = local_pos.pose.pose.position.y + camera_offset_x * sin_yaw + camera_offset_y * cos_yaw;
        cos_yaw = cos(now_yaw);
				sin_yaw = sin(now_yaw);
        if(box_number == 1)
        {
          now_check_catapult_x = check_catapult_y * sin_yaw + gravity_offset_x * cos_yaw;
          now_check_catapult_y = - check_catapult_y * cos_yaw + gravity_offset_x * sin_yaw;
        }
        else if(box_number == 2)
        {
          now_check_catapult_x = - check_catapult_y * sin_yaw + gravity_offset_x * cos_yaw;
          now_check_catapult_y = check_catapult_y * cos_yaw + gravity_offset_x * sin_yaw;
        }
        else if(box_number == 3)
        {
          now_check_catapult_x = check_catapult_x * cos_yaw;
          now_check_catapult_y = check_catapult_x * sin_yaw;
        }
        
        if(mission_pos_cruise(now_target_x + now_check_catapult_x, now_target_y + now_check_catapult_y, CATAPULT_ALTITUDE, now_yaw, err_max))
        {
          // 当前位置修正后投放
          if(lib_time_record_func(3.0, ros::Time::now()))
          {
            if(box_number == 1)
            {
              catapult_ctrl.data = 0;
              catapult_pub.publish(catapult_ctrl);
              // catapult_pub_box1.publish(catapult_msg);
            }
            else if(box_number == 2)
            {
              catapult_ctrl.data = 1;
              catapult_pub.publish(catapult_ctrl);
              // catapult_pub_box2.publish(catapult_msg);
            }
            else if(box_number == 3)
            {
              catapult_ctrl.data = 2;
              catapult_pub.publish(catapult_ctrl);
              // catapult_pub_box3.publish(catapult_msg);
            }
            mission_num = 41;
            last_request = ros::Time::now();
          }
        }
        else if(ros::Time::now() - last_request >= ros::Duration(6.0))
        {
          if(box_number == 1)
          {
            catapult_ctrl.data = 0;
            catapult_pub.publish(catapult_ctrl);
            // catapult_pub_box1.publish(catapult_msg);
          }
          else if(box_number == 2)
          {
            catapult_ctrl.data = 1;
            catapult_pub.publish(catapult_ctrl);
            // catapult_pub_box2.publish(catapult_msg);
          }
          else if(box_number == 3)
          {
            catapult_ctrl.data = 2;
            catapult_pub.publish(catapult_ctrl);
            // catapult_pub_box3.publish(catapult_msg);
          }
          mission_num = 41;
          last_request = ros::Time::now();
          lib_time_init_flag = true;
        }
        break;

      case 41:
        if(mission_pos_cruise(now_target_x + now_check_catapult_x, now_target_y + now_check_catapult_y, CATAPULT_ALTITUDE, now_yaw, err_max))
        {
          if(lib_time_record_func(1.0, ros::Time::now())){
            if(index < 3)
            {
              mission_num = 2;
              now_yaw = yaw;
              last_request = ros::Time::now();
              index++;
            }
            else
            {
              mission_num = 7;
              last_request = ros::Time::now();
            }
          }
        }
        else if(ros::Time::now() - last_request >= ros::Duration(4.0))
        {
          if(index < 3)
          {
            mission_num = 2;
            now_yaw = yaw;
            last_request = ros::Time::now();
            index++;
          }
          else
          {
            mission_num = 7;
            last_request = ros::Time::now();
          }
          lib_time_init_flag = true;
        }
        break;

      // 穿门任务
      case 7: // 门前拐角处
        if(ego_check == false){
          catapult_ctrl.data = 2;
          catapult_pub.publish(catapult_ctrl);
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
              mission_num = 701;
              ego_check = false; // 重置ego_check状态
              last_request = ros::Time::now();
              now_yaw = yaw;
            }
          }
        }
        break;
     
      case 701://旋转角度
        if(current_position_cruise(0, 0, DOOR_ALTITUDE, 0, err_max)){
          if(lib_time_record_func(0.5, ros::Time::now())){
            mission_num = 702;
            last_request = ros::Time::now();
          }
        }
        break;

      case 702://中点
        if(mission_pos_cruise((target6_x + target8_x)/2, target6_y, DOOR_ALTITUDE, 0, err_max))
        {
          if (lib_time_record_func(0.3, ros::Time::now()))
          {
            mission_num =71;
            last_request = ros::Time::now();
          }
        }
        else if(ros::Time::now() - last_request >= ros::Duration(3.0))
        {
          mission_num =71;
          last_request = ros::Time::now();
          lib_time_init_flag = true;
        }
	     break;

      case 71: // 门前
        if(mission_pos_cruise(target6_x, target6_y, DOOR_ALTITUDE, 0, err_max))
        {
          if (lib_time_record_func(0.3, ros::Time::now()))
          {
            mission_num =715;
            target6_y = check_door_1_y; // 调整y坐标
            ego_check = false; // 重置ego_check状态
            last_request = ros::Time::now();
            now_yaw = yaw;
          }
        }
        else if(ros::Time::now() - last_request >= ros::Duration(3.0))
        {
          mission_num =71;
          last_request = ros::Time::now();
          lib_time_init_flag = true;
        }
        break;

      case 715: // 门前调整
        if(current_position_cruise(0, 0, DOOR_ALTITUDE, -1.57, err_max)){
          if(lib_time_record_func(0.5, ros::Time::now())){
            mission_num = 72;
            last_request = ros::Time::now();
          }
        }
        break;
      
      case 72: //向前靠近门
        if (mission_pos_cruise(target6_x, target6_y, DOOR_ALTITUDE, -1.57, err_max))
        {
          if(lib_time_record_func(0.5, ros::Time::now()))
          {
            if(if_realsence == 1){
              adjust_x = target6_x;
              adjust_y = target6_y;
              mission_num = 8; // 用深度相机调整第一个门
              last_request = ros::Time::now();
            }
            mission_num = 73; // 开始雷达穿门
            last_request = ros::Time::now();
          }   
        }
        else if(ros::Time::now() - last_request >= ros::Duration(4.0))
        {
          if(if_realsence == 1){
            adjust_x = target6_x;
            adjust_y = target6_y;
            mission_num = 8; // 用深度相机调整第一个门
            last_request = ros::Time::now();
          }
          mission_num = 73; // 开始雷达穿门
          last_request = ros::Time::now(); 
          lib_time_init_flag = true;
        }

        break;

      case 73: //雷达检测
      if(abs(local_pos.pose.pose.position.y - target6_y) < 0.1) start_check_door_flag = true;
        mission_pos_cruise(target6_x,target6_y ,DOOR_ALTITUDE,-1.57,err_max);
        if(finish_check_door_flag)
        {
          door_num ++;
          finish_check_door_flag = false;
          start_check_door_flag = false;
          offset_door_x = target6_x;
          check_num = 0;
          sum_door_x = 0;
          sum_door_y = 0;
          sum_door_z = 0;
          avg_door_x = 0;
          avg_door_y = 0;
          avg_door_z = 0;
          mission_num = 74;
          last_request = ros::Time::now();
        }
        break;

      case 74: //横向调整
        while(abs(local_pos.pose.pose.position.x - door_x) > 0.08){
	      ROS_WARN("door_x: %f",door_x);
          if(lib_time_record_func(0.1, ros::Time::now())){
            if(door_x > local_pos.pose.pose.position.x){
              offset_door_x += 0.01;
	            ROS_WARN("adjust_left");
            }
            else{
              offset_door_x -= 0.01;
	            ROS_WARN("adjust_right");
            }
            mission_pos_cruise(offset_door_x,target6_y,DOOR_ALTITUDE,-1.57,err_max);
          }
          mavros_setpoint_pos_pub.publish(setpoint_raw);
          ros::spinOnce();
          rate.sleep();
        }
        mission_num = 75;
        last_request = ros::Time::now();
        break;
            
      case 75: //直接向前
        if(mission_pos_cruise(door_x,door_y - 0.7 , DOOR_ALTITUDE, -1.57, err_max))
        {
          if(door_num >= 2)
          {
            now_yaw = yaw;
            mission_num = 10; // 降落任务
            last_request = ros::Time::now();
          }
          else
          {
            start_check_door_flag = true;
            mission_num = 73;
            target6_y = check_door_2_y;
            last_request = ros::Time::now();
          }
        }
        else if(ros::Time::now() - last_request >= ros::Duration(3.0))
        {
          if(door_num >= 2)
          {
            now_yaw = yaw;
            mission_num = 10; // 降落任务
            last_request = ros::Time::now();
          }
          else
          {
            start_check_door_flag = true;
            mission_num = 73;
            target6_y = check_door_2_y;
            last_request = ros::Time::now();
          }
        }
        break;

      case 10: // 降落点悬停
        if(mission_pos_cruise(target7_x, target7_y, DOOR_ALTITUDE, -1.57, err_max))
        {
          if(lib_time_record_func(3.0, ros::Time::now()))
          {
            mission_num = 100;
            now_yaw = yaw;
            last_request = ros::Time::now();
          } 
        }
        else if(ros::Time::now() - last_request >= ros::Duration(10.0))
        {
          mission_num = 100;
          now_yaw = yaw;
          last_request = ros::Time::now();
          lib_time_init_flag = true;
        }
        break;

      
      case 100: // 在降落点降落
        arm_cmd.request.value = false;
        if(mission_pos_cruise(target7_x, target7_y, LANDING_ALTITUDE, now_yaw, err_max))
        {
          if(lib_time_record_func(2.0, ros::Time::now()))
          {
            mission_num = -1; // 任务结束
          }
        }
        else if(ros::Time::now() - last_request >= ros::Duration(5.0)){
          mission_num = -1; // 任务结束
          lib_time_init_flag = true;
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


