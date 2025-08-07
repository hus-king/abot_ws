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
  // ros::Publisher catapult_pub_box1 = nh.advertise<std_msgs::Empty>("servo/front_left/open", 1);
  // ros::Publisher catapult_pub_box2 = nh.advertise<std_msgs::Empty>("servo/front_right/open", 1);
  // ros::Publisher catapult_pub_box_large = nh.advertise<std_msgs::Empty>("servo/all/open", 1);
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


  // // 等待连接到飞控
  // while (ros::ok())
  // {
  //   depth_flag = true; // 默认有深度信息
  //   int now_door = judge_door();
  //   cout << "now_door: " << now_door << endl;
  //   ROS_INFO("门的方向: %d,1:left 2:right", now_door);
    // ros::spinOnce();
    // rate.sleep();
  // }
  while(ros::ok){
    ros::spinOnce();
    rate.sleep();
    depth_flag = true; // 默认有深度信息
    find_line();
    print_line();
  }
  return 0;
}


