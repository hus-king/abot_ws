#include <lib_library.h>
#include <gjs.h>


void print_param()
{
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

  ros::Subscriber yolo_ros_box_sub = nh.subscribe<yolov8_ros_msgs::BoundingBoxes>("/object_position", 1, yolo_ros_cb);
  
  ros::Subscriber depth_sub = nh.subscribe<camera_processor::PointDepth>("/camera_processor/depth/points", 1, depth_image_cb);

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  // 参数读取

  nh.param<float>("door_step", door_step, 2);
  nh.param<float>("door_shred", door_shred, 1);


  print_param();

  
  int choice = 0;
  std::cout << "1 to go on , else to quit" << std::endl;
  std::cin >> choice;
  if (choice != 1)
    return 0;
  ros::spinOnce();
  rate.sleep();
  while(ros::ok){
    ros::spinOnce();
    rate.sleep();
    depth_flag = true; // 默认有深度信息
    find_line();
    print_line();
    find_door();
  }
  return 0;
}


