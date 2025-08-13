// 简化版本：只保留起飞后在1米悬停的功能
#include <lib_library.h>
#include <hover.h>

int mission_num = 0;   //mission_flag
float current_yaw = 0.0;  // 当前yaw角度
float yaw_rate = 0;     // yaw旋转速度 (弧度/秒)

void print_param()
{
  std::cout << "悬停高度: " << ALTITUDE << " 米" << std::endl;
  std::cout << "旋转速度: " << yaw_rate << " 弧度/秒" << std::endl;
}

int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "hover_simple");

  // 创建节点句柄
  ros::NodeHandle nh;

  // 创建一个Subscriber订阅者，订阅名为/mavros/state的topic，注册回调函数state_cb
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

  // 创建一个Subscriber订阅者，订阅名为/mavros/local_position/odom的topic，注册回调函数local_pos_cb
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

  // 创建一个服务客户端，连接名为/mavros/cmd/arming的服务，用于请求无人机解锁
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

  // 创建一个服务客户端，连接名为/mavros/set_mode的服务，用于请求无人机进入offboard模式
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  print_param();

  int choice = 0;
  std::cout << "输入1开始起飞悬停，其他任意键退出: ";
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
  std::cout << "准备起飞..." << std::endl;

  // 定义客户端变量，设置为offboard模式
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // 定义客户端变量，请求无人机解锁
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  // 记录当前时间，并赋值给变量last_request
  ros::Time last_request = ros::Time::now();
  ros::Time last_yaw_update = ros::Time::now();

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
    
    // 当无人机到达起飞点高度后，开始悬停和旋转
    if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.1)
    {
      if (mission_num == 0)
      {
        mission_num = 1;
        std::cout << "已达到目标高度，开始悬停和旋转..." << std::endl;
        last_request = ros::Time::now();
        last_yaw_update = ros::Time::now(); // 重置yaw更新时间
      }
      
      // 只有到达目标高度后才开始更新yaw角度 - 缓慢旋转
      double dt = (ros::Time::now() - last_yaw_update).toSec();
      current_yaw += yaw_rate * dt;
      
      // 限制yaw角度在-π到π之间
      if (current_yaw > M_PI)
        current_yaw -= 2 * M_PI;
      else if (current_yaw < -M_PI)
        current_yaw += 2 * M_PI;
      
      last_yaw_update = ros::Time::now();
    }
    
    // 持续发布位置指令
    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = 0;
    setpoint_raw.position.y = 0;
    setpoint_raw.position.z = ALTITUDE;
    setpoint_raw.yaw = current_yaw;  // 使用当前yaw角度（到达高度前为0，到达后开始旋转）
    
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    
    // 显示当前状态
    if (mission_num == 1 && (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      std::cout << "悬停中 - 当前位置: (" 
                << local_pos.pose.pose.position.x << ", " 
                << local_pos.pose.pose.position.y << ", " 
                << local_pos.pose.pose.position.z << ") "
                << "目标高度: " << ALTITUDE << "米 "
                << "当前Yaw: " << current_yaw * 180.0 / M_PI << "度" << std::endl;
      last_request = ros::Time::now();
    }
    else if (mission_num == 0 && (ros::Time::now() - last_request > ros::Duration(3.0)))
    {
      std::cout << "起飞中 - 当前高度: " << local_pos.pose.pose.position.z 
                << "米，目标高度: " << ALTITUDE << "米" << std::endl;
      last_request = ros::Time::now();
    }
    
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}


