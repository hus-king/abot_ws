// 说明：舵机方向根据实际调整，这里用的占空比0%表示闭合，100%表示全部打开
// gjs-version1
// collision_avoidance and catapult
#include <lib_library.h>
#include <gjs.h>


int mission_num = 0;   //mission_flag
int tmp_marker_id = 1;
int box_number = 0;
float square_yaw = 0;
// the four picture
vector<float> target_array_x;
vector<float> target_array_y;

float target1_x = 0, target1_y = 0;
float target2_x = 0, target2_y = 0;
float target3_x = 0, target3_y = 0;
float target4_x = 0, target4_y = 0;
float target5_x = 0, target5_y = 0;
float target6_x = 0, target6_y = 0;
//只投放car，bridge，跳过pillbox，tank，tent


float park_x = 0,park_y = 0;

float track_distance;

float temp_x = 0 , temp_y = 0 ;

float err_max = 0;
float err_max_ego = 0;

float fly_height = 0.75;



float check_catapult_x;
float check_catapult_y;
float catapult_shred;


void print_param()
{
  std::cout << "target1 : ( " << target1_x << ", " << target1_y << " )" << std::endl;
  std::cout << "target2 : ( " << target2_x << ", " << target2_y << " )" << std::endl;
  std::cout << "target3 : ( " << target3_x << ", " << target3_y << " )" << std::endl;
  std::cout << "target4 : ( " << target4_x << ", " << target4_y << " )" << std::endl;
  std::cout << "target5 : ( " << target5_x << ", " << target5_y << " )" << std::endl;
  std::cout << "target6 : ( " << target6_x << ", " << target6_y << " )" << std::endl;
  
  std::cout << "park : ( " << park_x << ", " << park_y << " )" << std::endl;

  
  std::cout << "err_max : " << err_max << std::endl;
  std::cout << "err_max_ego : " << err_max_ego << std::endl;
  std::cout << "track_distance : " << track_distance << std::endl;
  std::cout << "ALTITUDE : " << ALTITUDE << std::endl;
  std::cout << "VIEW_ALTITUDE : "<< VIEW_ALTITUDE << std::endl;

  std::cout << "check_catapult_x : " << check_catapult_x << std::endl;
  std::cout << "check_catapult_y : " << check_catapult_y << std::endl;
  std::cout << "catapult_shred : " << catapult_shred << std::endl;
  std::cout << "camera_height : " << camera_height << std::endl;

}


int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "gjs");

  // 创建节点句柄
  ros::NodeHandle nh;

  // 订阅ego_planner规划出来的结果
  ros::Subscriber ego_sub = nh.subscribe("/position_cmd", 100, ego_sub_cb);

  // 发布ego_planner目标
  planner_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal", 100);

  ros::Subscriber rec_traj_sub = nh.subscribe("/rec_traj", 100, rec_traj_cb);

  finish_ego_pub = nh.advertise<std_msgs::Bool>("/finish_ego", 1);

  // 创建一个Subscriber订阅者，订阅名为/mavros/state的topic，注册回调函数state_cb
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

  // 创建一个Subscriber订阅者，订阅名为/mavros/local_position/odom的topic，注册回调函数local_pos_cb
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  // 订阅move_base规划出来的速度，用于避障
  // ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmd_to_px4);

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
  ros::Subscriber yolo_ros_box_sub = nh.subscribe<yolov8_ros_msgs::BoundingBoxes>("/object_position", 1, yolo_ros_cb);

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

  nh.param<float>("track_distance", track_distance, 0); 
  nh.param<float>("err_max", err_max, 0);
  nh.param<float>("err_max_ego", err_max_ego, 0.3);
  nh.param<float>("fly_height", fly_height, 0);

  nh.param<float>("check_catapult_x", check_catapult_x, 0);
  nh.param<float>("check_catapult_y", check_catapult_y, 0);
  
  nh.param<float>("catapult_shred", catapult_shred, 0);

  nh.param<double>("sigma_a", sigma_a, 0.1);
  nh.param<double>("tank_time", tank_time, 0.0);
  nh.param<double>("tank_shred", tank_shred, 0.0);

  nh.param<float>("camera_height", camera_height, 0);

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
      //mission1: take_off
      case 1:
        if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
        {
          if(lib_time_record_func(0.5,ros::Time::now()))
          {
            mission_num = 2;
            last_request = ros::Time::now();
          } 
        }
       break;
      
      case 2:
        if (pub_ego_goal(target_array_x[index], target_array_y[index], ALTITUDE, 0.2 , 0))
        {
          if(lib_time_record_func(0.5,ros::Time::now()))
          {
            if(box_number >=2)
            {
              if(index == 3)
              {
                mission_num = 5;
                last_request = ros::Time::now();
              }
              else
              {
                mission_num = 2;
                last_request = ros::Time::now();
                index++;
              }  
            }
            else
            {
              square_yaw = square_yaw_cb;
              cout<<"square_yaw = "<<square_yaw<<endl;
              mission_num = 21;
              last_request = ros::Time::now();
            }
          } 
        }
        break;

      case 21:
        if(mission_pos_cruise(target_array_x[index], target_array_y[index], ALTITUDE, square_yaw, err_max))
        {
          if(lib_time_record_func(0.5,ros::Time::now()))
          {
            mission_num = 3;
            last_request = ros::Time::now();
          }
        }
        else if(ros::Time::now() - last_request >= ros::Duration(3.0))
        {
          mission_num = 3;
          last_request = ros::Time::now();
        }
        break;

      case 3:
        mission_pos_cruise(target_array_x[index], target_array_y[index], VIEW_ALTITUDE, square_yaw, err_max);
        if(local_pos.pose.pose.position.z > (VIEW_ALTITUDE - 0.1))
        {
          start_checking = true;
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
          start_checking = false;
          found_false = false; // 重置found_false状态
          index ++;//走完一个点
          last_request = ros::Time::now();
        }
        else if(ros::Time::now() - last_request >= ros::Duration(3.0))
        {
          mission_num = 31;
          last_request = ros::Time::now();
        }
        break;

      //左下
      case 31:
        mission_pos_cruise(target_array_x[index] - track_distance, target_array_y[index] + track_distance, VIEW_ALTITUDE, square_yaw, err_max);
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
          start_checking = false;
          found_false = false; // 重置found_false状态
          index ++;//走完一个点
          last_request = ros::Time::now();
        }
        else if(ros::Time::now() - last_request >= ros::Duration(1.0))
        {
          mission_num = 32;
          last_request = ros::Time::now();
        }
        break;
      
      //左上
      case 32:
        mission_pos_cruise(target_array_x[index] + track_distance, target_array_y[index] + track_distance, VIEW_ALTITUDE, square_yaw, err_max);
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
          start_checking = false;
          found_false = false; // 重置found_false状态
          index ++;//走完一个点
          last_request = ros::Time::now();
        }
        else if(ros::Time::now() - last_request >= ros::Duration(1.0))
        {
          mission_num = 33;
          last_request = ros::Time::now();
        }
        break;
      
      //右上
      case 33:
        mission_pos_cruise(target_array_x[index] + track_distance, target_array_y[index] - track_distance, VIEW_ALTITUDE, square_yaw, err_max);
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
          start_checking = false;
          found_false = false; // 重置found_false状态
          index ++;//走完一个点
          last_request = ros::Time::now();
        }
        else if(ros::Time::now() - last_request >= ros::Duration(1.0))
        {
          mission_num = 34;
          last_request = ros::Time::now();
        }
        break;
      
      //右下
      case 34:
        mission_pos_cruise(target_array_x[index] - track_distance, target_array_y[index] - track_distance, VIEW_ALTITUDE, square_yaw, err_max);
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
          start_checking = false;
          found_false = false; // 重置found_false状态
          index ++;//走完一个点
          last_request = ros::Time::now();
        }
        else if(ros::Time::now() - last_request >= ros::Duration(1.0))
        {
          if(index < 3)
          {
            mission_num = 2;
            index ++;//走完一个点
            last_request = ros::Time::now();
          }
          else
          {
            mission_num = 5;
            last_request = ros::Time::now();
          }
        }
        break;
      
      case 4://投放
        cout<<"bounding_box.Class"<<cb.Class<<endl;
        cout<<"bounding_box.probability"<<cb.probability<<endl;
        cout << "box_target_x = "<<box_target_x << ", box_target_y = " << box_target_y << endl;
        found = false; // 重置found状态
        if(box_number == 1)
        {
          now_check_catapult_x = 0;
          now_check_catapult_y = -check_catapult_y;
        }
        else if(box_number == 2)
        {
          now_check_catapult_x = 0;
          now_check_catapult_y = check_catapult_y;
        }
        if(mission_pos_cruise(now_target_x + now_check_catapult_x, now_target_y + now_check_catapult_y, 0.05, square_yaw, err_max))
        {
          if (lib_time_record_func(1.0, ros::Time::now())){
            if(index < 3)
            {
              mission_num = 2;
              last_request = ros::Time::now();
              index ++;
            }
            else
            {
              mission_num = 5;
              last_request = ros::Time::now();
            }
          }
          if(box_number == 1)
          {
            catapult_pub_box1.publish(catapult_msg);
          }
          else if(box_number == 2)
          {
            catapult_pub_box2.publish(catapult_msg);
          }
        }
        break;
      
      //动态靶
      case 5:
        if(pub_ego_goal(target5_x, target5_y, ALTITUDE, 0.2, 0))
        {
          if (lib_time_record_func(0.5, ros::Time::now())){
            mission_num = 61; //61-only_guess,66-kalman-method
            //kalman_start_flag = true;
            //now_target_x = target5_x, now_target_y = target5_y ;
            last_request = ros::Time::now();
          }
        }
        break;

      //视觉任务
      //61:识别第一帧且距离接近于正中心的值
      case 61:
        mission_pos_cruise(target5_x, target5_y, ALTITUDE, 0, err_max);
        if(found)
        {
          mission_num = 62;
          now_target_x = box_target_x;
          now_target_y = box_target_y;
          last_request = ros::Time::now();
        }
        break;
      
      case 62:
        if(mission_pos_cruise(now_target_x + check_catapult_x, now_target_y, ALTITUDE, 0, err_max))
        {
          if(!found)
          {
            catapult_flag = true;
          }
          if(catapult_flag && found)
          {
            if(abs((box_target_x - now_target_x)*(box_target_x - now_target_x) + (box_target_y - now_target_y)*(box_target_y - now_target_y)) <= catapult_shred)
            {
              catapult_pub_box_large.publish(catapult_msg);
              mission_num = 63;
              last_request = ros::Time::now();
              catapult_flag = false;
            }
          }
        }
        break;

      case 63:
       if(mission_pos_cruise(now_target_x + check_catapult_x, now_target_y, ALTITUDE, 0, err_max))
        { 
          catapult_pub_box_large.publish(catapult_msg);
          if (lib_time_record_func(1.0, ros::Time::now())){
            mission_num = 7;
            last_request = ros::Time::now();
          }
        }
        break;
      
      //kalman-version
      case 66:
        mission_pos_cruise(now_target_x + check_catapult_x,now_target_y,TANK_ALTITUDE,0,err_max);
        if(step == 1)
        {
          ;
        }
        else if(step == 2)
        {
          now_target_x = kalman_predict_x;
          now_target_y = kalman_predict_y;
        }
        if(Kalman_prediction())
        {
          mission_num = 67;
          catapult_pub_box_large.publish(catapult_msg);
          last_request = ros::Time::now();
        }
        if(ros::Time::now() - last_request >= ros::Duration(60))
        {
          mission_num = 67;
          last_request = ros::Time::now();
        }
        break;
      
      case 67:
        if(mission_pos_cruise(now_target_x + check_catapult_x,now_target_y,TANK_ALTITUDE,0,err_max))
        {
          if(lib_time_record_func(1.0, ros::Time::now()))
          {
            mission_num = 68;
            last_request = ros::Time::now();
          }
          catapult_pub_box_large.publish(catapult_msg);
        }
        else if(ros::Time::now() - last_request > ros::Duration(5.0))
        {
          mission_num = 68;
          last_request = ros::Time::now();
          catapult_pub_box_large.publish(catapult_msg);
        }
        break;

      case 68:
        if(mission_pos_cruise(now_target_x + check_catapult_x,now_target_y,ALTITUDE,0,err_max))
        {
          if(lib_time_record_func(0.5, ros::Time::now()))
          {
            mission_num = 68;
            last_request = ros::Time::now();
          }
        }
        else if(ros::Time::now() - last_request > ros::Duration(2.0))
        {
          mission_num = 68;
          last_request = ros::Time::now();
        }
        break;
       
      
      //门
      case 7:
      if(pub_ego_goal(target6_x, target6_y, ALTITUDE, 0.2, 0))
      {
          if (lib_time_record_func(0.3, ros::Time::now())){
            mission_num = 9;
            last_request = ros::Time::now();
          }
      }
      break;
      
      //穿门任务
      case 8: //飞向二维码识别点
        if(mission_pos_cruise(target6_x , target6_y , ALTITUDE , 0 , err_max))
        {
          if (lib_time_record_func(0.5, ros::Time::now())){
            mission_num = 12;
            last_request = ros::Time::now();
          }
        }
        else if(ros::Time::now() - last_request > ros::Duration(5.0))
        {
          mission_num = 12;
          last_request = ros::Time::now();
        }
        break;
      
      //降落点降落
      case 9:
        arm_cmd.request.value = false;
        if(mission_pos_cruise(park_x,park_y,0.01,0,err_max)){
          ROS_INFO("Vehicle disarmed");
          mission_num = -1;
        }
        break;
      
      //起飞点降落
      case 10:
        arm_cmd.request.value = false;
        if(mission_pos_cruise(0, 0 ,0.01,0,err_max)){
          ROS_INFO("Vehicle disarmed");
          mission_num = -1;
        }
        break;
    }
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
    if(mission_num == -1) exit(0);
  }
  return 0;
}


