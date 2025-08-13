// 说明：舵机方向根据实际调整，这里用的占空比0%表示闭合，100%表示全部打开
// RCAIC version-2
// collision_avoidance and catapult
#include <lib_library.h>
#include <complete_mission.h>


int mission_num = 0;   //mission_flag
int tmp_marker_id = 1;
int box_number = 0;

float box1_x = 0, box1_y = 0;
float box2_x = 0, box2_y = 0;
float box3_x = 0, box3_y = 0;
float box4_x = 0, box4_y = 0;
float box5_x = 0, box5_y = 0;

int if_point = 0;
int if_safe = 0;
int point1 = 0;
int point2 = 0;
int point3 = 0;
int point4 = 0;

//qr
float target_qr_x = 0,target_qr_y = 0;

// the four picture
float target1_x = 0, target1_y = 0;
float target2_x = 0, target2_y = 0;
float target3_x = 0, target3_y = 0;
float target4_x = 0, target4_y = 0;
float target5_x = 0, target5_y = 0;
float target6_x = 0, target6_y = 0;
float park_left_x = 0,park_left_y = 0;
float park_right_x = 0,park_right_y = 0;
float target_ring_x = 0 , target_ring_y = 0; // 识别圆环时悬停点位
float fly_ring_x = 0 , fly_ring_y = 0; // 识别到圆后计算得到的圆后点位
float ring_back_x = 0 , ring_back_y = 0; // 环后点位（世界坐标系）
float temp_x = 0 , temp_y = 0 ;

float err_max = 0;

float fly_height = 0.75;

float align_threshold = 20.0; // 横向对齐阈值(像素)
float adjust_step = 0.05;     // 每次调整步长(米)
float backward_step = 0.1;    // 每次后退步长(米)
float max_backward = 0.5;     // 最大后退距离(米)
float through_distance = 1.5; // 穿环飞行距离(米)

float check_catapult_x = 0.0;
float check_catapult_y = 0.0;

int ring_mission_flag = -1 ; //是否继续绕圈

std::string qr_msg1 = "";
std::string qr_msg2 = "";
std::string qr_msg3 = "";



void print_param()
{
  std::cout << "box1 : ( " << box1_x << ", " << box1_y << " )" << std::endl;
  std::cout << "box2 : ( " << box2_x << ", " << box2_y << " )" << std::endl;
  std::cout << "box3 : ( " << box3_x << ", " << box3_y << " )" << std::endl;
  std::cout << "box4 : ( " << box4_x << ", " << box4_y << " )" << std::endl;
  std::cout << "box5 : ( " << box5_x << ", " << box5_y << " )" << std::endl;
  
  std::cout << "park_left : ( " << park_left_x << ", " << park_left_y << " )" << std::endl;
  std::cout << "target_qr : ( " << target_qr_x << ", " << target_qr_y << " )" << std::endl;
  std::cout << "target1 : ( " << target1_x << ", " << target1_y << " )" << std::endl;
  std::cout << "target2 : ( " << target2_x << ", " << target2_y << " )" << std::endl;
  std::cout << "target3 : ( " << target3_x << ", " << target3_y << " )" << std::endl;
  std::cout << "target4 : ( " << target4_x << ", " << target4_y << " )" << std::endl;
  std::cout << "target5 : ( " << target5_x << ", " << target5_y << " )" << std::endl;
  std::cout << "target6 : ( " << target6_x << ", " << target6_y << " )" << std::endl;

  std::cout << "park_left : ( " << park_left_x << ", " << park_left_y << " )" << std::endl;
  std::cout << "park_right : ( " << park_right_x << ", " << park_right_y << " )" << std::endl;

  std::cout << "target_ring : ( " << target_ring_x << ", " << target_ring_y << " )" << std::endl;
  std::cout << "ring_back : ( " << ring_back_x << ", " << ring_back_y << " )" << std::endl;
  
  std::cout << "check_catapult_x : " << check_catapult_x << std::endl;
  std::cout << "check_catapult_y : " << check_catapult_y << std::endl;

  std::cout << "err_max : " << err_max << std::endl;
  std::cout << "fly_height : " << fly_height << std::endl;
  std::cout << "qr_msg : " << qr_msg1.c_str() << "," << qr_msg2.c_str() << "," << qr_msg3.c_str() << std::endl;

  std::cout << "if_point : " << if_point << std::endl;
  std::cout << "if_safe : " << if_safe << std::endl;
  std::cout << "point1 : " << point1 << std::endl;
  std::cout << "point2 : " << point2 << std::endl;
  std::cout << "point3 : " << point3 << std::endl;
  std::cout << "point4 : " << point4 << std::endl;

  ROS_INFO("Ring mission started with parameters:");
  ROS_INFO("align_threshold: %.1fpx", align_threshold);
  ROS_INFO("adjust_step: %.3fm", adjust_step);
  ROS_INFO("backward_step: %.3fm", backward_step);
  ROS_INFO("max_backward: %.3fm", max_backward);
  ROS_INFO("through_distance: %.3fm", through_distance);

  std::cout << "spin_flag : " << spin_flag << std::endl;
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

  finish_ego_pub = nh.advertise<std_msgs::Bool>("/finish_ego", 1);

  // 创建一个Subscriber订阅者，订阅名为/mavros/state的topic，注册回调函数state_cb
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

  // 创建一个Subscriber订阅者，订阅名为/mavros/local_position/odom的topic，注册回调函数local_pos_cb
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  // 订阅move_base规划出来的速度，用于避障
  // ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmd_to_px4);

  ros::Subscriber qr_sub = nh.subscribe("/qrcode_data", 10 , qr_cb);

  ros::Subscriber circle_sub = nh.subscribe("/circle_detector/position", 1 , circle_detection_cb);

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
  ros::Subscriber black_square_sub = nh.subscribe<geometry_msgs::Point>("/ocvd/target_center",1,black_square_cb);


  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  //  参数读取
  
  nh.param<float>("box1_x", box1_x, 0);
  nh.param<float>("box1_y", box1_y, 0);
  nh.param<float>("box2_x", box2_x, 0);
  nh.param<float>("box2_y", box2_y, 0);
  nh.param<float>("box3_x", box3_x, 0);
  nh.param<float>("box3_y", box3_y, 0);
  nh.param<float>("box4_x", box4_x, 0);
  nh.param<float>("box4_y", box4_y, 0);
  nh.param<float>("box5_x", box5_x, 0);
  nh.param<float>("box5_y", box5_y, 0);

  nh.param<float>("target_qr_x", target_qr_x, 0);
  nh.param<float>("target_qr_y", target_qr_y, 0);
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

  nh.param<float>("park_left_x", park_left_x, 0);
  nh.param<float>("park_left_y", park_left_y, 0);
  nh.param<float>("park_right_x", park_right_x, 0);
  nh.param<float>("park_right_y", park_right_y, 0);

  nh.param<float>("target_ring_x", target_ring_x, 0);
  nh.param<float>("target_ring_y", target_ring_y, 0);

  nh.param<float>("ring_back_x", ring_back_x, 7.5);
  nh.param<float>("ring_back_y", ring_back_y, -2.0);

  nh.param<float>("err_max", err_max, 0);
  nh.param<float>("fly_height", fly_height, 0);

  nh.param<float>("check_catapult_x", check_catapult_x, 0);
  nh.param<float>("check_catapult_y", check_catapult_y, 0);

  nh.param<float>("align_threshold", align_threshold, 50.0);
  nh.param<float>("adjust_step", adjust_step, 0.05);
  nh.param<float>("backward_step", backward_step, 0.1);
  nh.param<float>("max_backward", max_backward, 0.5);
  nh.param<float>("through_distance", through_distance, 3.0);

  nh.param<std::string>("qr_msg1", qr_msg1, "");
  nh.param<std::string>("qr_msg2", qr_msg2, "");
  nh.param<std::string>("qr_msg3", qr_msg3, "");

  nh.param<int>("if_safe", if_safe, 1);
  nh.param<int>("if_point", if_point, 1); 
  nh.param<int>("point1", point1, 1);
  nh.param<int>("point2", point2, 1);
  nh.param<int>("point3", point3, 1);
  nh.param<int>("point4", point4, 1);

  nh.param<int>("spin_flag", spin_flag, 1);


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

  float now_check_catapult_x = 0.0;
  float now_check_catapult_y = 0.0; 

  float now_box_target_x = 0.0;
  float now_box_target_y = 0.0;

  int tmp_mission = 1;
  bool black_square_flag = false;
  std_msgs::Empty catapult_msg;
  string test_black_flag = "false";

  while (ros::ok())
  {
    if (tmp_mission != mission_num)
    {
      tmp_mission = mission_num;
      printf("change mission_num = %d\r\n", mission_num);
    }

    printf("mission_num = %d\r\n", mission_num);
    printf("1.%s,2.%s  havefound:%s\n",read_data[0].c_str(),read_data[1].c_str(),have_found.c_str());
    printf("detect_code:%s,%s,%s\n",read_data[0].c_str(),read_data[1].c_str(),read_data[2].c_str());
    printf("black_square:%s\n",test_black_flag);
    
    switch (mission_num)
    {
      //mission1:recognize qr_code
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
      
      
      //flying to the first point
      case 2:
        if (mission_pos_cruise(target1_x , target1_y , ALTITUDE , 0 , err_max))
        {
          ;
        }
        // {
        //   if (lib_time_record_func(0.5, ros::Time::now()))
        //   {
        //     if(if_point)
        //     {
        //       if(point1)
        //       {
        //         mission_num = 22;
        //         box_flag = true;
        //         now_box_target_x = target1_x;
        //         now_box_target_y = target1_y;
        //         last_request = ros::Time::now();
        //       }
        //       else
        //       {
        //         mission_num = 23;
        //         last_request = ros::Time::now();
        //       }
        //     }
        //     else
        //     {
        //       mission_num = 21;    
        //       found = false; //重置识别标志位
        //       start_checking = true;
        //       last_request = ros::Time::now();
        //     }
        //   }
        // }
        // else if(ros::Time::now()-last_request >= ros::Duration(3.0))
        // {
        //   if(if_point)
        //     {
        //       if(point1)
        //       {
        //         mission_num = 22;
        //         box_flag = true;
        //         now_box_target_x = target1_x;
        //         now_box_target_y = target1_y;
        //         last_request = ros::Time::now();
        //       }
        //       else
        //       {
        //         mission_num = 23;
        //         last_request = ros::Time::now();
        //       }
        //     }
        //     else
        //     {
        //       mission_num = 21;
        //       found = false; //重置识别标志位
        //       start_checking = true;
        //       last_request = ros::Time::now();
        //     }
        // }
        break;

      case 21:
        if(judge_box_target(target1_x,target1_y))
        {
          black_square_flag = true;
          test_black_flag = "true";
        }
        else if(ros::Time::now() - last_request >= ros::Duration(2.0))
        {
          black_square_flag = true;
        }
        mission_view(target1_x, target1_y, VIEW_ALTITUDE, 0);
        
        if (black_square_flag)
        {
          last_request = ros::Time::now();
          black_square_flag = false;
          mission_num = 22;
          start_checking = false;
          box_flag = true;
          now_box_target_x = box_target_x;
          now_box_target_y = box_target_y;
          ROS_WARN("TARGET:dx0:%f,dy:%f,box_target_x:%f,box_target_y:%f,center_x:%f,center_y:%f",dx0,dy,box_target_x,box_target_y,center_x,center_y);
          ROS_WARN("TARGET:dx0:%f,dy:%f,box_target_x:%f,box_target_y:%f,center_x:%f,center_y:%f",dx0,dy,box_target_x,box_target_y,center_x,center_y);
          ROS_WARN("TARGET:dx0:%f,dy:%f,box_target_x:%f,box_target_y:%f,center_x:%f,center_y:%f",dx0,dy,box_target_x,box_target_y,center_x,center_y);
          ROS_WARN("TARGET:dx0:%f,dy:%f,box_target_x:%f,box_target_y:%f,center_x:%f,center_y:%f",dx0,dy,box_target_x,box_target_y,center_x,center_y);
          ROS_WARN("TARGET:dx0:%f,dy:%f,box_target_x:%f,box_target_y:%f,center_x:%f,center_y:%f",dx0,dy,box_target_x,box_target_y,center_x,center_y);
          ROS_WARN("TARGET:dx0:%f,dy:%f,box_target_x:%f,box_target_y:%f,center_x:%f,center_y:%f",dx0,dy,box_target_x,box_target_y,center_x,center_y);
          if(box_number == 0)
          {
            now_check_catapult_x = -check_catapult_x;
            now_check_catapult_y = -check_catapult_y;
          }
          else if(box_number == 1)
          {
            now_check_catapult_x = -check_catapult_x;
            now_check_catapult_y = check_catapult_y;
          }
        }
        else
        {
          if(ros::Time::now() - last_request >= ros::Duration(5.0))
          { 
            start_checking = false;
            found = false; //重置识别标志位
            if(point1 && if_safe)
            {
              mission_num = 22;
              
              box_flag = true;
              now_box_target_x = target1_x;
              now_box_target_y = target1_y;
            }
            else
            {
              mission_num = 23;
            }
            last_request = ros::Time::now();
            if(box_number == 0)
            {
              now_check_catapult_x = -check_catapult_x;
              now_check_catapult_y = -check_catapult_y;
            }
            else if(box_number == 1)
            {
              now_check_catapult_x = -check_catapult_x;
              now_check_catapult_y = check_catapult_y;
            }
          }
        }
        break;

      case 22:
        if (mission_pos_cruise(now_box_target_x + now_check_catapult_x , now_box_target_y + now_check_catapult_y , 0.03 , 0 , err_max))
        {
          if(box_flag)
          {
            box_number++;
            box_flag = false;
          }
          if(lib_time_record_func(1.0, ros::Time::now()))
          {
            mission_num = 23;
            last_request = ros::Time::now();
          }
          catapult_pub_box1.publish(catapult_msg);
        }
        break;

      case 23:
        if (mission_pos_cruise(target1_x , target1_y , ALTITUDE , 0 , err_max))
        {
          if(lib_time_record_func(0.5, ros::Time::now()))
          {
            mission_num = 100;
            last_request = ros::Time::now();
          }
        }
        else if(ros::Time::now()-last_request >= ros::Duration(3.0))
        {
          mission_num = 100;
          last_request = ros::Time::now();
        }
        break;

      case 100:
        // arm_cmd.request.value = false;
        // if(mission_pos_cruise(park_right_x,park_right_y,0.01,0,err_max)){
        //   ROS_INFO("Vehicle disarmed");
        //   mission_num = -1;
        // }
        // break;
        arm_cmd.request.value = false;
        if(mission_pos_cruise(0,0,0.01,0,err_max)){
          ROS_INFO("Vehicle disarmed");
          mission_num = -1;
        }
        break;

      case 101:
        arm_cmd.request.value = false;
        if(mission_pos_cruise(park_left_x,park_left_y,0.01,0,err_max)){
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


