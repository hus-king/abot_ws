//????????????????��?????????????????????
//????target_ring_xy ????????��??
//     target6_x ??????????????��??

// ??????????????????????????????????0%???????100%????????
// RCAIC version-1
#include <lib_library.h>
#include <complete_mission.h>

int mission_num = 0;   //mission_flag
int tmp_marker_id = 1;
int box_number = 0;

//qr
float target_qr_x = 0,target_qr_y = 0;

// the four picture
float target1_x = 0, target1_y = 0;
float target2_x = 0, target2_y = 0;
float target3_x = 0, target3_y = 0;
float target4_x = 0, target4_y = 0;
float target5_x = 0, target5_y = 0;
float target6_x = 0, target6_y = 0;
float target_ring_x = 0 , target_ring_y = 0; // ????????????��
float fly_ring_x = 0 , fly_ring_y = 0; // ?????????????????��
float ring_back_x = 0 , ring_back_y = 0; // ?????��?????????????
float temp_x = 0 , temp_y = 0 ;

float err_max = 0;

float fly_height = 0.75;

float align_threshold = 20.0; // ??????????(????)
float adjust_step = 0.05;     // ??��???????(??)
float backward_step = 0.1;    // ??��??????(??)
float max_backward = 0.5;     // ?????????(??)
float through_distance = 1.5; // ???????��???(??)

int ring_mission_flag = -1 ;


void print_param()
{
  std::cout << "target_qr : ( " << target_qr_x << ", " << target_qr_y << " )" << std::endl;
  std::cout << "target1 : ( " << target1_x << ", " << target1_y << " )" << std::endl;
  std::cout << "target2 : ( " << target2_x << ", " << target2_y << " )" << std::endl;
  std::cout << "target3 : ( " << target3_x << ", " << target3_y << " )" << std::endl;
  std::cout << "target4 : ( " << target4_x << ", " << target4_y << " )" << std::endl;
  std::cout << "target5 : ( " << target5_x << ", " << target5_y << " )" << std::endl;
  std::cout << "target6 : ( " << target6_x << ", " << target6_y << " )" << std::endl;

  std::cout << "target_ring : ( " << target_ring_x << ", " << target_ring_y << " )" << std::endl;
  std::cout << "ring_back : ( " << ring_back_x << ", " << ring_back_y << " )" << std::endl;
  
  std::cout << "err_max : " << err_max << std::endl;
  std::cout << "fly_height : " << fly_height << std::endl;

  ROS_INFO("Ring mission started with parameters:");
  ROS_INFO("align_threshold: %.1fpx", align_threshold);
  ROS_INFO("adjust_step: %.3fm", adjust_step);
  ROS_INFO("backward_step: %.3fm", backward_step);
  ROS_INFO("max_backward: %.3fm", max_backward);
  ROS_INFO("through_distance: %.3fm", through_distance);
}

int main(int argc, char **argv)
{
  // ??????????????
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "complete_mission");

  // ?????????
  ros::NodeHandle nh;

  // ????ego_planner?��????????
  //ros::Subscriber ego_sub = nh.subscribe("/position_cmd", 100, ego_sub_cb);

  // ????ego_planner???
  //planner_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal", 100);

  //ros::Subscriber rec_traj_sub = nh.subscribe("/rec_traj", 100, rec_traj_cb);

  //finish_ego_pub = nh.advertise<std_msgs::Bool>("/finish_ego", 1);

  // ???????Subscriber??????????????/mavros/state??topic???????????state_cb
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

  // ???????Subscriber??????????????/mavros/local_position/odom??topic???????????local_pos_cb
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  // ????move_base?��?????????????????
  // ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmd_to_px4);

  ros::Subscriber qr_sub = nh.subscribe("/qr_code_detection/qrcode_data", 10 , qr_cb);

  ros::Subscriber circle_sub = nh.subscribe("/circle_detector/position", 1 , circle_detection_cb);

  // ???????????????????
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

  // ???????????????????????/mavros/cmd/arming??????????????????????
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

  // ???????????????????????/mavros/set_mode??????????????????????offboard??
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // ??????????????
  ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  ros::Publisher catapult_pub_box1 = nh.advertise<std_msgs::Empty>("servo/front_left/open", 1);
  ros::Publisher catapult_pub_box2 = nh.advertise<std_msgs::Empty>("servo/front_right/open", 1);
  ros::Publisher catapult_pub_box_large = nh.advertise<std_msgs::Empty>("servo/all/open", 1);

  // ????????????????????2Hz???????????500ms????????
  ros::Rate rate(20);

  //  ???????
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

  nh.param<float>("target_ring_x", target_ring_x, 0);
  nh.param<float>("target_ring_y", target_ring_y, 0);

  nh.param<float>("ring_back_x", ring_back_x, 7.5);
  nh.param<float>("ring_back_y", ring_back_y, -2.0);

  nh.param<float>("err_max", err_max, 0);
  nh.param<float>("fly_height", fly_height, 0);

  nh.param<float>("align_threshold", align_threshold, 50.0);
  nh.param<float>("adjust_step", adjust_step, 0.05);
  nh.param<float>("backward_step", backward_step, 0.1);
  nh.param<float>("max_backward", max_backward, 0.5);
  nh.param<float>("through_distance", through_distance, 3.0);

  


  print_param();

  int choice = 0;
  std::cout << "1 to go on , else to quit" << std::endl;
  std::cin >> choice;
  if (choice != 1)
    return 0;

  // ???????????
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  //???????????????��??
 
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

  // ??????????????????offboard??
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // ??????????????????????????
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  // ????????????????????last_request
  ros::Time last_request = ros::Time::now();

  // ?????????��????????5??6??int??0-100????????
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
    // ?????????????????????3????????????????????��??
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
  
  // ?????????????��
  bool tmp_time_record_start_flag = false;
  // ??????????????
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
printf("ring_mission_flag = %d\r\n",ring_mission_flag);
printf("latest_circle_x=%d\r\n",latest_circle_x);
    printf("mission_num = %d\r\n", mission_num);
    switch (mission_num)
    {
      //mission1:recognize qr_code
      case 1:
        if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
        {
          mission_num = 73;
        }
        break;

      case 73:
        if(mission_pos_cruise(target_ring_x,target_ring_y,RING_HEIGHT,0,err_max))
        {
          if(lib_time_record_func(0.5,ros::Time::now()))
          {
            mission_num = 8;
          }
        }
	      break;

      //pass_ring
      case 8://????????????????
        if(mission_pos_cruise(target_ring_x,target_ring_y,RING_HEIGHT,-1.57,err_max))
        {
          last_time = ros::Time::now();
          if(lib_time_record_func(2.0,ros::Time::now())) {
            mission_num = 81;
            latest_circle_x = 0;
            latest_circle_y = 0;
ros::spinOnce(); 
          }
        }
	      break;

      case 81: // ???????
        ring_mission_flag = ring_mission(align_threshold) ;

        if( ring_mission_flag == 0 ) { //????
          ROS_WARN("<<<<<<<<<<ADJUST SUCESSFULLY>>>>>>>>>>");
          ROS_WARN("<<<<<<<<<<ADJUST SUCESSFULLY>>>>>>>>>>");
          ROS_WARN("<<<<<<<<<<ADJUST SUCESSFULLY>>>>>>>>>>");
          fly_ring_x = local_pos.pose.pose.position.x ;
          // ??????��???????????????Y????? ??????y???ring_back_y
          //fly_ring_y = local_pos.pose.pose.position.y - through_distance;
          fly_ring_y = ring_back_y ;

          mission_num = 83;
        }
        else {
          if( ring_mission_flag == 1 ) { //??????????
            temp_x = local_pos.pose.pose.position.x - adjust_step ;
            temp_y = target_ring_y ;
            if (mission_pos_cruise(temp_x , temp_y , RING_HEIGHT, -1.57, err_max)) {
              mission_num = 81;
            }
          }
          else {
            if ( ring_mission_flag == 2 ) { //??????????
              temp_x = local_pos.pose.pose.position.x + adjust_step ;
              temp_y = target_ring_y ;
              if (mission_pos_cruise(temp_x , temp_y , RING_HEIGHT, -1.57, err_max)) {
                mission_num = 81;
              }
            }
          }
        }

        if(ros::Time::now() - last_time >= ros::Duration(10.0)) {
          ROS_WARN("<<<<<<<<<<TIME OUT>>>>>>>>>>");
          ROS_WARN("<<<<<<<<<<TIME OUT>>>>>>>>>>");
          ROS_WARN("<<<<<<<<<<TIME OUT>>>>>>>>>>");
          mission_num = 9;  //????????????????9??????target6_x
        }
        break;

          
      case 83: // ???????????????
        ROS_WARN("<<<<<<<flying %.2f , %.2f >>>>>>>>", fly_ring_x , fly_ring_y);
        //if(lib_time_record_func(1.0,ros::Time::now())) {
        //    mission_num = 9; // ??????????
        //}
        
        if (mission_pos_cruise(fly_ring_x , fly_ring_y , RING_HEIGHT, -1.57, err_max)) {
            ROS_INFO("<<<<<<<Through ring completed, flying %.2f , %.2f >>>>>>>>", fly_ring_x , fly_ring_y);
            if(lib_time_record_func(2.0,ros::Time::now())) {
              mission_num = 9; // ??????????
            }
        }
        break;

        
      //??????
      case 9:
        if(mission_pos_cruise(fly_ring_x,fly_ring_y,ALTITUDE,-1.57,err_max))
        {
          mission_num = 100;
        }
	      break;

      case 100:
        arm_cmd.request.value = false;
        if(mission_pos_cruise(fly_ring_x,fly_ring_y,0.01,-1.57,err_max)){
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
