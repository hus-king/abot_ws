#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <Eigen/Eigen>

// collision_avoidance
Eigen::Vector3d Euler_fcu;
Eigen::Quaterniond q_fcu;

Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q);
void rotation_yaw(float yaw_angle, float input[2], float output[2]);

/************************************************************************
函数功能: 订阅位置信息
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
geometry_msgs::PoseStamped pos_drone;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  pos_drone = *msg;
  // Read the Quaternion from the Mavros Package [Frame: ENU]
  Eigen::Quaterniond q_fcu_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
  q_fcu = q_fcu_enu;
  // Transform the Quaternion to Euler Angles
  Euler_fcu = quaternion_to_euler(q_fcu);
}

/************************************************************************
函数功能: 接收雷达的数据避障函数
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
sensor_msgs::LaserScan Laser;      // 激光雷达点云数据
int range_min = 0;                 // 激光雷达探测范围最小角度
int range_max = 0;                 // 激光雷达探测范围最大角度
float distance_c = 0, angle_c = 0; // 最近障碍物距离 角度
float lidar_max_distance = 0;
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr &scan); // 接收雷达的数据，并做相应处理,然后计算前后左右四向最小距离
void cal_min_distance();                                     // 计算前后左右四向最小距离
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  sensor_msgs::LaserScan Laser_tmp;
  Laser_tmp = *scan;
  Laser = *scan;
  int count;
  count = Laser.ranges.size();

  // // 剔除inf的情况
  for (int i = 0; i < count; i++)
  {
    // 判断是否为inf
    int a = isinf(Laser_tmp.ranges[i]);
    // 如果为inf，则赋值上一角度的值
    // if (a == 1)
    // {
    // 	if (i == 0)
    // 	{
    // 		Laser_tmp.ranges[i] = Laser_tmp.ranges[count - 1];
    // 	}
    // 	else
    // 	{
    // 		Laser_tmp.ranges[i] = Laser_tmp.ranges[i - 1];
    // 	}
    // }
    if (a == 1)
      Laser_tmp.ranges[i] = lidar_max_distance;
  }
  for (int i = 0; i < count; i++)
  {
    if (i + 180 > 359)
      Laser.ranges[i] = Laser_tmp.ranges[i - 180];
    else
      Laser.ranges[i] = Laser_tmp.ranges[i + 180];
    // cout<<"tmp: "<<i<<" l:"<<Laser_tmp.ranges[i]<<"|| Laser: "<<Laser.ranges[i]<<endl;
  }

  cal_min_distance();
}
void cal_min_distance()
{
  distance_c = Laser.ranges[range_min];
  angle_c = 0;
  for (int i = range_min; i <= range_max; i++)
  {
    if (Laser.ranges[i] < distance_c)
    {
      distance_c = Laser.ranges[i];
      angle_c = i;
    }
  }
}

/************************************************************************
函数功能: 避障函数
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float vel_sp_ENU[2]; // ENU下的总速度

const float PI = acos(-1);
struct Point2D
{
  float x;
  float y;
  float z;
};

enum collision_avoidance_flag
{
  Cross_door,
  Normal,
  Hit
};

float scan_distance_max_current;
float scan_distance_max;
float scan_distance_max_cross_door;

float scan_distance_min_current;
float scan_distance_min;

float barrier_distance_max_current;
float barrier_distance_max;
float barrier_distance_max_cross_door;

float angle_resolution;
float sector_value;
Point2D Uavp;
vector<float> map_cv;
vector<float> barrier;
vector<double> ranges;

float vel;
float vel_cross_door;
float vel_current;

float vel_max_normal;
float vel_max_hit;
float vel_max_cross_door;
float vel_max_current;

vector<int> mesh;
vector<int> barrier_mesh;

vector<float> divid_line_current;
vector<float> divid_line;
vector<float> divid_line_cross_door;

int print_time;
float speed_mul;

bool mission_collision(float x, float y, float z, float yaw, float err_max);

const float INF = 100;

int comid = 0;

void filtering()
{
  for (int i = 0; i < 360; i++)
    if (Laser.ranges[i] > 3)
      Laser.ranges[i] = 3; // 无穷�?
  for (int i = 1; i < 359; i++)
    if (fabs(Laser.ranges[i] - Laser.ranges[i - 1]) > 1 && fabs(Laser.ranges[i] - Laser.ranges[i + 1]) > 1)
      Laser.ranges[i] = Laser.ranges[i - 1];
  // 产生的突�?
  // cout << "--------------------------------test10" << endl;

  // 高斯滤波
  int kernel_size = 30, size = 360; // 窗口大小
  double *kernel = new double[kernel_size];
  int mid = kernel_size / 2;
  // 构建高斯�?
  float sigma = 0.7;
  for (int i = 0; i < kernel_size; i++)
    kernel[i] = exp(-0.5 * pow((i - mid) / sigma, 2)) / (sigma * sqrt(2 * PI));
  //	for(int i=0;i<kernel_size;i++)	kernel[i]=1.0/kernel_size;
  // 进行滤波
  double *temp = new double[size];
  for (int i = 0; i < size; i++)
  {
    double sum = 0.0;
    for (int j = 0; j < kernel_size; j++)
    {
      int k = i + j - mid;
      k %= size;
      sum += Laser.ranges[k] * kernel[j];
    }
    temp[i] = sum;
  }
  // mcout << "--------------------------------test11" << endl;

  // 复制滤波后的数据到原始数据数�?
  for (int i = 0; i < size; i++)
  {
    Laser.ranges[i] = temp[i];
  }

  // cout << "--------------------------------test12" << endl;

  // cout << "--------------------------------test13" << endl;
  // 释放临时数组和高斯核数组
  delete[] temp;
  delete[] kernel;
}

void ComputeMV(vector<float> r, int flag)
{ // 计算直方�?
  float dist[360] = {0}, barrier_dist[360] = {0};
  ranges.clear();
  map_cv.clear();
  barrier.clear();
  int range_size = r.size(); // 激光条�?

  if (flag == Normal)
  {
    scan_distance_max_current = scan_distance_max;
    scan_distance_min_current = scan_distance_min;
    barrier_distance_max_current = barrier_distance_max;
    divid_line_current = divid_line;
    vel_current = vel;
  }
  else if (flag == Cross_door)
  {
    scan_distance_max_current = scan_distance_max_cross_door;
    scan_distance_min_current = scan_distance_min;
    barrier_distance_max_current = barrier_distance_max_cross_door;
    divid_line_current = divid_line_cross_door;
    vel_current = vel_cross_door;
  }

  for (size_t i = 0; i < range_size; i++)
  {
    // A non-zero value (true) if x is a NaN value; and zero (false) otherwise.

    // isinf A non-zero value (true) if x is an infinity; and zero (false) otherwise.
    if (!std::isnan(r[i]) && !std::isinf(r[i]))
    { // 取出有效�?
      float scan_distance = 0, barrier_distance = 0;
      // 0-30都是一个sector
      int sector_index = std::floor((i * angle_resolution) / sector_value); //(i*1)/30 sector_index:[0 12]
      if (r[i] >= scan_distance_max_current || r[i] < scan_distance_min_current)
        scan_distance = 0;
      else
        scan_distance = scan_distance_max_current - r[i]; // 距离为雷达探测最大距离减去扫描距�?  做了一个反向。实际距离越�?,scan_distence 越小

      dist[sector_index] += scan_distance; // 设定柱状高度

      if (r[i] >= barrier_distance_max_current || r[i] < scan_distance_min_current)
        barrier_distance = 0;
      else
        barrier_distance = barrier_distance_max_current - r[i];

      barrier_dist[sector_index] += barrier_distance;
    }
    ranges.push_back(r[i]); // ranges存放360度距离信�?
  }

  for (int j = 0; j < (int)(360 / sector_value); j++)
  { // 把uav四周分为12个sector,每个sector的值越�?,代表越安�?
    map_cv.push_back(dist[j]);
    barrier.push_back(barrier_dist[j]);
  }
}
/*输入期望坐标,输出期望机头方向*/
float CalculDirection(float target_x, float target_y, int flag)
{
  if (flag == Normal)
  {
    scan_distance_max_current = scan_distance_max;
    scan_distance_min_current = scan_distance_min;
    barrier_distance_max_current = barrier_distance_max;
    divid_line_current = divid_line;
    vel_current = vel;
  }
  else if (flag == Cross_door)
  {
    scan_distance_max_current = scan_distance_max_cross_door;
    scan_distance_min_current = scan_distance_min;
    barrier_distance_max_current = barrier_distance_max_cross_door;
    divid_line_current = divid_line_cross_door;
    vel_current = vel_cross_door;
  }

  float ori;
  // Compute arc tangent with two parameters
  // return Principal arc tangent of y/x, in the interval [-pi,+pi] radians.
  // One radian is equivalent to 180/PI degrees.
  float G_theta = atan2((target_y - Uavp.y), (target_x - Uavp.x));
  float goal_ori = G_theta * 180 / PI; // 目标点相对uav的方向信�?,即与y轴的夹角,正值代表在第一三象�?,负值代表在二四象限，ENU坐标系下
  if (goal_ori < 0)
    goal_ori += 360;

  // if (comid % print_time == 0)
  // 	cout << "+++++++++++++++++++<<>>goal_ori:  " << G_theta << " " << goal_ori << '\n';

  float scan_distance = 0;
  for (int i = -sector_value; i < sector_value; i++)
  {
    int real_index = ((int)goal_ori + i + 360) % 360;
    if (!std::isnan(ranges[real_index]) && !std::isinf(ranges[real_index]))
    {
      if (ranges[real_index] < scan_distance_max_current && ranges[real_index] >= scan_distance_min_current)
        scan_distance += scan_distance_max_current - ranges[real_index];
    }
  }
  // if (comid % print_time == 0)
  // 	cout << "+++++++++++++++++++++scan_dis:  " << scan_distance << '\n';
  mesh.clear();
  for (int i = 0; i < map_cv.size(); i++)
  { // 确定栅格中CV�? 共有12个栅�?,CV值越�?,存在障碍物可能性越�?
    /*if (map_cv[i] < divid_line[0])
      mesh.push_back(0);
    else if (map_cv[i] >= divid_line[0] && map_cv[i] < divid_line[1])
      mesh.push_back(1);
    else if (map_cv[i] >= divid_line[1] && map_cv[i] < divid_line[2])
      mesh.push_back(2);
    else if (map_cv[i] >= divid_line[2] && map_cv[i] < divid_line[3])
      mesh.push_back(3);
    else if (map_cv[i] >= divid_line[3])
      mesh.push_back(4);*/
    if (map_cv[i] < divid_line_current[0])
      mesh.push_back(0);
    else if (map_cv[i] >= divid_line_current[0])
      mesh.push_back(1);
  }
  barrier_mesh.clear();
  for (int i = 0; i < barrier.size(); i++)
  { // 确定栅格中CV�? 共有12个栅�?,CV值越�?,存在障碍物可能性越�?
    if (barrier[i] < divid_line_current[0])
      barrier_mesh.push_back(0);
    else if (barrier[i] >= divid_line_current[0] && barrier[i] < divid_line_current[1])
      barrier_mesh.push_back(1);
    else if (barrier[i] >= divid_line_current[1] && barrier[i] < divid_line_current[2])
      barrier_mesh.push_back(2);
    else if (barrier[i] >= divid_line_current[2] && barrier[i] < divid_line_current[3])
      barrier_mesh.push_back(3);
    else if (barrier[i] >= divid_line_current[3])
      barrier_mesh.push_back(4);
  }
  // if (comid % print_time == 0)
  // {
  // 	cout << "eve_dis:  \n";
  // 	for (int i = 0; i < mesh.size(); i++)
  // 	{
  // 		if (comid % print_time == 0)
  // 			cout << 30 * i << ":" << mesh[i] << "           ";
  // 	}
  // 	for (int i = 0; i < barrier_mesh.size(); i++)
  // 	{
  // 		if (comid % print_time == 0)
  // 			cout << 30 * i << ":" << barrier_mesh[i] << "           ";
  // 	}
  // 	for (int i = 0; i < barrier.size(); i++)
  // 	{
  // 		if (comid % print_time == 0)
  // 			cout << 30 * i << ":" << barrier[i] << "           ";
  // 	}
  // }
  // if (comid % print_time == 0)
  // 	cout << '\n'
  // 			 << '\n';
  if (scan_distance < 0.1)
    return goal_ori; // 判断目标方向处是否安�?,若安�?,则把机头指向目标
  vector<float> cand_dir;
  for (int i = 0; i < mesh.size(); i++)
  {
    if (i == mesh.size() - 1)
    {
      if (mesh[0] + mesh[mesh.size() - 1] == 0)
        cand_dir.push_back(0.0);
    }
    else
    {
      if (mesh[i] + mesh[i + 1] == 0)
        cand_dir.push_back((i + 1) * sector_value); // 寻找安全角度,即确定波�?
    }
  }
  // if (comid % print_time == 0)
  // 	cout << "---------------\n";
  // if (comid % print_time == 0)
  // 	cout << "cand_dir:\n"
  // 			 << cand_dir.size() << "\n";

  // for (int i = 0; i < cand_dir.size(); i++)
  // {
  // 	if (comid % print_time == 0)
  // 		cout << cand_dir[i] << " ";
  // }
  // if (comid % print_time == 0)
  // 	cout << '\n';
  // if (comid % print_time == 0)
  // 	cout << "-----------------\n";
  if (cand_dir.size())
  {
    float ori = 0, min_dec_value = 1000;
    for (int i = 0; i < cand_dir.size(); i++)
    {
      float dec_value = fabs(cand_dir[i] - goal_ori);
      if (dec_value < min_dec_value)
      {
        min_dec_value = dec_value;
        ori = cand_dir[i];
      }
      if (360 - dec_value < min_dec_value)
      {
        min_dec_value = 360 - dec_value;
        ori = cand_dir[i];
      }
    }
    return ori;
  }

  return -1;
}

void saturation(float &vel_x, float &vel_y)
{ // 速度函数
  for (int i = 0; i < barrier_mesh.size(); i++)
  {
    if (barrier_mesh[i] == 0)
      continue;
    int angle = i * 30 + 15;
    angle = (angle + 180) % 360;
    if (angle > 180)
      angle -= 360;
    float arc = 3.1415 / 180 * angle;
    float vel_st = 0;
    if (barrier_mesh[i] == 2)
      vel_st = vel * speed_mul * 1;
    else if (barrier_mesh[i] == 3)
      vel_st = vel * speed_mul * 2;
    else if (barrier_mesh[i] == 4)
      vel_st = vel * speed_mul * 3;
    vel_x += vel_st * cos(arc);
    vel_y += vel_st * sin(arc);
  }
  return;
}
// 0正前 90正左�?
void collision_avoidance(float target_x, float target_y, int flag)
{
  if (flag == Normal)
  {
    scan_distance_max_current = scan_distance_max;
    scan_distance_min_current = scan_distance_min;
    barrier_distance_max_current = barrier_distance_max;
    divid_line_current = divid_line;
    vel_current = vel;
    vel_max_current = vel_max_normal;
  }
  else if (flag == Cross_door)
  {
    scan_distance_max_current = scan_distance_max_cross_door;
    scan_distance_min_current = scan_distance_min;
    barrier_distance_max_current = barrier_distance_max_cross_door;
    divid_line_current = divid_line_cross_door;
    vel_current = vel_cross_door;
    vel_max_current = vel_max_cross_door;
  }
  else if (flag == Hit)
  {
    scan_distance_max_current = scan_distance_max;
    scan_distance_min_current = scan_distance_min;
    barrier_distance_max_current = barrier_distance_max;
    divid_line_current = divid_line;
    vel_current = vel;
    vel_max_current = vel_max_hit;
  }

  // cout << "------------------test1--------------" << endl;
  filtering();
  // cout << "------------------test2--------------" << endl;

  ComputeMV(Laser.ranges, flag);
  Uavp.x = pos_drone.pose.position.x;
  Uavp.y = pos_drone.pose.position.y;

  // cout << "------------------test3--------------" << endl;

  float direction = CalculDirection(target_x, target_y, flag);

  // cout << "------------------test4--------------" << endl;

  if (comid % print_time == 0)
    cout << "-------------------------------\nto:  " << direction << '\n';
  if (direction > -0.5) // direction返回0-180
  {
    if (direction > 180)
      direction -= 360;
    float arc = PI / 180 * direction;
    float vel_x = 0, vel_y = 0;
    vel_x = vel_current * cos(arc);
    vel_y = vel_current * sin(arc);
    saturation(vel_x, vel_y);

    // if (flag == Normal)
    // {
    // 	vel_x = vel_x / s_vel * vel;
    // 	vel_y = vel_y / s_vel * vel;
    // }
    // else
    // {
    // 	vel_x = vel_x / s_vel * vel_cross_door;
    // 	vel_y = vel_y / s_vel * vel_cross_door;
    // }

    // cout << "------------------test5--------------" << endl;

    // 速度限幅
    float s_vel = sqrt(vel_x * vel_x + vel_y * vel_y);
    if (s_vel > vel_max_current)
    {
      vel_x = vel_x / s_vel * vel_max_current;
      vel_y = vel_y / s_vel * vel_max_current;
    }
    // cout << "------------------test6--------------" << endl;

    vel_sp_ENU[0] = vel_x;
    vel_sp_ENU[1] = vel_y;
    if (comid % print_time == 0)
      cout << "vel:  " << vel << "\n";
    return;
  }
  else
  {
    if (comid % print_time == 0)
      cout << "ERROR\n";
    vel_sp_ENU[0] = 0;
    vel_sp_ENU[1] = 0;
    return;
  }
}

bool mission_collision(float x, float y, float z, float yaw, float error_max)
{
  comid++;
  collision_avoidance(x, y, Normal);

  setpoint_raw.type_mask = 0b100111100011; // vx vy z+ yaw
  setpoint_raw.coordinate_frame = 1;
  setpoint_raw.velocity.x = vel_sp_ENU[0];
  setpoint_raw.velocity.y = vel_sp_ENU[1];
  setpoint_raw.position.z = z;
  setpoint_raw.yaw = yaw;

  std::cout << "vx: " << vel_sp_ENU[0] << " vy: " << vel_sp_ENU[1] << "z: " << z << std::endl;

  if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max)
  {
    ROS_INFO("到达目标点，避障任务完成");
    return true;
  }
  return false;
}

// 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// q0 q1 q2 q3
// w x y z
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
{
  float quat[4];
  quat[0] = q.w();
  quat[1] = q.x();
  quat[2] = q.y();
  quat[3] = q.z();

  Eigen::Vector3d ans;
  ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
  ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
  ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
  return ans;
}

void rotation_yaw(float yaw_angle, float input[2], float output[2])
{
  output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
  output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}