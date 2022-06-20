#include <iostream>
#include <string>
#include <unistd.h>
#include <cstring>
#include <unordered_map>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>

using namespace std;

const int arraySize = 20;

ros::Publisher pubRobotState;

unordered_map<string, int> hashState = {{"/robotMode", 0},{"/decision", 1}, {"/exploration", 2}, {"/hdl_localization_nodelet", 3},{"/localPlanner", 4},
                                                                                      {"/imageProjection", 5},{"/ps3_joy", 6},{"/velodyne_nodelet_manager", 7},{"/joyControl", 8},{"/realsense", 9},
                                                                                      {"/detectionStairs", 10},{"/findStairs", 11},{"/combStairs", 12},{"/zerdCamera", 13},{"/dds", 14},
                                                                                      {"/cmd", 15},{"/zerdCamera", 16},{"/zerdCamera", 17},{"/zerdCamera", 18},{"/zerdCamera", 19}};
char robotState[arraySize] = {'0'};
std_msgs::UInt8MultiArray uintArray;

bool excute_cmd(const std::string &cmd)
{
  FILE *fp = NULL;
  char result_buf[1024];
  const char *sysCommand = cmd.data();
  fp = popen(sysCommand, "r");
  if (fp == NULL)
  {
    std::cout << "popen failed!" << std::endl;
    return false;
  }
  for(int i=1;i<sizeof(robotState);++i){
    robotState[i] = '0';
    uintArray.data[i]=0;
  }
  
  while (fgets(result_buf, sizeof(result_buf), fp) != NULL)
  {
    if (result_buf[strlen(result_buf) - 1] == '\n')
      result_buf[strlen(result_buf) - 1] = '\0';
    string str(result_buf);
    if (hashState.count(str) != 0)
    {
      robotState[hashState[str]] = 1+'0';
      uintArray.data[hashState[str]] =  1;
    }
  }
  pubRobotState.publish(uintArray);
  std::cout << robotState << std::endl;
  pclose(fp);
  return true;
}

void robotModeCallback(const std_msgs::UInt8::ConstPtr &msg){
  robotState[0] = msg->data + '0';
  uintArray.data[0] = msg->data;
}

void robotStateThread(int rate){
  ros::Rate r(rate);
   while (ros::ok())
  {
    excute_cmd("rosnode list");
    r.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robotState");

  ROS_INFO("\033[1;32m---->\033[0m robotState Started.");

  uintArray.data.resize(20);

  ros::NodeHandle nh;
  pubRobotState = nh.advertise<std_msgs::UInt8MultiArray>("robotState",10);
  ros::Subscriber subRobotMode = nh.subscribe<std_msgs::UInt8>("robotMode",10,robotModeCallback);

  thread pubRobotState  = thread(robotStateThread,1);

  ros::spin();

  pubRobotState.join();

  return 0;
}
