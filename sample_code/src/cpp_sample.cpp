#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#define RAD2DEG(x) ((x)*180./M_PI)

int counter=0;


void callback(const ros::TimerEvent&){
  counter++;
  ROS_INFO("sample file called : %d\t times",counter);
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
  int scan_num = std::round( ((scan->angle_max - scan->angle_min) / scan->angle_increment) / 1e1 ) * 1e1;
  for(int i=0;i<scan_num;i++){
    float degree=RAD2DEG( scan->angle_increment * i ); 	// The first point is defined as 0 degrees.
    printf("[LIDAR INFO]:angle-distance:[%4.1f, %5.3f]\n", degree,scan->ranges[i]);
  }
}


int main(int argc, char **argv){
  ros::init(argc,argv,"cpp_sample");

  ros::NodeHandle n;

  ros::Timer timer1 = n.createTimer(ros::Duration(0.1),callback);
  
  ros::Subscriber sub=n.subscribe<sensor_msgs::LaserScan>("/scan",1000,scanCallback);

  ros::spin();

  return 0;
}
