#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

#define RAD2DEG(x) ((x)*180./M_PI)

class CppSample : public rclcpp::Node 
{
private:
  size_t counter;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;

public:
  CppSample() : Node("cpp_sample"), counter(0) {

    timer = this->create_wall_timer(
      100ms, 
      [this]() {
        RCLCPP_INFO(this->get_logger(), "Sample file called %zu times", ++(this->counter));
      });

    sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), 
      [this](sensor_msgs::msg::LaserScan::SharedPtr scan) {
        int scan_num = std::round( ((scan->angle_max - scan->angle_min) / scan->angle_increment) / 1e1 ) * 1e1;
        for (int i=0;i<scan_num;i++) {
          double degree = RAD2DEG( scan->angle_increment * i ); 	// The first point is defined as 0 degrees.
          printf("[LIDAR INFO]:angle-distance:[%4.1f, %5.3f]\n", degree,scan->ranges[i]);
        }
      });
  }
};


int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CppSample>();

  rclcpp::spin(node);
  
  rclcpp::shutdown();

  return 0;
}