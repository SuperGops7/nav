#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "turtlebot_interfaces/srv/trajectory_request.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

class OdomSubscriber : public rclcpp::Node
{
  public:
    OdomSubscriber()
    : Node("odom_subscriber"), x(0), y(0), z(0), sec(0), nsec(0)
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&OdomSubscriber::topic_callback, this, _1));

        // file_.open("trajectory.csv", std::ios::out); - csv
        file_.open("trajectory.json", std::ios::out);
        if (file_.is_open())
        {
          // file_ << "sec,nsec,x,y,z\n"; - csv
          // file_<< "#trajectory\n"; //- yaml
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory for writing!");
        }
    }

    ~OdomSubscriber()
    {
        // Close the file stream when the node is destroyed.
        if (file_.is_open())
        {
        file_.close();
        }
    }

  private:

    void topic_callback(const nav_msgs::msg::Odometry & msg)
    {
          sec = msg.header.stamp.sec;
          nsec = msg.header.stamp.nanosec;
          x = msg.pose.pose.position.x;
          y = msg.pose.pose.position.y;
          z = msg.pose.pose.position.z;

          RCLCPP_INFO(this->get_logger(), "sec: %d, nsec: %u, x: %f, y: %f, z: %f",
                sec, nsec, x, y, z);

          // CSV
          // if (file_.is_open())
          //   {
          //   file_ << sec << "," << nsec << "," << x << "," << y << "," << z << "\n";
          //   }

          if (file_.is_open())
          {
            file_ << "sec:" << sec <<"\n";
            file_ << "nsec:" << nsec <<"\n";
            file_ << "x:" << x <<"\n";
            file_ << "y:" << y <<"\n";
            file_ << "z:" << z <<"\n";
            file_ << "---\n";
          }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    double x;
    double y;
    double z;
    int32_t sec;
    uint32_t nsec;
    std::ofstream file_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomSubscriber>());
  rclcpp::shutdown();
  return 0;
}