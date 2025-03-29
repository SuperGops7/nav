#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "turtlebot_interfaces/srv/trajectory_request.hpp"
#include "trajectory_struct.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class OdomSubscriber : public rclcpp::Node
{
  public:
    OdomSubscriber()
    : Node("odom_subscriber")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&OdomSubscriber::odom_pub_callback, this, _1));

        trajectory_capture_service_ = this->create_service<turtlebot_interfaces::srv::TrajectoryRequest>(
        "trajectory_capture_service", std::bind(&OdomSubscriber::trajectory_capture_callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Trajectory Subscriber and Service Started");
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

    void odom_pub_callback(const nav_msgs::msg::Odometry & msg)
    {
          TrajectoryStruct trajectory_point;
          trajectory_point.sec = msg.header.stamp.sec;
          trajectory_point.nsec = msg.header.stamp.nanosec;
          trajectory_point.x = msg.pose.pose.position.x;
          trajectory_point.y = msg.pose.pose.position.y;
          trajectory_point.z = msg.pose.pose.position.z;

          RCLCPP_INFO(this->get_logger(), "sec: %d, nsec: %u, x: %f, y: %f, z: %f",
                trajectory_point.sec, trajectory_point.nsec, trajectory_point.x, trajectory_point.y, trajectory_point.z);

          trajectory_.push_back(trajectory_point);
    }

    void pruneOldTrajectoryPoints(std::vector<TrajectoryStruct>& trajectory, double duration) {
  if (trajectory.empty()) {
    return;
  }

  // Compute the time (in seconds) of the latest (most recent) entry.
  double latest_time = trajectory.back().sec + trajectory.back().nsec * 1e-9;

  // Get the time of the oldest entry.
  double first_time = trajectory.front().sec + trajectory.front().nsec * 1e-9;

  // If the oldest entry is less than 40 seconds older than the latest, then no pruning is needed.
  if (latest_time - first_time <= duration) {
    return;
  }

  // Remove all entries older than 40 seconds relative to the latest time.
  trajectory.erase(
    std::remove_if(trajectory.begin(), trajectory.end(),
      [latest_time, duration](const TrajectoryStruct& point) {
        double point_time = point.sec + point.nsec * 1e-9;
        return (latest_time - point_time > duration);
      }
    ),
    trajectory.end()
  );
}


    void trajectory_capture_callback(
    const std::shared_ptr<turtlebot_interfaces::srv::TrajectoryRequest_Request> request,
    std::shared_ptr<turtlebot_interfaces::srv::TrajectoryRequest_Response> response)
{
    std::string filename = request->filename;
    double duration = request->duration;
    RCLCPP_INFO(this->get_logger(), "Saving trajectory with %s", request->filename.c_str());
    RCLCPP_INFO(this->get_logger(), "Saving trajectory with %f", duration);

    pruneOldTrajectoryPoints(trajectory_, duration);

    std::string dir = "src/trajectory_capture/logs/";
    std::string full_path = dir + request->filename;
    file_.open(full_path.c_str(), std::ios::out);
    if (file_.is_open())
    {
      if (filename.size() >= 4 && filename.substr(filename.size() - 4) == ".csv"){
        file_ << "sec,nsec,x,y,z\n";
        for (const auto &trajectory_point : trajectory_) {
          file_ << trajectory_point.sec << "," << trajectory_point.nsec << ","
                << trajectory_point.x   << "," << trajectory_point.y   << "," << trajectory_point.z << "\n";
        }
    } else if (filename.size() >= 5 && filename.substr(filename.size() - 5) == ".yaml") {
        file_<< "#trajectory\n";
        for (const auto &trajectory_point : trajectory_) {
          file_ << "sec: " << trajectory_point.sec << "\n"
                << "nsec: " << trajectory_point.nsec << "\n"
                << "x: " << trajectory_point.x << "\n"
                << "y: " << trajectory_point.y << "\n"
                << "z: " << trajectory_point.z << "\n"
                << "---\n";
        }
    } else if (filename.size() >= 5 && filename.substr(filename.size() - 5) == ".json") {
        file_ << "[\n";
        for (size_t i = 0; i < trajectory_.size(); i++) {
          const auto &trajectory_point = trajectory_[i];
          file_ << "  {\n"
                << "    \"sec\": " << trajectory_point.sec << ",\n"
                << "    \"nsec\": " << trajectory_point.nsec << ",\n"
                << "    \"x\": " << trajectory_point.x << ",\n"
                << "    \"y\": " << trajectory_point.y << ",\n"
                << "    \"z\": " << trajectory_point.z << "\n"
                << "  }";
          if (i < trajectory_.size() - 1) {
            file_ << ",";
          }
          file_ << "\n";
        }
        file_ << "]\n";
      }
      file_.close();
    }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory for writing!");
        }

    response->success = true;
    response->message = "Trajectory captured successfully";
}


    rclcpp::Service<turtlebot_interfaces::srv::TrajectoryRequest>::SharedPtr trajectory_capture_service_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::vector<TrajectoryStruct> trajectory_;
    std::vector<TrajectoryStruct> updated_trajectory_;
    std::ofstream file_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomSubscriber>());
  rclcpp::shutdown();
  return 0;
}