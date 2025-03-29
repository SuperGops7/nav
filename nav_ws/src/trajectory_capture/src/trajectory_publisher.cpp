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
#include "visualization_msgs/msg/marker_array.hpp"

class TrajPublisher : public rclcpp::Node
{
    public:
        TrajPublisher()
        : Node("trajectory_publisher")
        {
            publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/trajectory_points_array", 10);

            std::string dir = "src/trajectory_capture/logs/";
            std::string full_path = dir + "test.csv";
            read_and_store_trajectory(full_path.c_str());

            timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrajPublisher::publish_trajectory, this));

            RCLCPP_INFO(this->get_logger(), "Trajectory Publisher Started");
        }

    private:
        void read_and_store_trajectory(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
            return;
        }
        stored_trajectory_.markers.clear();
        visualization_msgs::msg::MarkerArray vector_marker;
        std::string line;
        int id = 0;

        while (std::getline(file, line)) {
            if (line == "sec,nsec,x,y,z"){
                continue;
            }
            std::istringstream ss(line);
            int32_t sec;
            uint32_t nsec;
            double x, y, z;
            char comma;
            if (!(ss >> sec >> comma >> nsec >> comma >> x >> comma >> y >> comma >> z)) {
                RCLCPP_ERROR(this->get_logger(), "Invalid CSV format for: %s", line.c_str());
                continue;
            }

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = this->now();
            marker.ns = "trajectory_markers";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            vector_marker.markers.push_back(marker);
        }
        stored_trajectory_ = vector_marker;
        file.close();
    }
        void publish_trajectory() {
        // Publish stored trajectory periodically
        RCLCPP_INFO(this->get_logger(), "Trajectory Publishing");
        publisher_->publish(stored_trajectory_);
    }


        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        visualization_msgs::msg::MarkerArray stored_trajectory_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajPublisher>());
  rclcpp::shutdown();
  return 0;
}