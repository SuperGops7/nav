#ifndef TRAJECTORY_SAVER_HPP
#define TRAJECTORY_SAVER_HPP

#include <string>
#include <vector>
#include <nav_msgs/msg/odometry.hpp>

class TrajectorySaver
{
public:
    TrajectorySaver();
    ~TrajectorySaver();

    // Provide the trajectory data to save
    void setTrajectoryData(const std::vector<nav_msgs::msg::Odometry>& data);

    // Save the trajectory data into the given file formats (base filename will be appended with extensions)
    bool saveToJson(const std::string &filename);
    bool saveToCsv(const std::string &filename);
    bool saveToYaml(const std::string &filename);

private:
    std::vector<nav_msgs::msg::Odometry> trajectory_data_;
};

#endif // TRAJECTORY_SAVER_HPP