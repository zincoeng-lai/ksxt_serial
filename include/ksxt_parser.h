#include <string>
#include <vector>
#include <cmath>
#include <ctime>
#include <stdexcept>
#include <iostream>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>

class KSXTParser
{
public:
    KSXTParser(ros::NodeHandle &nh);
    void decode(const std::string &buffer);

private:
    ros::Publisher nav_pub;
    ros::Publisher vel_pub;

    bool decode_buffer(const std::string &buffer, std::string &payload, size_t &errors);
    uint16_t calculate_checksum(const std::string &data);
    std::vector<std::string> split_string(const std::string &str, char delimiter);
    double radians(double degrees);
};
