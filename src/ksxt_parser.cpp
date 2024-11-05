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

#include <ksxt_parser.h>

// class KSXTParser
// {
// public:
KSXTParser::KSXTParser(ros::NodeHandle &nh)
    : nav_pub(nh.advertise<sensor_msgs::NavSatFix>("fix", 10)),
      vel_pub(nh.advertise<geometry_msgs::TwistStamped>("vel", 10)) {}

void KSXTParser::decode(const std::string &buffer)
{
    size_t errors = 0;
    std::string packet_data;
    if (!decode_buffer(buffer, packet_data, errors))
        return;

    std::vector<std::string> fields = split_string(packet_data, ',');
    if (fields.size() != 23)
        return;

    if (fields[0].find("KSXT") != std::string::npos)
    {
        try
        {
            sensor_msgs::NavSatFix pos_msg;
            geometry_msgs::TwistStamped vel_msg;

            pos_msg.header.stamp = ros::Time::now();
            pos_msg.latitude = radians(std::stod(fields[3]));
            pos_msg.longitude = radians(std::stod(fields[2]));
            pos_msg.altitude = std::stod(fields[4]);

            double yaw = radians(std::stod(fields[5]));
            double pitch = radians(std::stod(fields[6]));
            double roll = radians(std::stod(fields[9]));

            pos_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
            if (std::stoi(fields[10]) == 2 || std::stoi(fields[10]) == 3)
            {
                pos_msg.position_covariance[0] = 5;
                pos_msg.position_covariance[4] = 5;
                pos_msg.position_covariance[8] = 5;
            }
            else
            {
                pos_msg.position_covariance[0] = 100;
                pos_msg.position_covariance[4] = 100;
                pos_msg.position_covariance[8] = 100;
            }

            double east_velocity = std::stod(fields[17]) / 3.6;
            double north_velocity = std::stod(fields[18]) / 3.6;

            vel_msg.twist.linear.x = north_velocity * cos(yaw) + east_velocity * sin(yaw);
            vel_msg.twist.linear.y = north_velocity * sin(yaw) + east_velocity * cos(yaw);
            vel_msg.twist.linear.z = std::stod(fields[19]) / 3.6;
            vel_msg.header.stamp = pos_msg.header.stamp;
            vel_msg.twist.angular.z = std::hypot(north_velocity, east_velocity);

            nav_pub.publish(pos_msg);
            vel_pub.publish(vel_msg);
        }
        catch (const std::invalid_argument &e)
        {
            ROS_WARN("Invalid value in NMEA sentence: %s", e.what());
        }
    }
}

// ros::Publisher nav_pub;
// ros::Publisher vel_pub;

bool KSXTParser::decode_buffer(const std::string &buffer, std::string &payload, size_t &errors)
{
    size_t header_pos = buffer.find('$');
    if (header_pos == std::string::npos)
        return false;

    size_t asterisk_pos = buffer.find('*', header_pos);
    if (asterisk_pos == std::string::npos)
        return false;

    payload = buffer.substr(header_pos + 1, asterisk_pos - header_pos - 1);
    uint16_t checksum = calculate_checksum(payload);

    if (buffer.size() > asterisk_pos + 2 &&
        checksum == std::stoi(buffer.substr(asterisk_pos + 1, 2), nullptr, 16))
    {
        return true;
    }
    else
    {
        errors++;
        return false;
    }
}

uint16_t KSXTParser::calculate_checksum(const std::string &data)
{
    uint8_t checksum = 0;
    for (char c : data)
    {
        checksum ^= static_cast<uint8_t>(c);
    }
    return checksum;
}

std::vector<std::string> KSXTParser::split_string(const std::string &str, char delimiter)
{
    std::vector<std::string> tokens;
    size_t start = 0, end = str.find(delimiter);
    while (end != std::string::npos)
    {
        tokens.push_back(str.substr(start, end - start));
        start = end + 1;
        end = str.find(delimiter, start);
    }
    tokens.push_back(str.substr(start));
    return tokens;
}

double KSXTParser::radians(double degrees)
{
    return degrees * M_PI / 180.0;
}

// }