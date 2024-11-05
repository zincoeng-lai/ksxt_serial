#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <serial/serial.h>
#include <vector>
#include <cstring>

#include <ksxt_parser.h>

#define NMEA0183_MESSAGE_HEADER_LENGTH 6

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ksxt_serial_node");
    ros::NodeHandle nh;
    ros::NodeHandle param_nh("~");

    // Read/set parameters
    std::string port;
    int baud;
    int refresh_rate_hz;
    param_nh.param("port", port, std::string("/dev/COM2"));
    param_nh.param("baud", baud, 115200);
    param_nh.param("refresh_rate_hz", refresh_rate_hz, 100);

    KSXTParser ksxt_parser(nh);
    // ros::Publisher nav_pub = nh.advertise<sensor_msgs::NavSatFix>("fix", 10);
    // ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("vel", 10);
    
    ros::Rate loop_rate(refresh_rate_hz);

    // serial settings
    serial::Serial *serial_port;
    try
    {
        serial_port = new serial::Serial(port, baud);
    }
    catch (...)
    {
        ROS_ERROR("Unable to initalize ksxt gnss serial port");
        return 1;
    }

    while (ros::ok())
    {
        // 发送请求
        // uint8_t request[] = {0xdd, 0xa5, 0x03, 0x00, 0xff, 0xfd, 0x77};
        // serial_port->write(request, sizeof(request));

        // 读取数据
        // std::vector<uint8_t> buffer(23);
        std::string buffer;
        size_t bytes_read = serial_port->read(buffer, buffer.size());

        ksxt_parser.decode(buffer);

        // if (bytes_read >= 24 && buffer[0] == 0xdd && buffer[1] == 0x03)
        // {
        //     // decode
        //     uint8_t rsoc = buffer[23];                       // 电量
        //     uint16_t voltage = (buffer[4] << 8) | buffer[5]; // 总电压
        //     int16_t current = (buffer[6] << 8) | buffer[7];  // 电流

        //     // publish battery msg
        //     sensor_msgs::NavSatFix ksxt_fix_msg;
        //     ksxt_fix_msg.header.stamp = ros::Time::now();
        //     ksxt_fix_msg.percentage = rsoc / 100.0; // 转换为百分比
        //     ksxt_fix_msg.voltage = voltage * 0.01;  // 转换为伏特
        //     ksxt_fix_msg.current = current * 0.01;  // 转换为安培

        //     nav_pub.publish(ksxt_fix_msg);
        // }

        ros::spinOnce();
        loop_rate.sleep();
    }

    delete serial_port;
    return 0;
}

// uint16_t calculateNMEA0183Checksum(const std::vector<uint8_t>& data) {
//     uint8_t u8_checksum = 0;

//     // XOR all bytes to calculate checksum
//     for (uint8_t byte : data) {
//         u8_checksum ^= byte;
//     }

//     // Convert the checksum to ASCII hex representation
//     uint8_t a = u8_checksum >> 4;   // High nibble
//     uint8_t b = u8_checksum & 0x0F; // Low nibble

//     a += (a > 0x09) ? ('A' - 0x0A) : '0';
//     b += (b > 0x09) ? ('A' - 0x0A) : '0';

//     // Pack as a 16-bit integer: low byte in higher order, high byte in lower order
//     return (b << 8) | a;
// }

// std::pair<std::vector<uint8_t>, int> decodeBuffer(std::vector<uint8_t>& buffer, int& errors) {
//     std::vector<uint8_t> payload;

//     while (buffer.size() >= NMEA0183_MESSAGE_HEADER_LENGTH) {
//         // Locate message header
//         if (buffer[0] != '$') {
//             buffer.erase(buffer.begin());
//             continue;
//         }

//         // Check if buffer has enough length for a full payload
//         auto asterisk_pos = std::find(buffer.begin(), buffer.end(), '*');
//         if (asterisk_pos == buffer.end()) break; // If '*' not found, wait for more data

//         size_t payload_length = std::distance(buffer.begin(), asterisk_pos) - 1;
//         size_t message_length = NMEA0183_MESSAGE_HEADER_LENGTH + payload_length;

//         if (buffer.size() < message_length) break; // Wait for more data

//         // Check XOR checksum
//         uint16_t expected_checksum = (buffer[payload_length + 2] | (buffer[payload_length + 3] << 8));
//         uint16_t calculated_checksum = calculateNMEA0183Checksum({buffer.begin() + 1, buffer.begin() + payload_length + 1});

//         if (expected_checksum != calculated_checksum) {
//             buffer.erase(buffer.begin());
//             errors++;
//             continue;
//         }

//         // Extract payload and remove processed message from buffer
//         payload = std::vector<uint8_t>(buffer.begin() + 1, buffer.begin() + payload_length + 1);
//         buffer.erase(buffer.begin(), buffer.begin() + message_length);
//         break;
//     }

//     return {payload, errors};
// }
