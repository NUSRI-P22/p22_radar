#include "rclcpp/rclcpp.hpp"
#include "radar_msgs/msg/radar_track.hpp"
#include "serial/serial.h"
#include <iostream>
#include <string>
#include <vector>
#include <algorithm> // 包含算法库

bool serialopen = 0; //when serial is open, change it to 1, otherwise, change it to 0;
std::string data;
std::string predata;
double x_coord, y_coord;
std::vector<int> x;
std::vector<int> y;

void parse_one_frame(const std::string& data) {
    


    if (data.compare(0, 6, "ffeedd") != 0) {
        std::cout << "数据字符串不以 'ffeedd' 开头。跳过解析。" << std::endl;
        return;
    }
    std::string data_length_str = data.substr(6, 4);
    int data_length = std::stoi(data_length_str, nullptr, 16);
    
    if (12 + 2 * (data_length - 1) + 2 + 6 != data.size()) {
        std::cout << "数据字符串长度不匹配。 跳过解析。" << std::endl;
        return;
    }
    
    if (data.compare(12 + 2 * (data_length - 1) + 2, 6, "ddeeff") != 0) {
        std::cout << "数据字符串不以 'ddeeff' 结尾。跳过解析。" << std::endl;
        return;
    }
    
    
    std::string data_frame = data.substr(12, 2 * (data_length - 1));

    // Split data_frame into pairs of two characters
    std::vector<std::string> data_list;
    
    

    for (size_t i = 0; i < data_frame.size(); i += 2) {
        data_list.push_back(data_frame.substr(i, 2));
    }

    // Process each pair of data_list
    for (size_t i = 0; i < data_list.size(); i += 2) {
        int x = std::stoi(data_list[i], nullptr, 16);
        if (x > 127) {
            x -= 256;
        }
        x_coord = static_cast<double>(x) / 10.0;


        int y = std::stoi(data_list[i + 1], nullptr, 16);
        if (y > 127) {
            y -= 256;
        }
        y_coord = static_cast<double>(y) / 10.0;
        
    }

    
    
    
}

std::string data_to_hexstring(const std::string& data) {
    std::string result;
    result.reserve(data.length() * 2);

    for (char c : data) {
        char high_nibble = (c >> 4) & 0xf;
        char low_nibble = c & 0xf;

        // Convert to hexadecimal characters
        high_nibble = (high_nibble > 9) ? high_nibble + 'a' - 10 : high_nibble + '0';
        low_nibble = (low_nibble > 9) ? low_nibble + 'a' - 10 : low_nibble + '0';

        result.push_back(high_nibble);
        result.push_back(low_nibble);
    }

    return result;
}



class RadarNode : public rclcpp::Node
{
public:

    // Replace "/dev/ttyUSB0" with the actual serial port name
    RadarNode() : Node("p22_radar"), serial_port_("/dev/ttyUSB0", 9600, serial::Timeout::simpleTimeout(100))
    {
        publisher_ = this->create_publisher<radar_msgs::msg::RadarTrack>("radar/detected_objects", 1000/period_ms);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&RadarNode::serialCallback, this));
    }
    
private:
   
    
    
    void serialCallback()
    {
        if (serial_port_.available() > 0)
        {
            radar_msgs::msg::RadarTrack msg;
            predata = serial_port_.readline();
            data = data_to_hexstring(predata);
            std::cout << data << std::endl;
            parse_one_frame(data);

            msg.position.x = x_coord; 
            msg.position.y = y_coord;
            
            
            
            publisher_->publish(msg);
        }
    }
    serial::Serial serial_port_;
    rclcpp::Publisher<radar_msgs::msg::RadarTrack>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int period_ms = 50; // pubilish period
};

class emptyRadarNode : public rclcpp::Node
{
public:

    emptyRadarNode() : Node("p22_radar")
    {
        publisher_ = this->create_publisher<radar_msgs::msg::RadarTrack>("radar/detected_objects", 1000/period_ms);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&emptyRadarNode::publishEmptyMessage, this));
    }
    
private:
   
    
    
    void publishEmptyMessage()
    {
    
        radar_msgs::msg::RadarTrack empty_radar_msg;

        empty_radar_msg.position.x = 0; 
        empty_radar_msg.position.y = 0;
        
        
        
        publisher_->publish(empty_radar_msg);
        
    }

    serial::Serial serial_port_;
    rclcpp::Publisher<radar_msgs::msg::RadarTrack>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int period_ms = 50; // pubilish period
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if (serialopen){
        auto node = std::make_shared<RadarNode>();
        rclcpp::spin(node);
    } else {
        auto emptynode = std::make_shared<emptyRadarNode>();
        rclcpp::spin(emptynode);
    }
    rclcpp::shutdown();
    return 0;
}
