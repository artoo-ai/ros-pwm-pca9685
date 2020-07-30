#ifndef _pca9685_node_dot_h
#define _pca9685_node_dot_h

#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <std_msgs/msg/Int32MultiArray.h>

#include <linux/i2c-dev.h>
#include <smbus_functions.h>

#define PCA9685_ADDRESS 0x40

#define PCA9685_MODE1_REG 0X00
#define PCA9685_MODE2_REG 0X01
#define PCA9685_SUBADR_REG 0X02
#define PCA9685_ALLCALLADR_REG 0X05
#define PCA9685_CHANNEL_0_REG 0x06
#define PCA9685_CHANNEL_1_REG 0x0A
#define PCA9685_CHANNEL_2_REG 0x0E
#define PCA9685_CHANNEL_3_REG 0x12
#define PCA9685_CHANNEL_4_REG 0x16
#define PCA9685_CHANNEL_5_REG 0x1A
#define PCA9685_CHANNEL_6_REG 0x1E
#define PCA9685_CHANNEL_7_REG 0x22
#define PCA9685_CHANNEL_8_REG 0x26
#define PCA9685_CHANNEL_9_REG 0x2A
#define PCA9685_CHANNEL_10_REG 0x2E
#define PCA9685_CHANNEL_11_REG 0x32
#define PCA9685_CHANNEL_12_REG 0x36
#define PCA9685_CHANNEL_13_REG 0x3A
#define PCA9685_CHANNEL_14_REG 0x3E
#define PCA9685_CHANNEL_15_REG 0x42
#define PCA9685_CHANNEL_ALL_REG 0xFA
#define PCA9685_PRESCALE_REG 0xFE
#define PCA9685_TESTMODE_REG 0xFF


class PCA9685Node : public rclcpp::Node
{
  public:
    PCA9685Node();

    bool start();
    bool stop();
    bool spinOnce();

    void onCommand(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    bool set(uint8_t channel, uint16_t value);

    uint64_t last_set_times[16];
    uint64_t last_change_times[16];
    int last_data[16];

  private:
    bool reset();

    // class variables
    uint32_t seq = 0;
    int file;

    // ROS parameters
    std::string param_device;
    int param_address;
    int param_frequency;
    std::vector<int> param_timeout;
    std::vector<int> param_timeout_value;
    std::vector<int> param_pwm_min;
    std::vector<int> param_pwm_max;

    // ROS timers
    rclcpp::TimerBase::SharedPtr timeout_timer;

    // ROS publishers

    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::Int32MultiArrayPtr>::SharedPtr sub_command;
    
    // ROS services
};

#endif // _pca9685_node_dot_h
