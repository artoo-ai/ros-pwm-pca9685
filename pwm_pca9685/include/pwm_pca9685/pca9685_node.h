#ifndef _pca9685_node_dot_h
#define _pca9685_node_dot_h

#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <std_msgs/msg/int32_multi_array.hpp>

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
    // XXX destructor should set the channels to timeout defaults

  private:
    void check_timeouts();

    void onCommand(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void set(uint8_t channel, uint16_t value);
    uint16_t sanitiseChannel(uint8_t channel, int val);

    rclcpp::Time last_update_time[16];  // the last time an event counted as an update to a channel
    rclcpp::Duration timeout_duration[16];  // the length of the timeout for each channel

    bool timeout_changes [16];  //true requires a channel command to change in order to recover the timeout
    bool timed_out [16]; //channel is timed out
    //uint16_t channel_offset[16];  //delay before cycle start (used to stagger pwm phases)
    uint16_t channel_value[16];  // the value currently written to the chip channel (only used by set())
    int32_t last_data[16];  //the last command received (not sanitised) changes indicate signs of life upstream

    bool reset();

    // class variables
    int file; // i2c device file handle

    // ROS parameters
    std::string param_device;
    int param_address;
    int param_frequency;
    std::vector<long int> param_timeout;
    std::vector<long int> param_pwm_min;
    std::vector<long int> param_pwm_max;
    std::vector<long int> param_timeout_value;
    //ros param arrays in eloquent only support char, bool, and long int
    // XXX ros2 supports parameter updates as callbacks

    // ROS timers
    rclcpp::TimerBase::SharedPtr timeout_cb_timer;

    // ROS publishers

    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_command;
    
    // ROS services
};

#endif // _pca9685_node_dot_h
