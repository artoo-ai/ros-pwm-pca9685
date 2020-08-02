/* pca9685_node.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 * Author: Brett Downing <brettrd@brettrd.com>
 *
 * Defines a PCA9685 class, constructed with composable nodes
 */

#include <pwm_pca9685/pca9685_node.h>


using namespace std::chrono_literals;
using std::placeholders::_1;

// ******** constructors ******** //

PCA9685Node::PCA9685Node() : 
    // node name
    Node("PCA9685Node"),
    timeout_duration({
        0ms, 0ms, 0ms, 0ms, 0ms, 0ms, 0ms, 0ms,
        0ms, 0ms, 0ms, 0ms, 0ms, 0ms, 0ms, 0ms
    }),
    // linux i2c device file
    param_device("/dev/i2c-0"),
    // i2c accress of the pca9685
    param_address((int)PCA9685_ADDRESS),
    // pwm frequency
    param_frequency((int)1600),
    // timeouts in milliseconds per channel
    param_timeout({
        5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000,
        5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000
    }),
    // minimum pwm value per channel
    param_pwm_min({
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0
    }),
    // maximum pwm value per channel
    param_pwm_max({
        4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
        4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095
    }),
    // default pwm value per channel after timeout is reached
    param_timeout_value({
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0
    })
{
    RCLCPP_INFO(this->get_logger(),"initializing");

    this->declare_parameter("device", param_device);
    this->declare_parameter("address", param_address);
    this->declare_parameter("frequency", param_frequency);
    this->declare_parameter("timeout", param_timeout);
    this->declare_parameter("pwm_min", param_pwm_min);
    this->declare_parameter("pwm_max", param_pwm_max);
    this->declare_parameter("timeout_value", param_timeout_value);


    this->get_parameter("device", param_device);
    this->get_parameter("address", param_address);
    this->get_parameter("frequency", param_frequency);
    this->get_parameter("timeout", param_timeout);
    this->get_parameter("pwm_min", param_pwm_min);
    this->get_parameter("pwm_max", param_pwm_max);
    this->get_parameter("timeout_value", param_timeout_value);

    if(param_timeout.size() != 16) {
        
        RCLCPP_ERROR(this->get_logger(), "size of param timeout must be 16");
        rclcpp::shutdown();
    }

    if(param_timeout_value.size() != 16) {
        RCLCPP_ERROR(this->get_logger(), "size of param timeout_value must be 16");
        rclcpp::shutdown();
    }

    if(param_pwm_min.size() != 16) {
        RCLCPP_ERROR(this->get_logger(), "size of param pwm_min must be 16");
        rclcpp::shutdown();
    }

    if(param_pwm_max.size() != 16) {
        RCLCPP_ERROR(this->get_logger(), "size of param pwm_min must be 16");
        rclcpp::shutdown();
    }

    if(param_address < 0 || param_address > 127) {
        RCLCPP_ERROR(this->get_logger(), "param address must be between 0 and 127 inclusive");
        rclcpp::shutdown();
    }

    if(param_frequency <= 0) {
        RCLCPP_ERROR(this->get_logger(), "param frequency must be positive");
        rclcpp::shutdown();
    }


    RCLCPP_INFO(this->get_logger(),"starting");


    file = open(param_device.c_str(), O_RDWR);
    if(ioctl(file, I2C_SLAVE, param_address) < 0) {
        RCLCPP_ERROR(this->get_logger(), "i2c device open failed");
        rclcpp::shutdown();
    }

    if(!reset()) {
        RCLCPP_ERROR(this->get_logger(), "chip reset and setup failed");
        rclcpp::shutdown();
    }
    

    auto t = this->now();
    for(int channel = 0; channel < 16; channel++) {
        last_update_time[channel] = t;
        timeout_duration[channel] = std::chrono::milliseconds(param_timeout[channel]);
        timed_out[channel] = true;  //default all channels to a designated safe value
        timeout_changes[channel] = param_timeout[channel] < 0;  //timeout resets after command >0 or after change <0
        last_data[channel] = param_timeout_value[channel];
        set(channel, sanitiseChannel(channel, param_timeout_value[channel]));
    }


    timeout_cb_timer = create_wall_timer(100ms, std::bind(&PCA9685Node::check_timeouts, this));

    if(!sub_command)
    {
        sub_command = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "command", 1, std::bind(&PCA9685Node::onCommand, this, _1));
    }
}

// ******** private methods ******** //

bool PCA9685Node::reset()
{
    // set frequency
    uint8_t prescale = (uint8_t)(25000000.0 / 4096.0 / param_frequency + 0.5);

    // XXX need to wait beween writes, wait on clock, not ros (sim) time

    _i2c_smbus_write_byte_data(file, PCA9685_MODE1_REG, 0b10000000); // reset

    _i2c_smbus_write_byte_data(file, PCA9685_MODE1_REG, 0b00010000); // sleep
    _i2c_smbus_write_byte_data(file, PCA9685_PRESCALE_REG, prescale); // set prescale
    _i2c_smbus_write_byte_data(file, PCA9685_MODE2_REG, 0x04); // outdrv
    _i2c_smbus_write_byte_data(file, PCA9685_MODE1_REG, 0xA1); // un-sleep

    _i2c_smbus_write_byte_data(file, PCA9685_CHANNEL_ALL_REG, 0);
    _i2c_smbus_write_byte_data(file, PCA9685_CHANNEL_ALL_REG + 1, 0);
    _i2c_smbus_write_byte_data(file, PCA9685_CHANNEL_ALL_REG + 2, 0);
    _i2c_smbus_write_byte_data(file, PCA9685_CHANNEL_ALL_REG + 3, 0);

    return true;
}

void PCA9685Node::set(uint8_t channel, uint16_t value)
{
    if(channel_value[channel] != value)
    {
        uint8_t values[4];
        if(value == 0x0FFF) { // always on
            values[0] = 0x00;
            values[1] = 0x10;
            values[2] = 0x00;
            values[3] = 0x00;
        } else if(value == 0x0000) { // always off
            values[0] = 0x00;
            values[1] = 0x00;
            values[2] = 0x00;
            values[3] = 0x10;
        } else { // PWM
            // XXX add phase offset
            values[0] = 0x00;
            values[1] = 0x00;
            values[2] = (value) & 0xFF;
            values[3] = ((value) >> 8) & 0x0F;
        }

        _i2c_smbus_write_i2c_block_data(file, PCA9685_CHANNEL_0_REG + (channel * 4), 4, values);
        channel_value[channel] = value;
    }
}

// ******** public methods ******** //

void PCA9685Node::check_timeouts()
{
    auto t = this->now();

    for(int channel = 0; channel < 16; channel++) {
        if(!timed_out[channel])
        {
            if(t - last_update_time[channel] > timeout_duration[channel])
            {
                set(channel, param_timeout_value[channel]);
                RCLCPP_WARN(this->get_logger(), "channel %d timeout", channel);
                timed_out[channel] = true;
            }
        }
    }
}


void PCA9685Node::onCommand(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    auto t = this->now();

    if(msg->data.size() != 16) {
        RCLCPP_ERROR(this->get_logger(), "array does not have a size of 16");
        return;
    }

    for(int channel = 0; channel < 16; channel++) {
        if(msg->data[channel] >= 0) // ignore negative channels
        {
            bool command_changed = (last_data[channel] != msg->data[channel]);
            last_data[channel] = msg->data[channel];


            // check if channels should recover from timeout
            if(timed_out[channel])
            {
                if( (!timeout_changes[channel]) || command_changed)
                {
                    RCLCPP_WARN(this->get_logger(), "channel %d recovered from timeout", channel);
                    timed_out[channel] = false;
                }
            }


            if( !timed_out[channel] )
            {
                set(channel, sanitiseChannel(channel, msg->data[channel]));
                // does this constitute a meaningful update for timeout purposes?
                if( (!timeout_changes[channel]) || command_changed)
                {
                    last_update_time[channel] = t;
                }
            }
        }
    }
}
uint16_t PCA9685Node::sanitiseChannel(uint8_t channel, int val)
{
    if(val > param_pwm_max[channel]) {
        val = param_pwm_max[channel];
    }
    if(val < param_pwm_min[channel]) {
        val = param_pwm_min[channel];
    }
    return (uint16_t) val;
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCA9685Node>());
    rclcpp::shutdown();
    return 0;
}
