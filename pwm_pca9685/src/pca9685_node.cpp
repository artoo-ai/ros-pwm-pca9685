/* pca9685_activity.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Defines a PCA9685 class, constructed with node handles
 * and which handles all ROS duties.
 */

#include <pwm_pca9685/pca9685_node.h>


using namespace std::chrono_literals;
using std::placeholders::_1;

// ******** constructors ******** //

PCA9685Node::PCA9685Node() : 
    // node name
    Node("PCA9685Node"),
    // linux i2c device file
    param_device("/dev/i2c-1"),
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
        65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535,
        65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535
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

    auto t = this->now();

    for(int channel = 0; channel < 16; channel++) {
        last_set_times[channel] = t;
        last_change_times[channel] = t;
        //timeout[channel] = std::chrono::milliseconds(param_timeout[channel]);
        last_data[channel] = 0;
    }


    RCLCPP_INFO(this->get_logger(),"starting");

    if(!sub_command)
    {
        sub_command = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "command", 1, std::bind(&PCA9685Node::onCommand, this, _1));
    }

    file = open(param_device.c_str(), O_RDWR);
    if(ioctl(file, I2C_SLAVE, param_address) < 0) {
        RCLCPP_ERROR(this->get_logger(), "i2c device open failed");
        rclcpp::shutdown();
    }

    if(!reset()) {
        RCLCPP_ERROR(this->get_logger(), "chip reset and setup failed");
        rclcpp::shutdown();
    }
    
    timeout_timer = create_wall_timer(100ms, std::bind(&PCA9685Node::check_timeouts, this));
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
    uint16_t value_12bit = value >> 4;
    uint8_t values[4];
    if(value_12bit == 0x0FFF) { // always on
        values[0] = 0x00;
        values[1] = 0x10;
        values[2] = 0x00;
        values[3] = 0x00;
    } else if(value_12bit == 0x0000) { // always off
        values[0] = 0x00;
        values[1] = 0x00;
        values[2] = 0x00;
        values[3] = 0x00;
    } else { // PWM
        values[0] = 0x00;
        values[1] = 0x00;
        values[2] = (value_12bit + 1) & 0xFF;
        values[3] = ((value_12bit + 1) >> 8) & 0x0F;
    }

    _i2c_smbus_write_i2c_block_data(file, PCA9685_CHANNEL_0_REG + (channel * 4), 4, values);
}

// ******** public methods ******** //

void PCA9685Node::check_timeouts()
{
    auto t = this->now();

    //rclcpp::Duration = 

    for(int channel = 0; channel < 16; channel++) {
        // positive timeout: timeout when no command is received
        if(param_timeout[channel] > 0 && t - last_set_times[channel] > std::abs(param_timeout[channel])) {
            set(channel, param_timeout_value[channel]);
            last_data[channel]  = param_timeout_value[channel]; // last data needs to be updated beacuse the channels are only changed if the value changes
            //ROS_WARN_STREAM("timeout " << channel);
        }
        // negative timeout: timeout when value doesn't change
        else if(param_timeout[channel] < 0 && t - last_change_times[channel] > std::abs(param_timeout[channel])) {
            set(channel, param_timeout_value[channel]);
            //last_data[channel]  = param_timeout_value[channel];   // if last_data is updated, the channel will exit timeout on the next unchanged command
            //ROS_WARN_STREAM("timeout " << channel);
        }
    }
}


void PCA9685Node::onCommand(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    auto t = this->now();

    if(msg->data.size() != 16) {
        RCLCPP_ERROR(this->get_logger(), "array is not have a size of 16");
        return;
    }

    for(int channel = 0; channel < 16; channel++) {
        int val = msg->data[channel];

        if(val >= 0) {
            last_set_times[channel] = t;

            if(val > param_pwm_max[channel]) {
                val = param_pwm_max[channel];
            }
            if(val < param_pwm_min[channel]) {
                val = param_pwm_min[channel];
            }
            // XXX this would make more sense with more explicit state variables instead of relying on implicit relationships
            if(val != last_data[channel]){
                set(channel, val);
                last_data[channel] = val;
                last_change_times[channel] = t;
            }
        }
    }
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCA9685Node>());
    rclcpp::shutdown();
    return 0;
}
