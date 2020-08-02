/* mixer_node.cpp
 * Author: Brett Downing <brettrd@brettrd.com>
 *
 * a simple mixer to map a teleop twist message to a PWM output
 * with clever use of the mixer matrix here and the limits on the pca9685 node, this can drive servo output.
 * composable nodes enabled
 */


#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>


using std::placeholders::_1;

// ******** constructors ******** //

class mixer_node : public rclcpp::Node
{
  public:
    mixer_node();

  private:
    const int n_channels = 16;
    void onCommand(const geometry_msgs::msg::Twist::SharedPtr msg);
    std::map<std::string, std::vector<double> > param_mixer;
    std::vector<bool> param_mask;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pwm_pub;

};



mixer_node::mixer_node() : 
    // node name
    Node("mixer_node"),
    //mixer geometry msg axis to pwm channel (padded to 16 channels with zeroes)
    //default is a simple two-motor pwm diff drive
    param_mixer({
        {"bias",    {   0,     0,      0,      0}},
        {"x",       {2048, -2048,   2048,  -2048}},
        {"y",       {   0,     0,      0,      0}},
        {"z",       {   0,     0,      0,      0}},
        {"yaw",     {2048, -2048,  -2048,   2048}},
        {"pitch",   {   0,     0,       0,      0}},
        {"roll",    {   0,     0,      0,      0}}
    }),
    //channel mask, padded to 16 channels with false
    param_mask({true, true, true, true})
{
    RCLCPP_INFO(this->get_logger(),"initializing");

    this->declare_parameters("mixer", param_mixer);
    this->declare_parameter("mask", param_mask);
    this->get_parameters("mixer", param_mixer);
    this->get_parameter("mask", param_mask);


    //sanitise the parameters
    std::vector<std::string> channel_names = {"bias", "x", "y", "z", "yaw", "pitch", "roll"};
    for( std::string chan : channel_names)
    {
        // insert a map key if it doesn't already exist
        param_mixer.insert({chan, {0}});
        // resize it to 16 channels
        param_mixer.at(chan).resize(n_channels, 0);
        param_mask.resize(n_channels, false);
        // XXX print the mixer and mask to ros info
    }

    RCLCPP_INFO(this->get_logger(),"starting");

    pwm_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("command", 10);

    twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, std::bind(&mixer_node::onCommand, this, _1));

}



void mixer_node::onCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    auto output = std::make_unique<std_msgs::msg::Int32MultiArray>();
    output->data.resize(n_channels);

    for(int i=0; i<n_channels; i++)
    {
        if(param_mask[i])
        {
            double accumulator = param_mixer.at("bias")[i];
            accumulator += param_mixer.at("x")[i] * msg->linear.x;
            accumulator += param_mixer.at("y")[i] * msg->linear.y;
            accumulator += param_mixer.at("z")[i] * msg->linear.z;
            accumulator += param_mixer.at("yaw")[i] * msg->angular.x;
            accumulator += param_mixer.at("pitch")[i] * msg->angular.y;
            accumulator += param_mixer.at("roll")[i] * msg->angular.z;

            if(accumulator < 0) accumulator = 0;    // pca9685_node uses negative values to handle channel masking, stop at zero

            output->data[i] = (int32_t) accumulator;
        }
        else
        {
            output->data[i] = -1;
        }
    }
    pwm_pub->publish(std::move(output));
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mixer_node>());
    rclcpp::shutdown();
    return 0;
}
