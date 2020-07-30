/* pca9685_i2c_node.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Instantiates a PCA9685 Node
 */

#include <pwm_pca9685/pca9685_node.h>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pwm_pca9685>());
    rclcpp::shutdown();
    return 0;
}
