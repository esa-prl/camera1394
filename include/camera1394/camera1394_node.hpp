
#ifndef CAMERA1394_NODE_HPP
#define CAMERA1394_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include "camera1394/camera1394_config.hpp"
#include "camera1394/driver1394.hpp"

class Camera1394Node : public rclcpp::Node
{
public:
    Camera1394Node();
    ~Camera1394Node();
    OnSetParametersCallbackHandle::SharedPtr callback_handler;

protected:
    template <typename T>
    void declare_and_get_parameter(
        rcl_interfaces::msg::ParameterDescriptor desc,
        T default_value,
        T &out_value,
        std::string log_info = std::string());

    void initialize_parameters();

    template <typename ConfigValueType>
    void change_parameter(
        const rclcpp::Parameter &param_changed,
        const rclcpp::ParameterType &expected_param_type,
        ConfigValueType &config_value,
        rcl_interfaces::msg::SetParametersResult &result);

    rcl_interfaces::msg::SetParametersResult param_change_callback(std::vector<rclcpp::Parameter> parameters);

private:
    camera1394::Camera1394Config config_;
    camera1394_driver::Camera1394Driver driver_;
};
#endif
