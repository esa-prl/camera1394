
#ifndef CAMERA1394_NODE_HPP
#define CAMERA1394_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>

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
    struct Camera1394Config
    {
        // Global Unique ID of camera, 16 hex digits (use first camera if null).
        // Default: ""
        std::string guid = "";

        // IIDC video mode.
        // Default: 640x480_mono8
        std::string video_mode = "640x480_mono8";

        // ROS tf frame of reference, resolved with tf_prefix unless absolute.
        // Default: camera
        std::string frame_id = "camera";

        // Camera speed (frames per second).
        // Default: 15.0 Min: 1.875 Max: 240.0
        double frame_rate = 15.0;

    } config_;
};
#endif
