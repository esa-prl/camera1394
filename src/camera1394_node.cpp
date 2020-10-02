#include "camera1394/camera1394_node.hpp"

Camera1394Node::Camera1394Node()
    : Node("Camera1394Node")
{
    initialize_parameters();
    RCLCPP_INFO(this->get_logger(), "%s initialized", this->get_name());
}

Camera1394Node::~Camera1394Node()
{
}

/** Helper funcion to declare and get parameters
 * @param desc ParameterDescripor
 * @param default_value Default value of parameter if not changed
 * @param out_value Actual value of parameter
 */
template <typename T>
void Camera1394Node::declare_and_get_parameter(
    rcl_interfaces::msg::ParameterDescriptor desc,
    T default_value,
    T &out_value,
    std::string log_info)
{
    declare_parameter(desc.name, rclcpp::ParameterValue(default_value));

    if (!get_parameter(desc.name, out_value))
    {
        RCLCPP_WARN_STREAM(get_logger(), "The parameter '" << desc.name << "' is not available or is not valid, using the default value: " << default_value);
    }

    if (!log_info.empty())
    {
        RCLCPP_INFO_STREAM(get_logger(), log_info << out_value << (default_value == out_value ? " [default]" : " [changed]"));
    }
}

/** Declare all parameters and store the values in config */
void Camera1394Node::initialize_parameters()
{
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "general.guid";
        descriptor.description =
            "Global Unique ID of camera, 16 hex digits (use first camera if null";
        descriptor.additional_constraints =
            "";
        declare_and_get_parameter(descriptor, config_.guid, config_.guid, "* GUID: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "general.video_mode";
        descriptor.description =
            "IIDC video mode.";
        descriptor.additional_constraints =
            "";
        declare_and_get_parameter(descriptor, config_.video_mode, config_.video_mode, "* IIDC video mode: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "general.frame_id";
        descriptor.description =
            "ROS tf frame of reference, resolved with tf_prefix unless absolute.";
        descriptor.additional_constraints =
            "";
        declare_and_get_parameter(descriptor, config_.frame_id, config_.frame_id, "* Frame ID: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "general.frame_rate";
        descriptor.description =
            "Camera speed (frames per second)";
        descriptor.additional_constraints =
            "Min: 1.875 Max: 240.0";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = 1.875;
        floating_point_range.to_value = 240.0;

        declare_and_get_parameter(descriptor, config_.frame_rate, config_.frame_rate, "* Frame rate: ");
    }

    this->callback_handler = add_on_set_parameters_callback(std::bind(&Camera1394Node::param_change_callback, this, std::placeholders::_1));
}

/** Called everytime a parameter is changed
 * @param parameters Vector of changed parameters
 */
rcl_interfaces::msg::SetParametersResult Camera1394Node::param_change_callback(
    std::vector<rclcpp::Parameter> parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &parameter : parameters)
    {
        if (parameter.get_name() == "general.guid")
        {
            change_parameter(parameter, rclcpp::PARAMETER_STRING, config_.guid, result);
        }
        else if (parameter.get_name() == "general.video_mode")
        {
            change_parameter(parameter, rclcpp::PARAMETER_STRING, config_.video_mode, result);
        }
        else if (parameter.get_name() == "general.frame_id")
        {
            change_parameter(parameter, rclcpp::PARAMETER_STRING, config_.frame_id, result);
        }
        else if (parameter.get_name() == "general.frame_rate")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.frame_rate, result);
        }
    }
    return result;
}

/** Check changed parameters type and set config value
 * 
 * @param param_changed Changed parameter
 * @param expected_param_type Expected type of changed parameter
 * @param config_value Value of internal camera configuration thats about to be changed
 * @param result Result of SetParameterEvent
 */
template <typename ConfigValueType>
void Camera1394Node::change_parameter(
    const rclcpp::Parameter &param_changed,
    const rclcpp::ParameterType &expected_param_type,
    ConfigValueType &config_value,
    rcl_interfaces::msg::SetParametersResult &result)
{
    if (param_changed.get_type() != expected_param_type)
    {
        result.successful = false;
        result.reason = param_changed.get_name() + " must be a " + rclcpp::to_string(expected_param_type);
        return;
    }

    config_value = param_changed.get_value<ConfigValueType>();

    RCLCPP_INFO_STREAM(get_logger(), "Parameter '" << param_changed.get_name() << "' correctly set to " << config_value);
    result.successful = true;
    result.reason = param_changed.get_name() + " correctly set.";
    return;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Camera1394Node>());

    rclcpp::shutdown();
    return 0;
}
