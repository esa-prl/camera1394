#include "camera1394/camera1394_node.hpp"

Camera1394Node::Camera1394Node()
    : Node("Camera1394Node"),
      driver_(this)
{

    initialize_parameters();

    RCLCPP_INFO(this->get_logger(), "%s initialized", this->get_name());
}

Camera1394Node::~Camera1394Node()
{
}

/** Helper function to declare and get parameters
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
    declare_parameter(desc.name, rclcpp::ParameterValue(default_value), desc);

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
    RCLCPP_INFO(get_logger(), "General parameters: ");
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "guid";
        descriptor.description =
            "Global Unique ID of camera, 16 hex digits (use first camera if null).\n  Default: \"\" (null)";
        declare_and_get_parameter(descriptor, config_.guid, config_.guid, "* GUID: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "video_mode";
        descriptor.description =
            "IIDC video mode.\n  Default: 640x480_mono8";
        declare_and_get_parameter(descriptor, config_.video_mode, config_.video_mode, "* IIDC video mode: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "frame_id";
        descriptor.description =
            "ROS tf frame of reference, resolved with tf_prefix unless absolute.\n  Default: camera";
        descriptor.additional_constraints =
            "";
        declare_and_get_parameter(descriptor, config_.frame_id, config_.frame_id, "* Frame ID: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "frame_rate";
        descriptor.description =
            "Camera speed (frames per second). \n  Default: 15.0";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = 1.875;
        floating_point_range.to_value = 240.0;

        declare_and_get_parameter(descriptor, config_.frame_rate, config_.frame_rate, "* Frame rate: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "max_consecutive_errors";
        descriptor.description =
            "Max number of consecutive read errors before attempting reconnection (0 to disable). \n  Default: 0";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 1000;

        declare_and_get_parameter(descriptor, config_.max_consecutive_errors, config_.max_consecutive_errors, "* Max consecutive errors: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "iso_speed";
        descriptor.description =
            "Total IEEE 1394 bus bandwidth (Megabits/second). \n  Default: 400";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 100;
        integer_range.to_value = 3200;

        declare_and_get_parameter(descriptor, config_.iso_speed, config_.iso_speed, "* ISO speed: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "num_dma_buffers";
        descriptor.description =
            "Number of frames in the DMA ring buffer. \n  Default: 4";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 1;
        integer_range.to_value = 1024;

        declare_and_get_parameter(descriptor, config_.num_dma_buffers, config_.num_dma_buffers, "* Number of DMA buffers: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "camera_info_url";
        descriptor.description =
            "Camera [[camera_info_manager#URL_Names|calibration URL]] for this video_mode (uncalibrated if null). \n  Default: \"\" (null)";
        descriptor.additional_constraints =
            "";
        declare_and_get_parameter(descriptor, config_.camera_info_url, config_.camera_info_url, "* Camera info URL: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "reset_on_open";
        descriptor.description =
            "Reset camera when opening the device. \n  Default: False";
        descriptor.additional_constraints =
            "";
        declare_and_get_parameter(descriptor, config_.reset_on_open, config_.reset_on_open, "* Reset camera when opened: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "use_ros_time";
        descriptor.description =
            "Timestamp for Image and CameraInfo using rclcpp::Clock::now() \n  Default: False";
        descriptor.additional_constraints =
            "";
        declare_and_get_parameter(descriptor, config_.use_ros_time, config_.use_ros_time, "* Use ROS time: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "time_offset";
        descriptor.description =
            "Offset in seconds to add to rclcpp::Clock::now(). \n  Default: 0.0";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = -5.0;
        floating_point_range.to_value = 1.0;

        declare_and_get_parameter(descriptor, config_.time_offset, config_.time_offset, "* Time offset: ");
    }

    RCLCPP_INFO(get_logger(), "Format7 specific parameters: ");
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "binning_x";
        descriptor.description =
            "Number of pixels combined for Format7 horizontal binning, use device hints if zero. \n  Default: 0";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 4;

        declare_and_get_parameter(descriptor, config_.binning_x, config_.binning_x, "* Horizontal binning: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "binning_y";
        descriptor.description =
            "Number of pixels combined for Format7 vertical binning, use device hints if zero. \n  Default: 0";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 4;

        declare_and_get_parameter(descriptor, config_.binning_y, config_.binning_y, "* Vertical binning: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "roi_width";
        descriptor.description =
            "Width of Format7 Region of Interest in unbinned pixels, full width if zero. \n  Default: 0";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 65535;

        declare_and_get_parameter(descriptor, config_.roi_width, config_.roi_width, "* ROI width: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "roi_height";
        descriptor.description =
            "Height of Format7 Region of Interest in unbinned pixels, full height if zero. \n  Default: 0";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 65535;

        declare_and_get_parameter(descriptor, config_.roi_height, config_.roi_height, "* ROI height: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "x_offset";
        descriptor.description =
            "Horizontal offset for left side of Format7 ROI in unbinned pixels. \n  Default: 0";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 65535;

        declare_and_get_parameter(descriptor, config_.x_offset, config_.x_offset, "* X offset: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "y_offset";
        descriptor.description =
            "Vertical offset for top side of Format7 ROI in unbinned pixels. \n  Default: 0";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 65535;

        declare_and_get_parameter(descriptor, config_.y_offset, config_.y_offset, "* Y offset: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "format7_packet_size";
        descriptor.description =
            "Format7 packet size (bytes), device-recommended size if zero. \n  Default: 0";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 39320;

        declare_and_get_parameter(descriptor, config_.format7_packet_size, config_.format7_packet_size, "* Format7 packet size: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "format7_color_coding";
        descriptor.description =
            "Color coding (only for Format7 modes). \n  Default: mono8";
        descriptor.additional_constraints =
            "mono8, mono16, mono16s, raw8, raw16, rgb8, rgb16, rgb16s, yuv411, yuv422, yuv444";
        declare_and_get_parameter(descriptor, config_.format7_color_coding, config_.format7_color_coding, "* Format7 color coding: ");
    }

    RCLCPP_INFO(get_logger(), "Bayer color filter parameters: ");
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "bayer_pattern";
        descriptor.description =
            "Bayer color encoding pattern. \n  Default: \"\"";
        descriptor.additional_constraints =
            "\"\", rggb, gbrg, grbg, bggr";
        declare_and_get_parameter(descriptor, config_.bayer_pattern, config_.bayer_pattern, "* Bayer color coding: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "bayer_method";
        descriptor.description =
            "Bayer decoding method \n  Default: \"\" (Decode via ROS image_proc)";
        descriptor.additional_constraints =
            "\"\", DownSample, Simple, Bilinear, HQ, VNG, AHD";
        declare_and_get_parameter(descriptor, config_.bayer_method, config_.bayer_method, "* Bayer decoding method: ");
    }

    RCLCPP_INFO(get_logger(), "Generic IIDC parameters: ");
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "auto_brightness";
        descriptor.description =
            "Brightness control state. \n  Default: 1 (Query)";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 4;
        descriptor.additional_constraints =
            "0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)";

        declare_and_get_parameter(descriptor, config_.auto_brightness, config_.auto_brightness, "* Brightness control state: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "brightness";
        descriptor.description =
            "Black level offset. \n  Default: 0.0";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = 0.0;
        floating_point_range.to_value = 4095.0;

        declare_and_get_parameter(descriptor, config_.brightness, config_.brightness, "* Brightness: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "auto_exposure";
        descriptor.description =
            "Combined Gain, Iris & Shutter control. \n  Default: 1 (Query)";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 4;
        descriptor.additional_constraints =
            "0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)";

        declare_and_get_parameter(descriptor, config_.auto_exposure, config_.auto_exposure, "* Exposure control state: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "exposure";
        descriptor.description =
            "Auto exposure value (like contrast). \n  Default: 0.0";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = -10.0;
        floating_point_range.to_value = 4095.0;

        declare_and_get_parameter(descriptor, config_.exposure, config_.exposure, "* Exposure: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "auto_focus";
        descriptor.description =
            "Focus control state. \n  Default: 1 (Query)";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 4;
        descriptor.additional_constraints =
            "0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)";

        declare_and_get_parameter(descriptor, config_.auto_focus, config_.auto_focus, "* Focus control state: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "focus";
        descriptor.description =
            "Focus control. \n  Default: 0.0";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = 0.0;
        floating_point_range.to_value = 4095.0;

        declare_and_get_parameter(descriptor, config_.focus, config_.focus, "* Focus: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "auto_gain";
        descriptor.description =
            "Gain control state. \n  Default: 1 (Query)";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 4;
        descriptor.additional_constraints =
            "0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)";

        declare_and_get_parameter(descriptor, config_.auto_gain, config_.auto_gain, "* Gain control state: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "gain";
        descriptor.description =
            "Relative circuit gain. \n  Default: 0.0";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = -10.0;
        floating_point_range.to_value = 4095.0;

        declare_and_get_parameter(descriptor, config_.gain, config_.gain, "* Gain: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "auto_gamma";
        descriptor.description =
            "Gamma control state. \n  Default: 1 (Query)";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 4;
        descriptor.additional_constraints =
            "0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)";

        declare_and_get_parameter(descriptor, config_.auto_gamma, config_.auto_gamma, "* Gamma control state: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "gamma";
        descriptor.description =
            "Gamma expansion exponent. \n  Default: 2.2";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = 0.0;
        floating_point_range.to_value = 10.0;

        declare_and_get_parameter(descriptor, config_.gamma, config_.gamma, "* Gamma: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "auto_hue";
        descriptor.description =
            "Hue control state. \n  Default: 1 (Query)";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 4;
        descriptor.additional_constraints =
            "0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)";

        declare_and_get_parameter(descriptor, config_.auto_hue, config_.auto_hue, "* Hue control state: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "hue";
        descriptor.description =
            "Color phase. \n  Default: 0.0";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = 0.0;
        floating_point_range.to_value = 4095.0;

        declare_and_get_parameter(descriptor, config_.hue, config_.hue, "* Hue: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "auto_iris";
        descriptor.description =
            "Iris control state. \n  Default: 1 (Query)";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 4;
        descriptor.additional_constraints =
            "0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)";

        declare_and_get_parameter(descriptor, config_.auto_iris, config_.auto_iris, "* Iris control state: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "iris";
        descriptor.description =
            "Iris control. \n  Default: 8.0";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = 0.0;
        floating_point_range.to_value = 4095.0;

        declare_and_get_parameter(descriptor, config_.iris, config_.iris, "* Iris: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "auto_pan";
        descriptor.description =
            "Pan control state. \n  Default: 1 (Query)";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 4;
        descriptor.additional_constraints =
            "0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)";

        declare_and_get_parameter(descriptor, config_.auto_pan, config_.auto_pan, "* Pan control state: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "pan";
        descriptor.description =
            "Pan control. \n  Default: 0.0";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = 0.0;
        floating_point_range.to_value = 4095.0;

        declare_and_get_parameter(descriptor, config_.pan, config_.pan, "* Pan: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "auto_saturation";
        descriptor.description =
            "Saturation control state. \n  Default: 1 (Query)";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 4;
        descriptor.additional_constraints =
            "0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)";

        declare_and_get_parameter(descriptor, config_.auto_saturation, config_.auto_saturation, "* Saturation control state: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "saturation";
        descriptor.description =
            "Color saturation. \n  Default: 1.0";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = 0.0;
        floating_point_range.to_value = 4095.0;

        declare_and_get_parameter(descriptor, config_.saturation, config_.saturation, "* Saturation: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "auto_sharpness";
        descriptor.description =
            "Sharpness control state. \n  Default: 1 (Query)";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 4;
        descriptor.additional_constraints =
            "0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)";

        declare_and_get_parameter(descriptor, config_.auto_sharpness, config_.auto_sharpness, "* Sharpness control state: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "sharpness";
        descriptor.description =
            "Image sharpness. \n  Default: 1.0";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = 0.0;
        floating_point_range.to_value = 4095.0;

        declare_and_get_parameter(descriptor, config_.sharpness, config_.sharpness, "* Sharpness: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "auto_shutter";
        descriptor.description =
            "Shutter control state. \n  Default: 1 (Query)";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 4;
        descriptor.additional_constraints =
            "0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)";

        declare_and_get_parameter(descriptor, config_.auto_shutter, config_.auto_shutter, "* Shutter control state: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "shutter";
        descriptor.description =
            "Shutter speed. \n  Default: 1.0";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = 0.0;
        floating_point_range.to_value = 4095.0;

        declare_and_get_parameter(descriptor, config_.shutter, config_.shutter, "* Shutter speed: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "external_trigger";
        descriptor.description =
            "External trigger power state. \n  Default: False";
        descriptor.additional_constraints =
            "";
        declare_and_get_parameter(descriptor, config_.external_trigger, config_.external_trigger, "* External trigger power state: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "software_trigger";
        descriptor.description =
            "Software trigger power state. \n  Default: False";
        descriptor.additional_constraints =
            "";
        declare_and_get_parameter(descriptor, config_.software_trigger, config_.software_trigger, "* Software trigger power state: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "trigger_mode";
        descriptor.description =
            "External trigger mode. \n  Default: mode_0";
        descriptor.additional_constraints =
            "\n"
            "    mode_0 - Exposure starts with a falling edge and stops when the the exposure specified by the SHUTTER feature is elapsed \n"
            "    mode_1 - Exposure starts with a falling edge and stops with the next rising edge \n"
            "    mode_2 - The camera starts the exposure at the first falling edge and stops the integration at the nth falling edge \n"
            "    mode_3 - This is an internal trigger mode. The trigger is generated every n*(period of fastest framerate) \n"
            "    mode_4 - A multiple exposure mode. N exposures are performed each time a falling edge is observed on the trigger signal. Each exposure is as long as defined by the SHUTTER feature \n"
            "    mode_5 - Same as Mode 4 except that the exposure is is defined by the length of the trigger pulse instead of the SHUTTER feature \n"
            "    mode_14 -- vendor specified trigger mode \n"
            "    mode_15 -- vendor specified trigger mode \n";
        declare_and_get_parameter(descriptor, config_.trigger_mode, config_.trigger_mode, "* External trigger mode: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "trigger_source";
        descriptor.description =
            "External trigger source. \n  Default: source_0";
        descriptor.additional_constraints =
            "source_0, source_1, source_2, source_3, source_software";
        declare_and_get_parameter(descriptor, config_.trigger_source, config_.trigger_source, "* External trigger source: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "trigger_polarity";
        descriptor.description =
            "Trigger polarity. \n  Default: active_high";
        descriptor.additional_constraints =
            "acitve_low, active_high";
        declare_and_get_parameter(descriptor, config_.trigger_polarity, config_.trigger_polarity, "* Trigger polarity: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "auto_trigger";
        descriptor.description =
            "Trigger control state. \n  Default: 1 (Query)";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 4;
        descriptor.additional_constraints =
            "0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)";

        declare_and_get_parameter(descriptor, config_.auto_trigger, config_.auto_trigger, "* Trigger control state: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "trigger";
        descriptor.description =
            "Trigger parameter N. \n  Default: 0.0";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = 0.0;
        floating_point_range.to_value = 4095.0;

        declare_and_get_parameter(descriptor, config_.trigger, config_.trigger, "* Trigger parameter N: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "auto_white_balance";
        descriptor.description =
            "White balance control state. \n  Default: 1 (Query)";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 4;
        descriptor.additional_constraints =
            "0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)";

        declare_and_get_parameter(descriptor, config_.auto_white_balance, config_.auto_white_balance, "* White balance control state: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "white_balance_BU";
        descriptor.description =
            "Blue or U component of white balance. \n  Default: 0.0";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = 0.0;
        floating_point_range.to_value = 4095.0;

        declare_and_get_parameter(descriptor, config_.white_balance_BU, config_.white_balance_BU, "* White balance BU: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "white_balance_RV";
        descriptor.description =
            "Red or V component of white balance. \n  Default: 0.0";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = 0.0;
        floating_point_range.to_value = 4095.0;

        declare_and_get_parameter(descriptor, config_.white_balance_RV, config_.white_balance_RV, "* White balance RV: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "auto_zoom";
        descriptor.description =
            "Zoom control state. \n  Default: 1 (Query)";
        descriptor.integer_range.resize(1);
        auto &integer_range = descriptor.integer_range.at(0);
        integer_range.from_value = 0;
        integer_range.to_value = 4;
        descriptor.additional_constraints =
            "0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)";

        declare_and_get_parameter(descriptor, config_.auto_zoom, config_.auto_zoom, "* Zoom control state: ");
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.name =
            "zoom";
        descriptor.description =
            "Zoom control. \n  Default: 0.0";
        descriptor.floating_point_range.resize(1);
        auto &floating_point_range = descriptor.floating_point_range.at(0);
        floating_point_range.from_value = 0.0;
        floating_point_range.to_value = 4095.0;

        declare_and_get_parameter(descriptor, config_.zoom, config_.zoom, "* Zoom: ");
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
        if (parameter.get_name() == "guid")
        {
            change_parameter(parameter, rclcpp::PARAMETER_STRING, config_.guid, result);
        }
        else if (parameter.get_name() == "video_mode")
        {
            if (camera1394::video_modes.find(parameter.as_string()) != camera1394::video_modes.end())
            {
                // Video mode is valid
                change_parameter(parameter, rclcpp::PARAMETER_STRING, config_.video_mode, result);
            }
            else
            {
                // Video mode is invalid
                result.reason = "Value: " + parameter.as_string() + " is not valid for parameter: " + parameter.get_name();
                result.successful = false;
            }
        }
        else if (parameter.get_name() == "frame_id")
        {
            change_parameter(parameter, rclcpp::PARAMETER_STRING, config_.frame_id, result);
        }
        else if (parameter.get_name() == "frame_rate")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.frame_rate, result);
        }
        else if (parameter.get_name() == "max_consecutive_errors")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.max_consecutive_errors, result);
        }
        else if (parameter.get_name() == "iso_speed")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.iso_speed, result);
        }
        else if (parameter.get_name() == "num_dma_buffers")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.num_dma_buffers, result);
        }
        else if (parameter.get_name() == "camera_info_url")
        {
            change_parameter(parameter, rclcpp::PARAMETER_STRING, config_.camera_info_url, result);
        }
        else if (parameter.get_name() == "reset_on_open")
        {
            change_parameter(parameter, rclcpp::PARAMETER_BOOL, config_.reset_on_open, result);
        }
        else if (parameter.get_name() == "use_ros_time")
        {
            change_parameter(parameter, rclcpp::PARAMETER_BOOL, config_.use_ros_time, result);
        }
        else if (parameter.get_name() == "time_offset")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.time_offset, result);
        }
        else if (parameter.get_name() == "binning_x")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.binning_x, result);
        }
        else if (parameter.get_name() == "binning_y")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.binning_y, result);
        }
        else if (parameter.get_name() == "roi_width")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.roi_width, result);
        }
        else if (parameter.get_name() == "roi_height")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.roi_height, result);
        }
        else if (parameter.get_name() == "x_offset")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.x_offset, result);
        }
        else if (parameter.get_name() == "y_offset")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.y_offset, result);
        }
        else if (parameter.get_name() == "format7_packet_size")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.format7_packet_size, result);
        }
        else if (parameter.get_name() == "format7_color_coding")
        {
            if (camera1394::format7_color_codings.find(parameter.as_string()) != camera1394::format7_color_codings.end())
            {
                // Color coding is valid
                change_parameter(parameter, rclcpp::PARAMETER_STRING, config_.format7_color_coding, result);
            }
            else
            {
                // Color coding is invalid
                result.reason = "Value: " + parameter.as_string() + " is not valid for parameter: " + parameter.get_name();
                result.successful = false;
            }
        }
        else if (parameter.get_name() == "bayer_pattern")
        {
            if (camera1394::bayer_patterns.find(parameter.as_string()) != camera1394::bayer_patterns.end())
            {
                // Bayer pattern valid
                change_parameter(parameter, rclcpp::PARAMETER_STRING, config_.bayer_pattern, result);
            }
            else
            {
                // Bayer pattern invalid
                result.reason = "Value: " + parameter.as_string() + " is not valid for parameter: " + parameter.get_name();
                result.successful = false;
            }
        }
        else if (parameter.get_name() == "bayer_method")
        {
            if (camera1394::bayer_methods.find(parameter.as_string()) != camera1394::bayer_methods.end())
            {
                // Bayer method invalid
                change_parameter(parameter, rclcpp::PARAMETER_STRING, config_.bayer_method, result);
            }
            else
            {
                // Bayer method invalid
                result.reason = "Value: " + parameter.as_string() + " is not valid for parameter: " + parameter.get_name();
                result.successful = false;
            }
        }
        else if (parameter.get_name() == "auto_brightness")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.auto_brightness, result);
        }
        else if (parameter.get_name() == "brightness")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.brightness, result);
        }
        else if (parameter.get_name() == "auto_exposure")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.auto_exposure, result);
        }
        else if (parameter.get_name() == "exposure")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.exposure, result);
        }
        else if (parameter.get_name() == "auto_focus")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.auto_focus, result);
        }
        else if (parameter.get_name() == "focus")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.focus, result);
        }
        else if (parameter.get_name() == "auto_gain")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.auto_gain, result);
        }
        else if (parameter.get_name() == "gain")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.gain, result);
        }
        else if (parameter.get_name() == "auto_gamma")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.auto_gamma, result);
        }
        else if (parameter.get_name() == "gamma")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.gamma, result);
        }
        else if (parameter.get_name() == "auto_hue")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.auto_hue, result);
        }
        else if (parameter.get_name() == "hue")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.hue, result);
        }
        else if (parameter.get_name() == "auto_iris")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.auto_iris, result);
        }
        else if (parameter.get_name() == "iris")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.iris, result);
        }
        else if (parameter.get_name() == "auto_pan ")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.auto_pan, result);
        }
        else if (parameter.get_name() == "pan")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.pan, result);
        }
        else if (parameter.get_name() == "auto_saturation")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.auto_saturation, result);
        }
        else if (parameter.get_name() == "saturation")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.saturation, result);
        }
        else if (parameter.get_name() == "auto_sharpness")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.auto_sharpness, result);
        }
        else if (parameter.get_name() == "sharpness")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.sharpness, result);
        }
        else if (parameter.get_name() == "auto_shutter")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.auto_shutter, result);
        }
        else if (parameter.get_name() == "shutter")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.shutter, result);
        }
        else if (parameter.get_name() == "external_trigger")
        {
            change_parameter(parameter, rclcpp::PARAMETER_BOOL, config_.external_trigger, result);
        }
        else if (parameter.get_name() == "software_trigger")
        {
            change_parameter(parameter, rclcpp::PARAMETER_BOOL, config_.software_trigger, result);
        }
        else if (parameter.get_name() == "trigger_mode")
        {
            if (camera1394::trigger_modes.find(parameter.as_string()) != camera1394::trigger_modes.end())
            {
                // Trigger mode is valid
                change_parameter(parameter, rclcpp::PARAMETER_STRING, config_.trigger_mode, result);
            }
            else
            {
                // Trigger mode is invalid
                result.reason = "Value: " + parameter.as_string() + " is not valid for parameter: " + parameter.get_name();
                result.successful = false;
            }
        }
        else if (parameter.get_name() == "trigger_source")
        {
            if (camera1394::trigger_sources.find(parameter.as_string()) != camera1394::trigger_sources.end())
            {
                // Trigger source valid
                change_parameter(parameter, rclcpp::PARAMETER_STRING, config_.trigger_source, result);
            }
            else
            {
                // Trigger source invalid
                result.reason = "Value: " + parameter.as_string() + " is not valid for parameter: " + parameter.get_name();
                result.successful = false;
            }
        }
        else if (parameter.get_name() == "trigger_polarity")
        {
            if (camera1394::trigger_polarities.find(parameter.as_string()) != camera1394::trigger_polarities.end())
            {
                // Trigger polarity valid
                change_parameter(parameter, rclcpp::PARAMETER_STRING, config_.trigger_polarity, result);
            }
            else
            {
                // Trigger polarity valid
                result.reason = "Value: " + parameter.as_string() + " is not valid for parameter: " + parameter.get_name();
                result.successful = false;
            }
        }
        else if (parameter.get_name() == "auto_trigger")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.auto_trigger, result);
        }
        else if (parameter.get_name() == "trigger")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.trigger, result);
        }
        else if (parameter.get_name() == "auto_white_balance")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.auto_white_balance, result);
        }
        else if (parameter.get_name() == "white_balance_BU")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.white_balance_BU, result);
        }
        else if (parameter.get_name() == "white_balance_RV")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.white_balance_RV, result);
        }
        else if (parameter.get_name() == "auto_zoom")
        {
            change_parameter(parameter, rclcpp::PARAMETER_INTEGER, config_.auto_zoom, result);
        }
        else if (parameter.get_name() == "zoom")
        {
            change_parameter(parameter, rclcpp::PARAMETER_DOUBLE, config_.zoom, result);
        }
        else
        {
            RCLCPP_WARN_STREAM(get_logger(), "Parameter with name: " << parameter.get_name() << " is not available.");
            result.reason = parameter.get_name() + " is not available as parameter.";
            result.successful = false;
        }
    }

    if (result.successful)
    {
        driver_.reconfig(config_, 0);
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
