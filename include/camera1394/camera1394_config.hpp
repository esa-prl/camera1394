
#ifndef CAMERA1394_CONFIG_HPP_
#define CAMERA1394_CONFIG_HPP_

#include <string>
namespace camera1394
{
    // Feature control states
    enum FeatureControlStates
    {
        // Use fixed value
        Camera1394_Off,
        // Query current values
        Camera1394_Query,
        // Camera sets continously
        Camera1394_Auto,
        // Use explicit value
        Camera1394_Manual,
        // Camera sets onces
        Camera1394_OnePush,
        // Feature not available
        Camera1394_None
    };

    struct Camera1394Config
    {
        // Global Unique ID of camera, 16 hex digits (use first camera if null).
        // Default: ""
        std::string guid = "";

        // IIDC video mode.
        // Default: 640x480_mono8
        // Constraints:
        // 160x120_yuv444
        // 320x240_yuv422
        // 640x480_yuv411
        // 640x480_yuv422
        // 640x480_rgb8
        // 640x480_mono8
        // 640x480_mono16
        // 800x600_yuv422
        // 800x600_rgb8
        // 800x600_mono8
        // 800x600_mono16
        // 1024x768_yuv422
        // 1024x768_rgb8
        // 1024x768_mono8
        // 1024x768_mono16
        // 1280x960_yuv422
        // 1280x960_rgb8
        // 1280x960_mono8
        // 1280x960_mono16
        // 1600x1200_yuv422
        // 1600x1200_rgb8
        // 1600x1200_mono8
        // 1600x1200_mono16
        // format7_mode0
        // format7_mode1
        // format7_mode2
        // format7_mode3
        // format7_mode4
        // format7_mode5
        // format7_mode6
        // format7_mode7
        std::string video_mode = "640x480_mono8";

        // ROS tf frame of reference, resolved with tf_prefix unless absolute.
        // Default: camera
        std::string frame_id = "camera";

        // Camera speed (frames per second).
        // Default: 15.0 Min: 1.875 Max: 240.0
        double frame_rate = 15.0;

        // Max number of consecutive read errors before attempting reconnection (0 to disable).
        // Default: 0 Min: 0 Max: 1000
        int max_consecutive_errors = 0;

        // Total IEEE 1394 bus bandwidth (Megabits/second).
        // Default: 400 Min: 100 Max: 3200
        int iso_speed = 400;

        // Number of frames in the DMA ring buffer.
        // Default: 4 Min: 1 Max: 1024
        int num_dma_buffers = 4;

        // Camera [[camera_info_manager#URL_Names|calibration URL]] for this
        // video_mode (uncalibrated if null)
        // Default: ""
        std::string camera_info_url = "";

        // Reset camera when opening the device.
        // Default: False
        bool reset_on_open = false;

        // Timestamp for Image and CameraInfo using rclcpp::Clock::now()
        // Default: False
        bool use_ros_time = false;

        // Offset in seconds to add to rclcpp::Clock::now()
        // Default: 0.0 Min: -5.0 Max: 1.0
        double time_offset = 0.0;

        //////
        //// Format7 specific parameters

        // Number of pixels combined for Format7 horizontal binning, use device hints if zero.
        // Default: 0 Min: 0 Max: 4
        int binning_x = 0;

        // Number of pixels combined for Format7 vertical binning, use device hints if zero.
        // Default: 0 Min: 0 Max: 4
        int binning_y = 0;

        // Width of Format7 Region of Interest in unbinned pixels, full width if zero.
        // Default: 0 Min: 0 Max: 65535
        int roi_width = 0;

        // Height of Format7 Region of Interest in unbinned pixels, full height if zero.
        // Default: 0 Min: 0 Max: 65535
        int roi_height = 0;

        // Horizontal offset for left side of Format7 ROI in unbinned pixels.
        // Default: 0 Min: 0 Max: 65535
        int x_offset = 0;

        // Vertical offset for top side of Format7 ROI in unbinned pixels.
        // Default: 0 Min: 0 Max: 65535
        int y_offset = 0;

        // Format7 packet size (bytes), device-recommended size if zero.
        // Default: 0 Min: 0 Max: 39320
        int format7_packet_size = 0;

        // Color coding (only for Format7 modes)
        // Default: mono8
        // Constraints: mono8, mono16, mono16s, raw8, raw16, rgb8, rgb16, rgb16s, yuv411, yuv422, yuv444
        std::string format7_color_coding = "mono8";

        //////
        //// Bayer color filter parameters

        // Bayer color encoding pattern
        // Default: none (No Bayer encoding)
        // Constraints: none, rggb, gbrg, grbg, bggr
        std::string bayer_pattern = "";

        // Bayer decoding method
        // Default: "" (Decode via ROS image_proc)
        // Constraints: "", DownSample, Simple, Bilinear, HQ, VNG, AHD
        std::string bayer_method = "";

        //////
        //// Generic IIDC features

        // Brightness control state
        // Default: 1 (Query)
        // Constraints: 0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)
        int auto_brightness = Camera1394_Query;

        // Black level offset
        // Default: 0.0 Min: 0.0 Max: 4095.0
        double brightness = 0.0;

        // Combined Gain, Iris & Shutter control
        // Default: 1 (Query)
        // Constraints: 0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)
        int auto_exposure = Camera1394_Query;

        // Auto exposure value (like contrast)
        // Default: 0.0 Min: -10.0 Max: 4095.0
        double exposure = 0.0;

        // Focus control state
        // Default: 1 (Query)
        // Constraints: 0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)
        int auto_focus = Camera1394_Query;

        // Focus control
        // Default: 0.0 Min: 0.0 Max: 4095.0
        double focus = 0.0;

        // Gain control state
        // Default: 1 (Query)
        // Constraints: 0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)
        int auto_gain = Camera1394_Query;

        // Relative circuit gain
        // Default: 0.0 Min: -10.0 Max: 4095.0
        double gain = 0.0;

        // Gamma control state
        // Default: 1 (Query)
        // Constraints: 0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)
        int auto_gamma = Camera1394_Query;

        // Gamma expansion exponent
        // Default: 2.2 Min: 0.0 Max: 10.0
        double gamma = 2.2;

        // Hue control state
        // Default: 1 (Query)
        // Constraints: 0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)
        int auto_hue = Camera1394_Query;

        // Color phase
        // Default: 0.0 Min: 0.0 Max: 4095.0
        double hue = 0.0;

        // Iris control state
        // Default: 1 (Query)
        // Constraints: 0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)
        int auto_iris = Camera1394_Query;

        // Iris control
        // Default: 8.0 Min: 0.0 Max: 4095.0
        double iris = 8.0;

        // Pan control state
        // Default: 1 (Query)
        // Constraints: 0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)
        int auto_pan = Camera1394_Query;

        // Pan control
        // Default: 0.0 Min: 0.0 Max: 4095.0
        double pan = 0.0;

        // Saturation control state
        // Default: 1 (Query)
        // Constraints: 0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)
        int auto_saturation = Camera1394_Query;

        // Color saturation
        // Default: 1.0 Min: 0.0 Max: 4095.0
        double saturation = 1.0;

        // Sharpness control state
        // Default: 1 (Query)
        // Constraints: 0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)
        int auto_sharpness = Camera1394_Query;

        // Image sharpness
        // Default: 1.0 Min: 0.0 Max: 4095.0
        double sharpness = 1.0;

        // Shutter control state
        // Default: 1 (Query)
        // Constraints: 0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)
        int auto_shutter = Camera1394_Query;

        // Shutter Speed
        // Default: 1.0 Min: 0.0 Max: 4095.0
        double shutter = 1.0;

        // External trigger power state
        // Default: false
        bool external_trigger = false;

        // Software trigger power state
        // Default: false
        bool software_trigger = false;

        // External trigger mode
        // Default: mode_0
        // Constraints:
        // mode_0 - Exposure starts with a falling edge and stops when the the exposure specified by the SHUTTER feature is elapsed
        // mode_1 - Exposure starts with a falling edge and stops with the next rising edge
        // mode_2 - The camera starts the exposure at the first falling edge and stops the integration at the nth falling edge
        // mode_3 - This is an internal trigger mode. The trigger is generated every n*(period of fastest framerate)
        // mode_4 - A multiple exposure mode. N exposures are performed each time a falling edge is observed on the trigger signal. Each exposure is as long as defined by the SHUTTER feature
        // mode_5 - Same as Mode 4 except that the exposure is is defined by the length of the trigger pulse instead of the SHUTTER feature
        // mode_14 -- vendor specified trigger mode
        // mode_15 -- vendor specified trigger mode
        std::string trigger_mode = "mode_0";

        // External trigger source
        // Default: source_0
        // Constraints: source_0, source_1, source_2, source_3, source_software
        std::string trigger_source = "source_0";

        // Trigger polarity
        // Default: active_low
        // Constraints: acitve_low, active_high
        std::string trigger_polarity = "active_low";

        // Trigger control state
        // Default: 1 (Query)
        // Constraints: 0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)
        int auto_trigger = Camera1394_Query;

        // Trigger parameter N
        // Default: 0.0 Min: 0.0 Max: 4095.0
        double trigger = 0.0;

        // White balance control state
        // Default: 1 (Query)
        // Constraints: 0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)
        int auto_white_balance = Camera1394_Query;

        // Blue or U component of white balance
        // Default: 0.0 Min: 0.0 Max: 4095.0
        double white_balance_BU = 0.0;

        // Red or V component of white balance
        // Default: 0.0 Min: 0.0 Max: 4095.0
        double white_balance_RV = 0.0;

        // Zoom control state
        // Default: 1 (Query)
        // Constraints: 0 (Off), 1 (Query), 2 (Auto), 3 (Manual), 4 (OnePush)
        int auto_zoom = Camera1394_Query;

        // Zoom control
        // Default: 0.0 Min: 0.0 Max: 4095.0
        double zoom = 0.0;
    };

} // namespace camera1394
#endif
