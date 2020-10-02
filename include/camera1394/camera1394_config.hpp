
#ifndef CAMERA1394_CONFIG_HPP_
#define CAMERA1394_CONFIG_HPP_

#include <string>
namespace camera1394
{
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
    };
} // namespace camera1394
#endif
