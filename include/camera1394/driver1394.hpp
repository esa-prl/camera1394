/* -*- mode: C++ -*- */
/* $Id$ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010 Jack O'Quin
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <boost/thread/mutex.hpp>

#include <rclcpp/rclcpp.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "camera1394/dev_camera1394.hpp"
#include "camera1394/camera1394_config.hpp"
#include "camera1394/srv/get_camera_registers.hpp"
#include "camera1394/srv/set_camera_registers.hpp"

typedef camera1394::Camera1394Config Config;

/** @file

    @brief ROS driver interface for IIDC-compatible IEEE 1394 digital cameras.

*/

namespace camera1394_driver
{

    // Dynamic reconfiguration levels
    //
    // Must agree with SensorLevels class in cfg/Camera1394.cfg
    class Levels
    {
    public:
        static const uint32_t RECONFIGURE_CLOSE = 3;   // Close the device to change
        static const uint32_t RECONFIGURE_STOP = 1;    // Stop the device to change
        static const uint32_t RECONFIGURE_RUNNING = 0; // Parameters that can change any time
    };

    class Camera1394Driver
    {
    public:
        // driver states
        static const uint8_t CLOSED = 0;  // Not connected to the device
        static const uint8_t OPENED = 1;  // Connected to the camera, ready to stream
        static const uint8_t RUNNING = 2; // Streaming images

        // public methods
        Camera1394Driver(rclcpp::Node *private_nh);

        ~Camera1394Driver();
        void poll(void);
        void setup(void);
        void shutdown(void);

    private:
        // private methods
        void closeCamera();
        bool openCamera(Config &newconfig);
        void publish(sensor_msgs::msg::Image::SharedPtr image);
        bool read(sensor_msgs::msg::Image::SharedPtr image);
        void reconfig(camera1394::Camera1394Config &newconfig, uint32_t level);

        bool getCameraRegisters(const std::shared_ptr<camera1394::srv::GetCameraRegisters::Request> request,
                                std::shared_ptr<camera1394::srv::GetCameraRegisters::Response> response);
        bool setCameraRegisters(const std::shared_ptr<camera1394::srv::SetCameraRegisters::Request> request,
                                std::shared_ptr<camera1394::srv::SetCameraRegisters::Response> response);

        /** Non-recursive mutex for serializing callbacks with device polling. */
        boost::mutex mutex_;

        /** driver state variables */
        volatile uint8_t state_;           // current driver state
        volatile bool reconfiguring_;      // true when reconfig() running
        rclcpp::Node *private_nh_;         // private node handle
        std::string camera_name_;          // camera name
        rclcpp::Rate cycle_;               // polling rate when closed
        uint32_t retries_;                 // count of openCamera() retries
        uint32_t consecutive_read_errors_; // number of consecutive read errors

        /** libdc1394 camera device interface */
        std::shared_ptr<camera1394::Camera1394> dev_;

        /** dynamic parameter configuration */
        camera1394::Camera1394Config config_;

        /** camera calibration information */
        std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
        bool calibration_matches_; // CameraInfo matches video mode

        /** image transport interfaces */
        image_transport::CameraPublisher image_pub_;

        /** services for getting/setting camera control and status registers (CSR) */
        rclcpp::Service<camera1394::srv::GetCameraRegisters>::SharedPtr get_camera_registers_srv_;
        rclcpp::Service<camera1394::srv::SetCameraRegisters>::SharedPtr set_camera_registers_srv_;
        // ros::ServiceServer get_camera_registers_srv_;
        // ros::ServiceServer set_camera_registers_srv_;

        /** diagnostics updater */
        diagnostic_updater::Updater diagnostics_;
        double topic_diagnostics_min_freq_;
        double topic_diagnostics_max_freq_;
        diagnostic_updater::TopicDiagnostic topic_diagnostics_;

    }; // end class Camera1394Driver

} // end namespace camera1394_driver
