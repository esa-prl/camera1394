#include "camera1394/camera1394_node.hpp"

Camera1394Node::Camera1394Node()
    : Node("Camera1394Node")
{

    RCLCPP_INFO(this->get_logger(), "%s initialized", this->get_name());
}

Camera1394Node::~Camera1394Node()
{
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Camera1394Node>());

    rclcpp::shutdown();
    return 0;
}
