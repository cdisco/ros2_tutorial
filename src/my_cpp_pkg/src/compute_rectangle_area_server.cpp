#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/compute_rectangle_area.hpp"

using std::placeholders::_1;
// using std::placeholders::_2;

class ComputeRectangleAreaServerNode : public rclcpp::Node
{
public:
    ComputeRectangleAreaServerNode() : Node("compute_rectangle_area_server")
    {
        server_ = this->create_service<my_robot_interfaces::srv::ComputeRectangleArea>(
            "compute_rectangle_area",
            std::bind(&ComputeRectangleAreaServerNode::callbackComputeRectangleArea, this, _1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "ComputeRectangleAreaServerNode initialized and server service started");
    }

private:
    rclcpp::Service<my_robot_interfaces::srv::ComputeRectangleArea>::SharedPtr server_;

    void callbackComputeRectangleArea(const my_robot_interfaces::srv::ComputeRectangleArea_Request::SharedPtr request,
                                      const my_robot_interfaces::srv::ComputeRectangleArea_Response::SharedPtr response)
    {
        response->area = request->length * request->width;
        RCLCPP_INFO(this->get_logger(), "Reactangle Area = %f x %f = %f", request->length, request->width, response->area);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ComputeRectangleAreaServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}