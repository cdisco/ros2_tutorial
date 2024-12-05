#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
// using std::placeholders::_2;

class AddTwoIntsServerNode : public rclcpp::Node
{
public:
   AddTwoIntsServerNode() : Node("add_two_ints_server")
   {
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, this, _1, std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "AddTwoIntsServerNode initialized and server service started" );
   }
 
private:
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;

    void callbackAddTwoInts(const example_interfaces::srv::AddTwoInts_Request::SharedPtr request, 
                            const example_interfaces::srv::AddTwoInts_Response::SharedPtr response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "%ld + %ld = %ld", request->a, request->b, response->sum );
    }
};
 
int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<AddTwoIntsServerNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}