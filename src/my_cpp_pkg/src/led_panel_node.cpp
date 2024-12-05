
#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/led_states.hpp"

using std::placeholders::_1;

class LedPanelNode : public rclcpp::Node
{
public:
   LedPanelNode() : Node("led_panel_node"), led_1_on(false),led_2_on(false),led_3_on(false)
   {
        server_ = this->create_service<my_robot_interfaces::srv::SetLed>(
            "set_led",
            std::bind(&LedPanelNode::callbackSetLed, this, _1, std::placeholders::_2));

        timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&LedPanelNode::publishLedStates, this));

        pub_ = this->create_publisher<my_robot_interfaces::msg::LedStates>("led_states", 10);

        RCLCPP_INFO(this->get_logger(), "LedPanelNode initialized and Set Led server service started" );
   }
 
private:
    rclcpp::Publisher<my_robot_interfaces::msg::LedStates>::SharedPtr pub_;
    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool led_1_on;
    bool led_2_on;
    bool led_3_on;

    void callbackSetLed(const my_robot_interfaces::srv::SetLed_Request::SharedPtr request, 
                        const my_robot_interfaces::srv::SetLed_Response::SharedPtr response)

    {
        RCLCPP_INFO(this->get_logger(), "Request received. led_number = %ld", request->led_number);

        if(request->led_number > 3 || request->led_number <= 0)
        {
            response->success = false;
        }

        // TODO: clean up and make array
        if(request->led_number == 3)
        {
            if(request->state == 0){
                led_3_on = false;
            }
            else{
                led_3_on = true;
            }
            response->success = true;
            publishLedStates();
        }
        RCLCPP_INFO(this->get_logger(), "Returning response");
    }

    void publishLedStates()
    {
        auto msg = my_robot_interfaces::msg::LedStates();
        msg.led_1_on = this->led_1_on;
        msg.led_2_on = this->led_2_on;
        msg.led_3_on = this->led_3_on;
        pub_->publish(msg);
    }

};
 
int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<LedPanelNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}