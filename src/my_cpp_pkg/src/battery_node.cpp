
#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
 
class BatteryNode : public rclcpp::Node
{
public:
   BatteryNode() : Node("battery_node"), battery_status_(100), counter_(1)
   {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&BatteryNode::mimicBatteryUsage, this));
        //threads.push_back(std::thread(std::bind(&BatteryNode::callSetLedService, this, 1, 200)));
   }
 
private:

    void mimicBatteryUsage()
    {
        counter_++;
        RCLCPP_INFO(this->get_logger(), "counter = %d", counter_);
        // battery empty from 4-10 seconds, 14-20, 24-30, etc....
        if(counter_ % 10 == 4 )
        {
            RCLCPP_INFO(this->get_logger(), "battery is EMPTY, sending request to turn LED on");
            battery_status_= 0;

            //if you call callSetLedService(3, 1); directly this will block at future.get() below
            // ANSWER: If you are calling future.get() directly in the same thread that is also responsible for spinning 
            //         (i.e., processing incoming messages and callbacks), then the thread cannot handle any other events, including receiving the response from the service.

            threads_.push_back(std::thread(std::bind(&BatteryNode::callSetLedService, this, 3, 1)));
        }
        else if(counter_ % 10 == 0) // battery full 10-14[, 20-24[, 30-34[, etc.....
        {
            battery_status_= 100;

            //if you call callSetLedService(3, 1); directly this will block at future.get() below
            // ANSWER: If you are calling future.get() directly in the same thread that is also responsible for spinning 
            //         (i.e., processing incoming messages and callbacks), then the thread cannot handle any other events, including receiving the response from the service.
            threads_.push_back(std::thread(std::bind(&BatteryNode::callSetLedService, this, 3, 0)));
            RCLCPP_INFO(this->get_logger(), "battery is FULL, sending request to turn LED off");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "battery is filling or emptying");
        }

    }

    void callSetLedService(int64_t ledNumber, int64_t ledState)
    {
        auto client = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");
        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "WARNING! Waiting for set LED server to be up!");
        }

        RCLCPP_INFO(this->get_logger(), "Set LED server is up!");

        auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
        request->led_number = ledNumber;
        request->state = ledState;

        auto future = client->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Async request sent");

        try{
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Result = %d", response->success);
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

    std::vector<std::thread> threads_;
    int battery_status_;
    int counter_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<BatteryNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}