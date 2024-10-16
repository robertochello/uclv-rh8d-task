// TASK PER PROVARE NORME DIVERSE DURANTE IL GRASP
// QUESTO HA DELLE NORME USATE PER LA SPUGNA
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "uclv_seed_robotics_ros_interfaces/srv/slipping_avoidance.hpp"
#include "uclv_dynamixel_utils/colors.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

// FOR THE SPONGE

class TaskNode3 : public rclcpp::Node
{
public:
    std::vector<double> desired_norm_data_1 = {0.1, 0.2, 0.2, 0.2, 0.2};
    std::vector<double> desired_norm_data_2 = {0.1, 0.2, 0.2, 0.2, 0.2};
    std::vector<double> desired_norm_data_3 = {0.1, 0.2, 1.0, 0.2, 0.2};
    std::vector<double> desired_norm_data_4 = {0.1, 1.0, 1.0, 0.7, 0.7};
    std::vector<double> desired_norm_data_5 = {0.1, 1.0, 1.0, 0.7, 0.7};
    std::vector<int64_t> desired_norm_ids_ = {0, 1, 2, 3, 4};

    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_publisher_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr close_client_;
    rclcpp::Client<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance>::SharedPtr slipping_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr open_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr calibrate_client_;

    TaskNode3()
        : Node("task_node3")
    {
        desired_norm_publisher_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            "/cmd/norm_forces", 10);

        close_client_ = this->create_client<std_srvs::srv::SetBool>("close");
        slipping_client_ = this->create_client<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance>("slipping_activation");
        open_client_ = this->create_client<std_srvs::srv::SetBool>("open");
        calibrate_client_ = this->create_client<std_srvs::srv::Trigger>("calibrate_sensors");
    }

    void run()
    {
        std::this_thread::sleep_for(std::chrono::seconds{5});
        calibrate();
        std::cout << "Calibrate " << SUCCESS_COLOR << "DONE" << CRESET << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds{3});

        publish_desired_norm_forces(desired_norm_data_1);
        std::cout << "Publish desired norm forces 1 " << SUCCESS_COLOR << "DONE" << CRESET << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds{1});

        std::cout << "Press for close..." << std::endl;
        std::cin.get();
        call_close_service();

        std::this_thread::sleep_for(std::chrono::seconds{20});

        publish_desired_norm_forces(desired_norm_data_2);
        std::cout << "Publish desired norm forces 2 " << SUCCESS_COLOR << "DONE" << CRESET << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds{20});

        publish_desired_norm_forces(desired_norm_data_3);
        std::cout << "Publish desired norm forces 3 " << SUCCESS_COLOR << "DONE" << CRESET << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds{20});

        publish_desired_norm_forces(desired_norm_data_4);
        std::cout << "Publish desired norm forces 4 " << SUCCESS_COLOR << "DONE" << CRESET << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds{20});

        publish_desired_norm_forces(desired_norm_data_4);
        
        std::cout << "Press for open..." << std::endl;
        std::cin.get();
        std::this_thread::sleep_for(std::chrono::seconds{2});
        call_open_service();
    }

private:
    void publish_desired_norm_forces(const std::vector<double> norms)
    {
        uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped desired_norm_msg_;
        desired_norm_msg_.header.stamp = this->get_clock()->now();
        for (size_t i = 0; i < desired_norm_ids_.size(); i++)
        {
            desired_norm_msg_.ids.push_back(desired_norm_ids_[i]);
            desired_norm_msg_.data.push_back(norms[i]);
        }
        desired_norm_publisher_->publish(desired_norm_msg_);
    }

    void call_close_service()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        close_client_->async_send_request(request);
    }

    void call_open_service()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        open_client_->async_send_request(request);
    }

    void calibrate()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result = calibrate_client_->async_send_request(request);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskNode3>();
    node->run();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
