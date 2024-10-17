// QUESTO TASK NODE MUOVE L'UR NELLO SPAZIO GIUNTI DA UNA POSIZIONE
// DI HOME AD UNA POSIZIONE DI PICK
// POI VENGONO CALIBRATI I SENSORI DELLA MANO, VENGONO INVIATE LE NORME DESIDERATE
// E POI SI CHIUDE LA MANO
// IL ROBOT VA PRIMA IN UNA POSIZIONE DI PRE-PLACE, POI IN PLACE
// LA MANO SI APRE
// IL ROBOT TORNA IN HOME



#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "uclv_seed_robotics_ros_interfaces/srv/slipping_avoidance.hpp"
#include "uclv_dynamixel_utils/colors.hpp"

#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class TaskNode2 : public rclcpp::Node
{
public:
    using Traj = control_msgs::action::FollowJointTrajectory;
    using GoalHandleTraj = rclcpp_action::ClientGoalHandle<Traj>;

    std::vector<int64_t> sensor_ids = {0, 1, 2, 3, 4};
    std::vector<double> sensor_norms = {0.2, 0.2, 0.2, 0.2, 0.2};

    std::vector<int64_t> hand_ids = {31, 32, 33, 34, 35, 36, 37, 38};
    std::vector<double> init_position = {100, 2000, 2000, 3000, 100, 100, 100, 100};

    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr motor_positions_pub_;

    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_publisher_;
    rclcpp::Client<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance>::SharedPtr slipping_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr close_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr open_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr calibrate_client_;

    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped msg;

    rclcpp_action::Client<Traj>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> joint_trajectory_joint_names_ = {
        "shoulder_lift_joint",
        "shoulder_pan_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"};

    std::vector<double> home_position = {
        -2.006,
        0.082,
        -2.142,
        -3.407,
        5.389,
        -2.185};

    std::vector<double> pick_position = {
        -0.813,
        1.205,
        -2.175,
        -2.272,
        5.936,
        -2.653};

    std::vector<double> pre_place_position = {
        -1.280,
        1.199,
        -2.698,
        -2.480,
        5.755,
        -2.070};

    std::vector<double> place_position = {
        -1.507,
        1.111,
        -2.144,
        -3.201,
        5.268,
        -2.175};

    TaskNode2() : Node("task_node2")
    {
        motor_positions_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>("/cmd/positions", 10);

        desired_norm_publisher_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            "/cmd/norm_forces", 10);

        close_client_ = this->create_client<std_srvs::srv::SetBool>("close");
        slipping_client_ = this->create_client<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance>("slipping_activation");
        open_client_ = this->create_client<std_srvs::srv::SetBool>("open");
        calibrate_client_ = this->create_client<std_srvs::srv::Trigger>("calibrate_sensors");
        this->client_ptr_ = rclcpp_action::create_client<Traj>(
            this,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory");
        // this->timer_ = this->create_wall_timer(
        // std::chrono::milliseconds(500),
        // std::bind(&TaskNode2::send_goal, this));
    }

    void run()
    {
        // open thumb
        // go to home
        // go to pick
        // run hand
        // close thumb
        // calibrate
        // publish norm desired
        // close
        // end run hand
        // go to pre-place
        // go to place
        std::this_thread::sleep_for(std::chrono::seconds{2});
        set_positions(hand_ids, init_position);
        std::this_thread::sleep_for(std::chrono::seconds{2});

        std::cout << "Go Home..." << std::endl;
        std::cin.get();
        goal_traj(home_position, 10.0);

        std::cout << "Go Pick..." << std::endl;
        std::cin.get();
        goal_traj(pick_position, 10.0);

        std::this_thread::sleep_for(std::chrono::seconds{2});

        hand_run();

        

        std::cout << "Go Pre-Place..." << std::endl;
        std::cin.get();
        goal_traj(pre_place_position, 5.0);

        std::cout << "Go Place..." << std::endl;
        std::cin.get();
        goal_traj(place_position, 5.0);

        std::cout << "Press for open..." << std::endl;
        std::cin.get();
        call_open_service();
        call_open_service();

        std::cout << "Go Home..." << std::endl;
        std::cin.get();
        goal_traj(home_position, 5.0);
    }

private:
    void set_positions(const std::vector<int64_t> ids, const std::vector<double> positions)
    {
        auto msg = uclv_seed_robotics_ros_interfaces::msg::MotorPositions();
        msg.header.stamp = rclcpp::Clock{}.now();
        for (size_t i = 0; i < ids.size(); i++)
        {
            msg.ids.push_back(ids[i]);
        }
        for (size_t i = 0; i < positions.size(); i++)
        {
            msg.positions.push_back(positions[i]);
        }
        
        motor_positions_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Init position published.");
    }

    void hand_run()
    {
        calibrate();
        std::cout << "Calibrate " << SUCCESS_COLOR << "DONE" << CRESET << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds{2});
        publish_desired_norm_forces(sensor_ids, sensor_norms);
        std::cout << "Publish desired norm forces " << SUCCESS_COLOR << "DONE" << CRESET << std::endl;
        std::cout << "Press for close..." << std::endl;
        std::cin.get();
        call_close_service();
    }

    void publish_desired_norm_forces(const std::vector<int64_t> ids, const std::vector<double> norms)
    {
        msg.header.stamp = rclcpp::Clock{}.now();
        for (size_t i = 0; i < ids.size(); i++)
        {
            msg.ids.push_back(ids[i]);
            msg.data.push_back(norms[i]);
        }
        desired_norm_publisher_->publish(msg);
    }

    void call_close_service()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        close_client_->async_send_request(request);
    }

    void call_slipping_service(const std::vector<int64_t> ids, const std::vector<double> norms)
    {
        auto request = std::make_shared<uclv_seed_robotics_ros_interfaces::srv::SlippingAvoidance::Request>();
        request->data = msg.data;
        request->ids = msg.ids;
        slipping_client_->async_send_request(request);
    }

    void call_open_service()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        open_client_->async_send_request(request);
        std::this_thread::sleep_for(std::chrono::seconds{1});
    }

    void calibrate()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result = calibrate_client_->async_send_request(request);
    }

    void send_goal()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        std::cout << "Go Home..." << std::endl;
        std::cin.get();
        goal_traj(home_position, 10.0);

        std::cout << "Go Pick..." << std::endl;
        std::cin.get();
        goal_traj(pick_position, 10.0);

        std::cout << "Go Pre-Place..." << std::endl;
        std::cin.get();
        goal_traj(pre_place_position, 5.0);

        std::cout << "Go Place..." << std::endl;
        std::cin.get();
        goal_traj(place_position, 5.0);

        std::cout << "Go Home..." << std::endl;
        std::cin.get();
        goal_traj(home_position, 10.0);
    }

    void goal_traj(const std::vector<double> position, double time)
    {
        auto goal_msg = Traj::Goal();
        goal_msg.trajectory.points.resize(1);
        for (size_t i = 0; i < joint_trajectory_joint_names_.size(); i++)
        {
            goal_msg.trajectory.joint_names.push_back(joint_trajectory_joint_names_[i]);
            goal_msg.trajectory.points[0].positions.push_back(position[i]);
        }
        goal_msg.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(time);

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Traj>::SendGoalOptions();
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(std::shared_future<GoalHandleTraj::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskNode2>();
    node->run();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
