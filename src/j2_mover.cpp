#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <fanuc_msgs/srv/set_bool_io.hpp>
#include <chrono>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

class J2MoverCpp : public rclcpp::Node
{
public:
    J2MoverCpp() : Node("joint_j2_mover_cpp"), sent_(false)
    {
        // Publishers
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);
        
        // Subscribers
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&J2MoverCpp::joint_state_callback, this, std::placeholders::_1));
        
        // Service client for IO - same as Python
        set_io_client_ = this->create_client<fanuc_msgs::srv::SetBoolIO>(
            "/fanuc_gpio_controller/set_bool_io");
        
        // Wait for service - same as Python
        while (!set_io_client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        
        // Timer for sending trajectory
        timer_ = this->create_wall_timer(
            1s, std::bind(&J2MoverCpp::send_trajectory, this));
        
        RCLCPP_INFO(this->get_logger(), "J2 Mover C++ Node initialized");
    }

private:
    void set_io_signal(int index, bool value)
    {
        auto request = std::make_shared<fanuc_msgs::srv::SetBoolIO::Request>();
        request->io_type.type = "DO";
        request->index = index;
        request->value = value;
        
        // Use async call with callback as second parameter
        set_io_client_->async_send_request(
            request,
            [this](rclcpp::Client<fanuc_msgs::srv::SetBoolIO>::SharedFuture future) {
                try {
                    auto result = future.get();
                    RCLCPP_INFO(this->get_logger(), "IO signal set successfully");
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            }
        );
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (joint_names_.empty()) {
            joint_names_ = msg->name;
            initial_positions_ = msg->position;
            RCLCPP_INFO(this->get_logger(), "Joint names received:");
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "Joint[%zu]: %s", i, joint_names_[i].c_str());
            }
        }
    }
    
    void send_trajectory()
    {
        if (sent_ || initial_positions_.empty()) {
            if (initial_positions_.empty()) {
                RCLCPP_INFO(this->get_logger(), "Waiting for initial joint states...");
            }
            return;
        }
        
        auto traj = trajectory_msgs::msg::JointTrajectory();
        traj.joint_names = joint_names_;

        // Joint index
        int j_index = 0;

        // Create trajectory points
        double duration = 10.0;  // Duration to 10 seconds
        int steps = 200;
        double amplitude = M_PI / 12.0; // 180/12 = 15 degrees in radians

        for (int i = 0; i <= steps; ++i) {
            double t = i * duration / steps;
            double phase = (2.0 * M_PI / duration) * t;
            
            auto point = trajectory_msgs::msg::JointTrajectoryPoint();
            point.positions = initial_positions_;
            
            // Sine wave trajectory for J2 - same as Python
            double angle = amplitude * (1 - std::cos(phase));
            point.positions[j_index] = initial_positions_[j_index] + angle;
            
            // Calculate velocity
            double velocity = amplitude * (2 * M_PI / duration) * std::sin(phase);
            point.velocities.resize(point.positions.size(), 0.0);
            point.velocities[j_index] = velocity;

            point.time_from_start = rclcpp::Duration::from_seconds(t);
            traj.points.push_back(point);
        }
        
        traj_pub_->publish(traj);
        RCLCPP_INFO(this->get_logger(), "Published trajectory.");
        sent_ = true;
        timer_->cancel();
        
        // Set IO signal - same as Python
        set_io_signal(1, true);
    }
    
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Client<fanuc_msgs::srv::SetBoolIO>::SharedPtr set_io_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<std::string> joint_names_;
    std::vector<double> initial_positions_;
    bool sent_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<J2MoverCpp>());
    rclcpp::shutdown();
    return 0;
}