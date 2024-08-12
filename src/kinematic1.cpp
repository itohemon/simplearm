#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>


namespace simplearm
{

class Kinematic1 : public rclcpp::Node
{
public:
  Kinematic1(rclcpp::NodeOptions options = rclcpp::NodeOptions());

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_;
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimerElapsed();
  int count_;
};
        
Kinematic1::Kinematic1(rclcpp::NodeOptions options)
  : Node("pub_joint_states", options)
{
  count_ = 0;

  pub_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Kinematic1::onTimerElapsed, this));

  RCLCPP_INFO(this->get_logger(), "onTimerElapsed called");

  auto msg = std::make_unique<sensor_msgs::msg::JointState>();
  msg->header.stamp = this->now();
  msg->name.resize(2);
  msg->name[0] = "joint1";
  msg->name[1] = "joint2";
  msg->position.resize(2);
  msg->position[0] = 0.0;
  msg->position[1] = 0.0;
  pub_joint_->publish(std::move(msg)); 
}

void Kinematic1::onTimerElapsed()
{
  double l1 = 0.5;
  double l2 = 0.5;
  double theta1_deg = 30.0; // degree
  double theta2_deg = 30.0; // degree
  double theta1 = 2.0 * M_PI / 360.0 * theta1_deg;
  double theta2 = 2.0 * M_PI / 360.0 * theta2_deg;
  double px = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
  double py = l1 * sin(theta1) + l2 * sin(theta1 + theta2);

  auto msg = std::make_unique<sensor_msgs::msg::JointState>();
  msg->header.stamp = this->now();
  msg->name.resize(2);
  msg->name[0] = "joint1";
  msg->name[1] = "joint2";
  msg->position.resize(2);
  msg->position[0] = theta1;
  msg->position[1] = theta2;

  pub_joint_->publish(std::move(msg)); 
  RCLCPP_INFO(this->get_logger(), "(x, y) = (%lf, %lf)", px, py);
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(simplearm::Kinematic1)
