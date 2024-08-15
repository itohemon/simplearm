#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>


namespace simplearm
{

class InvertedKinematic : public rclcpp::Node
{
public:
  InvertedKinematic(rclcpp::NodeOptions options = rclcpp::NodeOptions());

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_;
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimerElapsed();
  int count_;
};
        
InvertedKinematic::InvertedKinematic(rclcpp::NodeOptions options)
  : Node("pub_joint_states", options)
{
  count_ = 0;

  pub_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&InvertedKinematic::onTimerElapsed, this));

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

void InvertedKinematic::onTimerElapsed()
{
  double l1 = 0.5;
  double l2 = 0.5;
  double px = 0.00;
  double py = 0.50;

  double theta2 = acos(((px * px) + (py * py) - (l1 * l1) - (l2 * l2)) / (2.0 * l1 * l2));
  double theta1 = atan2( -1.0 * l2 * sin(theta2) * px + (l1 + l2 * cos(theta2)) * py,
                         (l1 + l2 * cos(theta2)) * px + l2 * sin(theta2) * py);

  auto msg = std::make_unique<sensor_msgs::msg::JointState>();
  msg->header.stamp = this->now();
  msg->name.resize(2);
  msg->name[0] = "joint1";
  msg->name[1] = "joint2";
  msg->position.resize(2);
  msg->position[0] = theta1;
  msg->position[1] = theta2;

  pub_joint_->publish(std::move(msg)); 
  RCLCPP_INFO(this->get_logger(), "(x, y) = (%lf, %lf), (theta1, theta2) = (%lf, %lf)", px, py, theta1, theta2);
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(simplearm::InvertedKinematic)
