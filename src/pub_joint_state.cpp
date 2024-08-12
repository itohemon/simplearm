#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>


namespace simplearm
{

class PubJointState : public rclcpp::Node
{
public:
  PubJointState(rclcpp::NodeOptions options = rclcpp::NodeOptions());

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_;
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimerElapsed();
  int count_;
};
        
PubJointState::PubJointState(rclcpp::NodeOptions options)
  : Node("pub_joint_states", options)
{
  count_ = 0;

  pub_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PubJointState::onTimerElapsed, this));
}

void PubJointState::onTimerElapsed()
{
  RCLCPP_INFO(this->get_logger(), "onTimerElapsed called");

  auto msg = std::make_unique<sensor_msgs::msg::JointState>();
  msg->header.stamp = this->now();
  msg->name.resize(2);
  msg->name[0] = "joint1";
  msg->name[1] = "joint2";
  msg->position.resize(2);
  msg->position[0] = -1.0 * (float)count_ / 40.0;
  msg->position[1] =  2.0 * (float)count_ / 40.0;

  pub_joint_->publish(std::move(msg)); 
  count_++;
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(simplearm::PubJointState)
