#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class TwistToThrustNode : public rclcpp::Node {
public:
  TwistToThrustNode() : Node("twist_to_thrust_node") {
    gain_surge_ = this->declare_parameter("gain_surge", 50.0);
    gain_sway_  = this->declare_parameter("gain_sway",  50.0);
    gain_yaw_   = this->declare_parameter("gain_yaw",   30.0);

    pub_f_ = make_pub("/thruster_surge_front_controller/commands");
    pub_r_ = make_pub("/thruster_surge_rear_controller/commands");
    pub_p_ = make_pub("/thruster_sway_port_controller/commands");
    pub_s_ = make_pub("/thruster_sway_starboard_controller/commands");

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&TwistToThrustNode::onTwist, this, std::placeholders::_1));
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
  make_pub(const std::string &topic) {
    return this->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 10);
  }

  static void send(const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr &p, double v) {
    std_msgs::msg::Float64MultiArray m; m.data = {v}; p->publish(m);
  }

  void onTwist(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const double u = msg->linear.x;
    const double v = msg->linear.y;
    const double r = msg->angular.z;

    const double Tf = gain_surge_*u + gain_yaw_*r;
    const double Tr = gain_surge_*u - gain_yaw_*r;
    const double Tp = gain_sway_*v;
    const double Ts = gain_sway_*v;

    send(pub_f_, Tf); send(pub_r_, Tr); send(pub_p_, Tp); send(pub_s_, Ts);
  }

  double gain_surge_, gain_sway_, gain_yaw_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_f_, pub_r_, pub_p_, pub_s_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToThrustNode>());
  rclcpp::shutdown();
  return 0;
}
