#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <algorithm>

class TwistToThrust : public rclcpp::Node {
public:
  TwistToThrust() : Node("twist_to_thrust") {
    declare_parameter<double>("max_thrust_N", 55.0);
    maxN_ = get_parameter("max_thrust_N").as_double();

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){ handle(*msg); });

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/thruster_controller/commands", 10);
  }
private:
  void handle(const geometry_msgs::msg::Twist & t){
    double ux = t.linear.x;   // surge
    double uy = t.linear.y;   // sway
    double r  = t.angular.z;  // yaw

    // Enkelt allokeringsskjema (tuning senere)
    double kx=40.0, ky=40.0, kr=15.0;

    double Tsl =  kx*ux + kr*(-r);
    double Tsr =  kx*ux + kr*(+r);
    double Tsf =  ky*uy;
    double Tsa = -ky*uy;

    auto clamp=[&](double v){ return std::clamp(v,-maxN_,maxN_); };
    std_msgs::msg::Float64MultiArray out;
    out.data = { clamp(Tsl), clamp(Tsr), clamp(Tsf), clamp(Tsa) };
    pub_->publish(out);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  double maxN_;
};

int main(int argc, char** argv)
{ 
  rclcpp::init(argc, argv); 
  rclcpp::spin(std::make_shared<TwistToThrust>()); 
  rclcpp::shutdown(); 
  
  return 0; 
}
