#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ThrusterMux : public rclcpp::Node {
public:
  ThrusterMux() : Node("thruster_cmd_mux") {
    gain_surge_ = this->declare_parameter("gain_surge", 50.0);
    gain_sway_  = this->declare_parameter("gain_sway",  50.0);
    gain_yaw_   = this->declare_parameter("gain_yaw",   30.0);

    pub_f_ = mkpub("/thruster_surge_front_controller/commands");
    pub_r_ = mkpub("/thruster_surge_rear_controller/commands");
    pub_p_ = mkpub("/thruster_sway_port_controller/commands");
    pub_s_ = mkpub("/thruster_sway_starboard_controller/commands");

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
      std::bind(&ThrusterMux::on_cmd_vel, this, std::placeholders::_1));
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mkpub(const std::string& t){
    return this->create_publisher<std_msgs::msg::Float64MultiArray>(t, 10);
  }
  template<typename PubT>
  void send(const PubT &p, double v) {
    std_msgs::msg::Float64MultiArray m;
    m.data = {v};
    p->publish(m);
}

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr m){
    const double u=m->linear.x, v=m->linear.y, r=m->angular.z;
    double Tf = gain_surge_*u + gain_yaw_*r;
    double Tr = gain_surge_*u - gain_yaw_*r;
    double Tp = gain_sway_ *v;
    double Ts = gain_sway_ *v;
    send(pub_f_,Tf); send(pub_r_,Tr); send(pub_p_,Tp); send(pub_s_,Ts);
  }

  double gain_surge_, gain_sway_, gain_yaw_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_f_, pub_r_, pub_p_, pub_s_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrusterMux>());
  rclcpp::shutdown();
  return 0;
}
