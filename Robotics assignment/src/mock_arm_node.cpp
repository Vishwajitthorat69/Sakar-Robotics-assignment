#include <chrono>
#include <memory>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
using namespace std::chrono_literals;
class MockArmNode : public rclcpp::Node {
public:
  MockArmNode(): Node("mock_arm_node"), gen_(rd_()), dist_(0.0,1.0) {
    pick_srv_ = create_service<std_srvs::srv::Trigger>("trigger_pick", std::bind(&MockArmNode::handle_pick, this, std::placeholders::_1, std::placeholders::_2));
    place_srv_ = create_service<std_srvs::srv::Trigger>("trigger_place", std::bind(&MockArmNode::handle_place, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Mock arm ready");
  }
private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pick_srv_, place_srv_;
  std::random_device rd_; std::mt19937 gen_; std::uniform_real_distribution<> dist_;
  void handle_pick(const std::shared_ptr<std_srvs::srv::Trigger::Request> , std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    rclcpp::sleep_for(1s);
    double p = dist_(gen_);
    res->success = (p>0.15);
    res->message = res->success?"picked":"pick_failed";
  }
  void handle_place(const std::shared_ptr<std_srvs::srv::Trigger::Request> , std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    rclcpp::sleep_for(1s);
    double p = dist_(gen_);
    res->success = (p>0.1);
    res->message = res->success?"placed":"place_failed";
  }
};
int main(int argc, char ** argv){ rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<MockArmNode>()); rclcpp::shutdown(); return 0; }
