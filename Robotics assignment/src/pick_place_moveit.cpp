#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
using namespace std::chrono_literals;
class PickPlaceMoveIt : public rclcpp::Node {
public:
  PickPlaceMoveIt(): Node("pick_place_moveit") {
    RCLCPP_INFO(get_logger(), "Starting MoveIt pick-place node (minimal)");            // Note: MoveGroupInterface requires proper MoveIt2 setup (planning contexts and robot description on the parameter server)
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this, "arm");
    timer_ = this->create_wall_timer(5s, std::bind(&PickPlaceMoveIt::cycle, this));
  }
private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  void cycle() {
    RCLCPP_INFO(get_logger(), "MoveIt cycle: planning to pick pose");
    geometry_msgs::msg::Pose target;
    target.position.x = 0.35;
    target.position.y = 0.0;
    target.position.z = 0.15;
    target.orientation.w = 1.0;
    move_group_->setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (ok) {
      RCLCPP_INFO(get_logger(), "Executing pick plan");
      move_group_->execute(plan);
    } else {
      RCLCPP_WARN(get_logger(), "Pick planning failed");
    }
    // place pose
    target.position.x = -0.35;
    move_group_->setPoseTarget(target);
    ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (ok) move_group_->execute(plan);
  }
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PickPlaceMoveIt>());
  rclcpp::shutdown();
  return 0;
}
