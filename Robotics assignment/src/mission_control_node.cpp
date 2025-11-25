// Simplified mission control node (same as earlier scaffold) with comments.
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class MissionControl : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  MissionControl(): Node("mission_control") {
    emergency_sub_ = this->create_subscription<std_msgs::msg::Bool>("/emergency_stop", 10, std::bind(&MissionControl::emergency_callback, this, std::placeholders::_1));
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    pick_client_ = this->create_client<std_srvs::srv::Trigger>("trigger_pick");
    place_client_ = this->create_client<std_srvs::srv::Trigger>("trigger_place");
    timer_ = this->create_wall_timer(1500ms, std::bind(&MissionControl::tick, this));
    state_ = State::IDLE;
  }
private:
  enum class State { IDLE, NAV_TO_PICK, PICK, NAV_TO_PLACE, PLACE, FINISHED, RECOVERY };
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pick_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr place_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  State state_;
  bool emergency_{false};
  geometry_msgs::msg::PoseStamped pick_pose_, place_pose_;

  void emergency_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    emergency_ = msg->data;
    if (emergency_) state_ = State::RECOVERY;
    else state_ = State::IDLE;
  }

  void tick() {
    switch(state_) {
      case State::IDLE:
        pick_pose_.header.frame_id = "map";
        pick_pose_.pose.position.x = 1.0;
        pick_pose_.pose.position.y = 0.0;
        pick_pose_.pose.orientation.w = 1.0;
        place_pose_.header.frame_id = "map";
        place_pose_.pose.position.x = -1.0;
        place_pose_.pose.position.y = 0.0;
        place_pose_.pose.orientation.w = 1.0;
        RCLCPP_INFO(this->get_logger(), "STATE IDLE -> NAV_TO_PICK");
        state_ = State::NAV_TO_PICK;
        break;
      case State::NAV_TO_PICK:
        if (emergency_) { state_ = State::RECOVERY; break; }
        if (send_nav_goal(pick_pose_)) state_ = State::PICK;
        else state_ = State::RECOVERY;
        break;
      case State::PICK:
        if (emergency_) { state_ = State::RECOVERY; break; }
        if (call_pick_service()) state_ = State::NAV_TO_PLACE; else state_ = State::RECOVERY;
        break;
      case State::NAV_TO_PLACE:
        if (emergency_) { state_ = State::RECOVERY; break; }
        if (send_nav_goal(place_pose_)) state_ = State::PLACE; else state_ = State::RECOVERY;
        break;
      case State::PLACE:
        if (emergency_) { state_ = State::RECOVERY; break; }
        if (call_place_service()) state_ = State::FINISHED; else state_ = State::RECOVERY;
        break;
      case State::FINISHED:
        RCLCPP_INFO(this->get_logger(), "Mission finished. Back to IDLE."); state_ = State::IDLE;
        break;
      case State::RECOVERY:
        RCLCPP_WARN(this->get_logger(), "Recovery: performing safe reset"); state_ = State::IDLE;
        break;
    }
  }

  bool send_nav_goal(const geometry_msgs::msg::PoseStamped & pose) {
    if (!nav_client_->wait_for_action_server(2s)) { RCLCPP_ERROR(this->get_logger(), "Nav action not available"); return false; }
    auto goal = NavigateToPose::Goal(); goal.pose = pose;
    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    auto future = nav_client_->async_send_goal(goal, options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 30s) != rclcpp::FutureReturnCode::SUCCESS) return false;
    auto gh = future.get();
    if (!gh) return false;
    auto result_future = nav_client_->async_get_result(gh);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, 90s) == rclcpp::FutureReturnCode::SUCCESS) {
      return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
    }
    return false;
  }

  bool call_pick_service() {
    if (!pick_client_->wait_for_service(2s)) return false;
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = pick_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut, 10s) != rclcpp::FutureReturnCode::SUCCESS) return false;
    auto res = fut.get();
    return res->success;
  }

  bool call_place_service() {
    if (!place_client_->wait_for_service(2s)) return false;
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = place_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut, 10s) != rclcpp::FutureReturnCode::SUCCESS) return false;
    auto res = fut.get();
    return res->success;
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionControl>());
  rclcpp::shutdown();
  return 0;
}
