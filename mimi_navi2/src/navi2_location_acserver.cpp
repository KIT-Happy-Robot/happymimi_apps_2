// 受け取ったロケーション座標をナビゲーションゴールとして送るサーバー

#include <memory>
#include <chrono>
#include <list>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/time.hpp"

#include "mimi_navi2/srv/navi2_location.hpp" 
//#include "mimi_navi2/action/nav2_loc_ac.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Navi2LocationAcServer : public rclcpp::Node
{
pravate:
  using Nav2Pose = nav2_msgs::action::NavigateToPose;
  using Nav2Loc = mimi_navi2::srv::Navi2Location; // Service
  //using Nav2Loc = mimi_navi2::action::Nav2LocAc; // Action
  using Nav2PoseCGH = rclcpp_action::ClientGoalHandle<Nav2Pose>;
  // ACTION
  rclcpp_action::Client<Nav2Pose>::SharedPtr n2p_client;
  // SERVICE
  rclcpp::Service<Navi2Location>::SharedPtr navi2_srv;
  // clear costmap
  //rclcpp::Service<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr clear_around_srv_;
  // PUBLISHER
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_location_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr head_pub;

  //!!!
  std::list<std::string> location_dict = rclcpp::parameter
  std::string location_name = NULL;

  //void init();
  bool searchLocationName(const std::shared_ptr<Navi2Location> request,
                          const std::shared_ptr<Navi2Location> result
  );
  void sendGoal();
  void naviFeedbackCB(){}
  void naviResultCB(){}

public:
  //Navi2LocationAcServer() : Node("navi2_location_acserver")
  explicit Navi2LocationAcServer(): Node("navi2_location_acserver")
  {
    // ACTION
    this->n2p_client =
      rclcpp_action::create_client<Nav2Pose>(this,"navi2pose_client");

    // SERVICE
    auto navi2_srv = this->create_service<Nav2Loc>
    // PUBLISHer
    auto current_location_pub =
      this->create_publisher<std::msg::String>("/current_location", 1);
    auto head_pub = this->create_publisher<std::msg::Float64>("/servo/head", 1);
    // location_list

  }

  bool searchLocationName(const std::shared_ptr<happymimi_msgs2::srv::NaviLocation> request,
                            const std::shared_ptr<happymimi_msgs2::srv::NaviLocation> result
    ){
      if (request.location_name )
    };


  void sendGoal(location_list) {
    //　wait　for action server
    while (!this->n2p_client->wait_for_action_server()) {
      RCLCPP_INFO(get_logger(), "waiting for the action server...");
      rclcpp::sleep_for(1000ms)
    }
  
    // set goal_pose
    auto goal = Nav2Pose::Goal();
    goal.pose.header.stamp = this->now();
    goal.pose.header.frame_id = "map"; //-------------
    goal.pose.pose.position.x = location_list[0]
    goal.pose.pose.position.y = location_list[1]
    goal.pose.pose.orientation.x = location_list[2]
    goal.pose.pose.orientation.w = location_list[3]
 
    //clear costmap
    //RCLCPP_INFO(get_logger(), "clearing cost map...");
    // rclcpp::wa---------------

  // Feedback
    auto goal_options = rclcpp_action::Client<Nav2Pose>::SendGoalOptions();
    goal_options.feedback_callback = std::bind(&Navi2LocationAcServer::naviFeedbackCB,
                                               this, _1, _2);
    goal_options.result_callback = std::bind(&Navi2LocationAcServer::naviResultCB,
                                             this, 1_);
    // send the goal to server
    n2p_client->async_send_goal(goal, goal_options);
  }
  // !!!
  void naviFeedbackCB(Nav2PoseCGH::SharedPtr,
                      const std::shared_ptr<const Nav2Pose::Feedback feedback){
    
  }

  void naviResultCB(const Nav2PoseCGH::WrappedResult &result){
    switch (result.code){
      RCLCPP_INFO(get.logger(), "Navigation Success"
    }
  }

  void execute(){
  }
}

void execute(){
  rclcpp::rate::Rate rate(33);
  NaviLocationAcServer2 nlas2;
  while (rclcpp::ok()) {

    rclcpp::spin_some(navi2_location_acserver);
    rate.sleep();
  }

  rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
  //execute()

  rclcpp::init(argc, argv);
  auto node = std::make_shared<Navi2LocationAcServer>();
  
  while (rclcpp::ok()) {

    rclcpp::spin_some(navi2_location_acserver);
    rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
