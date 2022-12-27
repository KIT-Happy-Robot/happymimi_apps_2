
#include <memory>
#include <chrono>
#include <iostream>
//#include <list>
#include <vector>
#include <algorithm> // for std::find
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/time.hpp"
#include "mimi_navi2/action/nav2_location_ac.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

//-----------

//-----------


class Navi2LocationAcServer : public rclcpp::Node
{

};

