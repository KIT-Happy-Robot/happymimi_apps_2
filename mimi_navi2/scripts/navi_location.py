#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#-----------------------------------------------------------
# Title: 目的地の名前と座標を設定するサービスサーバー
# Author: Issei Iida
#-----------------------------------------------------------
import rclpy
from rclpy.node import Node
import rosparam
#import actionlib
from std_msgs.msg import String, Float64
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from mimi_navi2.srv import NaviLocation, NaviLocationResponse


class NaviLocationServer(Node):
    def __init__(self):
        super().__init__('navi_location_server')
        service = self.create_service(NaviLocation, 'navi_location_server', self.searchLocationName)
        self.get_logger().info("Ready to navi_location_server")
        # Action
        self.ac = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        # Publisher
        self.crloc_pub = self.create_publisher('/current_location', String, queue_size = 1)
        self.head_pub = self.create_publisher('/servo/head', Float64, queue_size = 1)
        # Service
        self.clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        # Value
        self.location_dict = rosparam.get_param('/location')
        self.location_name = "null"

    def searchLocationName(self, srv_req):
        if srv_req.location_name in self.location_dict:
            self.location_name = srv_req.location_name
            self.get_logger().info(self.location_dict[self.location_name])
            return self.sendGoal(self.location_dict[self.location_name])
        else:
            rospy.logerr("<" + srv_req.location_name + "> doesn't exist.")
            return NaviLocationResponse(result = False)

    def sendGoal(self, location_list):
        # set goal_pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = location_list[0]
        goal.target_pose.pose.position.y = location_list[1]
        goal.target_pose.pose.orientation.z = location_list[2]
        goal.target_pose.pose.orientation.w = location_list[3]
        # clearing costmap
        self.get_logger().info("Clearing costmap...")
        rospy.wait_for_service('move_base/clear_costmaps')
        self.clear_costmap()
        rospy.sleep(0.5)
        # start navigation
        self.head_pub.publish(0)
        self.ac.wait_for_server()
        self.ac.send_goal(goal)
        # self.ac.wait_for_result()
        navi_state = self.ac.get_state()
        while not rospy.is_shutdown():
            navi_state = self.ac.get_state()
            if navi_state == 3:
                self.get_logger().info('Navigation success!!')
                self.crloc_pub.publish(self.location_name)
                return NaviLocationResponse(result = True)
            elif navi_state == 4:
                self.get_logger().info('Navigation Failed')
                return NaviLocationResponse(result = False)
            else:
                pass

if __name__ == '__main__':
    rospy.init_node('navi_location_server', anonymous = True)
    try:
        sls = NaviLocationServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
