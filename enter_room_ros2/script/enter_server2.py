#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#-------------------------------------------------------------
# Title: ドアが開いたことを検出して入室するServiceServer

# Memo: ドアが閉まっていることを前提条件とする
# サービスで「進む距離」と「速度」を指定できる
#-------------------------------------------------------------

import rclpy
import sys
from sensor_msgs.msg import LaserScan



class EnterRoomServer():    #ros2用に変更したものを順次書き込む
    def __init__(self):
    #     service = rclpy.Service('/enter_room_server', EnterRoom, self.execute)
        rclpy.loginfo("Ready to set enter_room_server")
        # speak
    #    self.tts_srv = rclpy.ServiceProxy('/tts', StrTrg)
        #Subscriber
        rclpy.Subscriber('/scan', LaserScan, self.laserCB)
    #     # Module
    #     self.base_control = BaseControl()
        #Value
        self.front_laser_dist = 999.9

    def laserCB(self, receive_msg):
        self.front_laser_dist = receive_msg.ranges[359]

    def execute(sef,srv_req):
        #try:
            #safe_dist = 1.0
            #target_dist = srv_req.distance + self.front_laser_dist
            #self.tts_srv('Prease open the door')
            print("Prease open the door")
        #     while self.front_laser_dist <= safe_dist:
        #         rclpy.sleep(0.1)
        #     self.tts_srv('Thank you')
        #     self.base_control.translateDist(target_dist, srv_req.velocity)
        #     return EnterRoomResponse(result = True)
        # except rclpy.ROSInterruptException:
        #     rclpy.logger("!!Interrupt!!")
        #     return EnterRoomResponse(result = False)

if __name__ == '__main__':
    rclpy.init_node('enter_server')
    ers = EnterRoomServer()
    rclpy.spin()
