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

#まだ変更していない
from enter_room.srv import EnterRoom, EnterRoomResponse
from happymimi_msgs2.srv import StrTrg
from happymimi_teleop.base_control import BaseControl   #このまま使えるかも？


class EnterRoomServer:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('enter_room_server')
        # サービスの作成
        self.service = self.node.create_service(EnterRoom, '/enter_room_server', self.execute)
        # ログを取得
        self.logger = self.node.get_logger()

        # speak
        self.tts_srv = self.node.create_client(StrTrg, '/tts')

        # Subscriber
        self.subscription = self.node.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        # Module
        self.base_control = BaseControl()

        # Value
        self.front_laser_dist = 999.9

    def laser_callback(self, receive_msg):
        self.front_laser_dist = receive_msg.ranges[359]

    def execute(self, request, response):
        try:
            safe_dist = 1.0
            target_dist = request.distance + self.front_laser_dist

            # speakサービスの呼び出し
            self.tts_srv.wait_for_service()
            self.tts_srv.call(StrTrg.Request(text='Prease open the door'))

            self.logger.info("Prease open the door")
            while self.front_laser_dist <= safe_dist:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            # speakサービスの呼び出し
            self.tts_srv.call(StrTrg.Request(text='Thank you'))
            self.base_control.translateDist(target_dist, request.velocity)
            response.result = True
        except KeyboardInterrupt:
            self.logger.info("!!Interrupt!!")
            response.result = False
        return response

def main(args=None):
    rclpy.init(args=args)
    enter_room_server = EnterRoomServer()
    rclpy.spin(enter_room_server.node)

if __name__ == '__main__':
    main()