#!/usr/bin/env python2
# coding:utf-8
from Turn_Right import *
from std_msgs.msg import Int32,Float32
from geometry_msgs.msg import Twist
from yahboomcar_msgs.msg import *
from follow_common import *
import os
import sys
class Turn():
    def __init__(self):
        self.sub_traffic_sign = rospy.Subscriber('DetectMsg', TargetArray, self.excute, queue_size=1)
        self.turn_right = YahboomCarPatrol()
        #self.sub_reverse = rospy.Subscriber('/test_reverse', Int32, self.Test)
        self.pubFlag = rospy.Publisher('move_flag',Int32,queue_size=1)
        #self.subGo_z = rospy.Subscriber('/go_z', Float32, self.GetGoZCallback)
        #self.subTurn_z = rospy.Subscriber('/turn_z', Float32, self.GetTurnZCallback)
        #self.line_detect = LineDetect()
        self.dist = 0.0
        self.move_flag  = 1 #1-Go,2-TurnRight
        self.move_status = Int32()
        #self.line_detect.VideoSwitch = False
        print("9999")
        #self.trun_angular_z = 0
        #self.go_angular_z = 0
        
    def GetGoZCallback(self,msg):
        self.go_angular_z = msg.data
        
    def GetTurnZCallback(self,msg):
        self.trun_angular_z = msg.data

    def excute(self,msg):
        #print("----")
        '''if msg.data != 1:
            self.turn_right.process()
        elif msg.data ==1 or msg.data ==0 :
            self.turn_right.Go()'''
        #print(msg.data[1].frame_id)
        #print(msg.data[1].frame_id)
        #print(msg.data)
        if msg.data == [] or msg.data[0].frame_id == "Green_light":
            print("Go Now")
            self.move_status.data = 1
            self.pubFlag.publish(self.move_status)
            self.turn_right.Go(self.turn_right.go_angular_z) 
            self.turn_right.finish_flag = 0
        elif msg.data[0].frame_id == "Turn_right":
            self.move_status.data = 2
            self.pubFlag.publish(self.move_status)
            if msg.data[0].centery>44 and self.turn_right.finish_flag == 0 : #如果改动了摄像头的位置高度，则这里的y值需要标定
                self.dist = msg.data[0].centerx * 0.0142
                print("self.dist: ",self.dist)
                #print("Turn Now")
                print("self.trun_angular_z: ",self.turn_right.trun_angular_z)
                self.turn_right.Stop()
                sleep(5)
                self.turn_right.process(self.dist,self.turn_right.trun_angular_z)
            else:
                print("Adjust")
                self.move_status.data = 1
                self.pubFlag.publish(self.move_status)
                #self.turn_right.finish_flag = 0
                self.pubFlag.publish(self.move_status)
                self.turn_right.Go(self.turn_right.go_angular_z)
        elif msg.data[0].frame_id == "Parking_lotB":
            print("reversing")
            os.system("rosbag play 2022-11-10-18-10-32.bag")
            os._exit(0)
        elif msg.data[0].frame_id == "Red_light":
            self.turn_right.Stop()
            
            
if __name__ == '__main__':
    rospy.init_node("turn_node", anonymous=False)
    turn = Turn()
    #line_detect = LineDetect()
    #trun_right.process()
    rospy.spin()
    print(6666)
