#!/usr/bin/env python2
# encoding: utf-8

#line_detect
import rospy
from follow_common import *
import os
import threading
import math
import rospkg
from std_msgs.msg import Bool,Int32,Float32
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from sensor_msgs.msg import CompressedImage, LaserScan, Image
from yahboomcar_linefollw.cfg import LineDetectPIDConfig

#yolov5
import base64
import sys
import time
import rospy
import rospkg
import cv2 as cv
from yahboomcar_msgs.msg import *
from yolov5_trt import YoLov5TRT
import numpy as np
from time import sleep


RAD2DEG = 180 / math.pi

class LineDetect:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        rospy.init_node("LineDetect", anonymous=False)
        self.img = None
        self.circle = ()
        self.hsv_range = ()
        self.Roi_init = ()
        self.warning = 1
        self.Start_state = True
        self.dyn_update = False
        self.Buzzer_state = False
        self.select_flags = False
        self.Track_state = 'identify'
        self.windows_name = 'frame'
        self.trun_angular_z = 0.0
        self.go_angular_z = 0.0
        self.ros_ctrl = ROSCtrl()
        self.color = color_follow()
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.img_flip = rospy.get_param("~img_flip", False)
        #self.VideoSwitch = rospy.get_param("~VideoSwitch", False)
        self.VideoSwitch = rospy.get_param("~VideoSwitch", True)
        self.hsv_text = rospkg.RosPack().get_path("yahboomcar_autodrive")+"/scripts/LineFollowHSV.text"
        #Server(LineDetectPIDConfig, self.dynamic_reconfigure_callback)
        #self.dyn_client = Client("LineDetect", timeout=60)
        self.scale = 1000
        self.FollowLinePID = (60, 0, 20)
        #print("----------------------------------------------")
        self.linear = 0.2
        self.LaserAngle = 30
        self.ResponseDist = 0.00
        self.PID_init()
        #self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.registerScan, queue_size=1)
        #self.pub_rgb = rospy.Publisher("/linefollw/rgb", Image, queue_size=1)
        self.pub_Buzzer = rospy.Publisher('/Buzzer', Bool, queue_size=1)
        self.pub_go_z = rospy.Publisher("/go_z",Float32,queue_size=1)
        self.pub_turn_z = rospy.Publisher("/turn_z",Float32,queue_size=1)
        self.subMove_Status = rospy.Subscriber('move_flag',Int32,self.Move_Statuscallback)
        self.flag = 1
        #self.bridge = CvBridge()
        self.go_z = Float32()
        self.turn_z = Float32()
        
        
        #yolov5_init
        self.pTime = self.cTime = 0
        device = rospy.get_param("~device", "nx")
        param_ = rospkg.RosPack().get_path("yahboomcar_yolov5") + '/param/' 
        #file_yaml = param_ + 'coco.yaml'
        file_yaml = param_ + 'traffic.yaml'
        PLUGIN_LIBRARY = param_ + device + "/libmyplugins.so"
        engine_file_path = param_ + device + "/yolov5s.engine"
        self.yolov5_wrapper = YoLov5TRT(file_yaml, PLUGIN_LIBRARY, engine_file_path)
        self.pub_image = rospy.Publisher('Detect/image_msg', Image_Msg, queue_size=1)
        self.pub_msg = rospy.Publisher('DetectMsg', TargetArray, queue_size=1)
        self.target_array = TargetArray()
        self.target = Target()
        
    #yolov5_detect
    def detect(self,detect_img):
        detect_img, result_boxes, result_scores, result_classid = self.yolov5_wrapper.infer(detect_img)
    # Draw rectangles and labels on the original image
        for j in range(len(result_boxes)):
            box = result_boxes[j]
            self.yolov5_wrapper.plot_one_box(
                box,
                detect_img,
                label="{}:{:.2f}".format(
                    self.yolov5_wrapper.categories[int(result_classid[j])],
                    result_scores[j]
                ),
            )
            self.target.frame_id = self.yolov5_wrapper.categories[int(result_classid[j])]
            self.target.stamp = rospy.Time.now()
            self.target.scores = result_scores[j]
            # x1, y1, x2, y2
            self.target.ptx = box[0]
            self.target.pty = box[1]
            self.target.distw = box[2] - box[0]
            self.target.disth = box[3] - box[1]
            self.target.centerx = (box[2] - box[0]) / 2
            self.target.centery = (box[3] - box[1]) / 2
            self.target_array.data.append(self.target)
        self.cTime = time.time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        text = "FPS : " + str(int(fps))
        cv.putText(detect_img, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        self.pub_msg.publish(self.target_array)
        #self.pub_imgMsg(frame)
        #print("target.centerx: ",self.target.centerx)
        if self.target.frame_id =="Turn_right":
            print("target.centery: ",self.target.centery) 
        action = cv.waitKey(1) & 0xFF
        #self.img = self.detect(self.img)
        cv.imshow("yolov5_frame", detect_img)
        cv.waitKey(1)  

    def cancel(self):
        self.Reset()
        self.ros_ctrl.cancel()
        
        #self.sub_img.unregister()
        #self.pub_rgb.unregister()
        self.pub_Buzzer.unregister()
        print ("Shutting down this node.")
        if self.VideoSwitch==False:
            #self.sub_img.unregister()
            cv.destroyAllWindows()

    def Move_Statuscallback(self,msg):
        self.flag = msg.data
        
    def compressed_callback(self, msg):
        if not isinstance(msg, CompressedImage): return
        start = time.time()
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        action = cv.waitKey(1) & 0xFF
        rgb_img, binary = self.process(frame)
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(rgb_img, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        if len(binary) != 0: cv.imshow(self.windows_name, ManyImgs(1, ([rgb_img, binary])))
        else: cv.imshow(self.windows_name, rgb_img)
        #self.pub_rgb.publish(self.bridge.cv2_to_imgmsg(rgb_img, "bgr8"))

    def process(self, rgb_img):
        binary = []
        rgb_img = cv.resize(rgb_img, (640, 480))
        #self.pub_rgb.publish(self.bridge.cv2_to_imgmsg(rgb_img, "bgr8"))
        if os.path.exists(self.hsv_text): self.hsv_range = read_HSV(self.hsv_text)
        if len(self.hsv_range) != 0:
            rgb_img, binary, self.circle = self.color.line_follow(rgb_img, self.hsv_range)
        if len(self.circle) != 0:
            threading.Thread(target=self.execute, args=(self.circle[0], self.circle[2])).start()
        return rgb_img, binary

    def execute(self, point_x, color_radius):
        if color_radius == 0: print("stop")
        else:
            twist = Twist()
            b = Bool()
            if self.flag == 1:
                center_x = 320-point_x
                #print(point_x)
                if point_x<320  :
                    point_x = point_x
                    [z_Pid, _] = self.PID_controller.update([(point_x - 30)*1.8/32,0])
                    if self.img_flip == True: self.go_angular_z = -z_Pid 
                    else: self.go_angular_z = +z_Pid

                elif point_x>320 or point_x==320:
                    [z_Pid, _] = -self.PID_controller.update([(610-point_x)*1.8/32, 0])#16
                    if self.img_flip == True: self.go_angular_z = -z_Pid 
                    else: self.go_angular_z = +z_Pid
            
                #print("z_pid" ,z_Pid)
                #print("go_angular_z: ",self.go_angular_z)
                if abs(self.go_angular_z)<0.1:
                    if self.img_flip == True:self.go_angular_z = -self.go_angular_z
                    else:
                        self.go_angular_z = +self.go_angular_z
                    #self.go_angular_z = 0
             
                    
                self.go_z.data = self.go_angular_z 
                self.pub_go_z.publish(self.go_z)
                

            elif self.flag == 2:
                [z_Pid, _] = self.PID_controller.update([(point_x - 320)*1.0/32, 0])
                print("zpid: ")
                if self.img_flip == True: self.trun_angular_z = -z_Pid #-z_Pid
                else: self.trun_angular_z = +z_Pid
                if abs(twist.angular.z)>1:
                    self.trun_angular_z = 1.0
                else:
                    self.trun_angular_z = abs(self.trun_angular_z)
                self.turn_z = self.trun_angular_z
                self.pub_turn_z.publish(self.turn_z)


    def Reset(self):
        self.PID_init()
        self.Track_state = 'init'
        self.hsv_range = ()
        self.ros_ctrl.Joy_active =False
        self.Mouse_XY = (0, 0)
        self.ros_ctrl.pub_cmdVel.publish(Twist())
        rospy.loginfo("Reset succes!!!")

    def PID_init(self):
        self.PID_controller = simplePID(
            [0, 0],
            [self.FollowLinePID[0] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[1] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[2] / 1.0 / (self.scale), 0])


if __name__ == '__main__':
    line_detect = LineDetect()
    if line_detect.VideoSwitch==False:rospy.spin()
    else:
        capture = cv.VideoCapture(0)
        cv_edition = cv.__version__
        if cv_edition[0]=='3': capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
        else: capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        while capture.isOpened():
            start = time.time()
            ret, frame = capture.read()
            action = cv.waitKey(10) & 0xFF
            detect_img = frame.copy()
            frame, binary = line_detect.process(frame)
            #threading.Thread(target=line_detect.detect, args=detect_img).start()
            line_detect.detect(detect_img)
            #line_detect.pub_rgb.publish(line_detect.bridge.cv2_to_imgmsg(frame, "bgr8"))
            
             
            end = time.time()
            fps = 1 / (end - start)
            text = "FPS : " + str(int(fps))
            cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
            if len(binary) != 0: cv.imshow('frame', ManyImgs(1, ([frame, binary])))
            else:cv.imshow('frame', frame)
            if action == ord('q') or action == 113: break
        capture.release()
        cv.destroyAllWindows()
