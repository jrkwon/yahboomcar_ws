#!/usr/bin/env python3
# encoding: utf-8
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
class YoloDetect:
    def __init__(self):
        #rospy.on_shutdown(self.cancel)
        rospy.init_node("YoloDetect", anonymous=False)
        self.pTime = self.cTime = 0
        device = rospy.get_param("~device", "tx2nx")
        param_ = rospkg.RosPack().get_path("yahboomcar_yolov5") + '/param/' 
        #file_yaml = param_ + 'coco.yaml'
        file_yaml = param_ + 'traffic.yaml'
        PLUGIN_LIBRARY = param_ + device + "/libmyplugins.so"
        engine_file_path = param_ + device + "/yolov5s.engine"
        self.yolov5_wrapper = YoLov5TRT(file_yaml, PLUGIN_LIBRARY, engine_file_path)
        self.pub_image = rospy.Publisher('Detect/image_msg', Image_Msg, queue_size=1)
        self.pub_cam_image = rospy.Publisher('Camera/image_msg', Image_Msg, queue_size=1)
        self.pub_msg = rospy.Publisher('DetectMsg', TargetArray, queue_size=1)
        
        self.image_sub = rospy.Subscriber("/image_data",Image_Msg,self.image_sub_callback)
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)  # 初始图像
        self.target_array = TargetArray()
        self.target = Target()

    def cancel(self):
        self.pub_image.unregister()
        self.pub_cam_image.unregister()
        self.pub_msg.unregister()
        self.yolov5_wrapper.destroy()
        
    def image_sub_callback(self, data):
        ''' callback of image_sub '''
        #image = np.ndarray(shape=(data.height, data.width, data.channels), dtype=np.uint8, buffer=data.data) # 将自定义图像消息转化为图像
        image = np.ndarray(shape=(data.height, data.width, data.channels), dtype=np.uint8, buffer=data.data) # 将自定义图像消息转化为图像
        self.img[:,:,0],self.img[:,:,1],self.img[:,:,2] = image[:,:,2],image[:,:,1],image[:,:,0] # 将rgb 转化为opencv的bgr顺序
        self.detect(self.img)
        

    def pub_imgMsg(self, frame):
        pic_base64 = base64.b64encode(frame)
        image = Image_Msg()
        size = frame.shape
        image.height = size[0]
        image.width = size[1]
        image.channels = size[2]
        image.data = pic_base64
        self.pub_image.publish(image)
        
    def pub_cam_imgMsg(self, frame):
        pic_base64 = base64.b64encode(frame)
        image = Image_Msg()
        size = frame.shape
        image.height = size[0]
        image.width = size[1]
        image.channels = size[2]
        image.data = pic_base64
        self.pub_cam_image.publish(image)

    def detect(self,frame):
        #self.pub_cam_imgMsg(frame)
        #target_array = TargetArray()
        #target = Target()
        frame, result_boxes, result_scores, result_classid = self.yolov5_wrapper.infer(frame)
        # Draw rectangles and labels on the original image
        for j in range(len(result_boxes)):
            box = result_boxes[j]
            self.yolov5_wrapper.plot_one_box(
                box,
                frame,
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
        '''box = result_boxes[0]
        self.yolov5_wrapper.plot_one_box(
		box,
		frame,
		label="{}:{:.2f}".format(
			self.yolov5_wrapper.categories[int(result_classid[0])],
			result_scores[0]
		),
		)
        self.target.frame_id = self.yolov5_wrapper.categories[int(result_classid[0])]
        self.target.stamp = rospy.Time.now()
        self.target.scores = result_scores[0]
		# x1, y1, x2, y2
        self.target.ptx = box[0]
        self.target.pty = box[1]
        self.target.distw = box[2] - box[0]
        self.target.disth = box[3] - box[1]
        self.target.centerx = (box[2] - box[0]) / 2
        self.target.centery = (box[3] - box[1]) / 2
        self.target_array.data.append(self.target)'''
        self.cTime = time.time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        self.pub_msg.publish(self.target_array)
        #self.pub_imgMsg(frame)
        #print("target.centerx: ",self.target.centerx)
        #print("target.centery: ",self.target.centery) 
        action = cv.waitKey(1) & 0xFF
        #self.img = self.detect(self.img)
        cv.imshow("frame", frame)
        cv.waitKey(1)
        #return frame

if __name__ == "__main__":
    print("Python version: ", sys.version)
    '''capture = cv.VideoCapture(0)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))'''
    detect = YoloDetect()
    #rospy.sleep(10)
    '''while not rospy.is_shutdown:
        frame =  detect.detect(detect.img)
        action = cv.waitKey(1) & 0xFF      
        cv.imshow("frame", frame)'''
    rospy.spin()
    '''while capture.isOpened():
        ret, frame = capture.read()
        detect.pub_cam_mgMsg(frame)
        action = cv.waitKey(1) & 0xFF
        frame = detect.detect(frame)
        if action == ord('q'): break
        if len(sys.argv) != 1:
            if sys.argv[1]=="true" or sys.argv[1]=="True": cv.imshow('frame', frame)
        else:cv.imshow('frame', frame)
    detect.cancel()
    capture.release()
    cv.destroyAllWindows()'''
