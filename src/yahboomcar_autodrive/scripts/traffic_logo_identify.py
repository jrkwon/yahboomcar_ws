#!/usr/bin/env python3
# coding:utf-8
import cv2
import torch
import numpy as np
from numpy import random
from utils.plots import plot_one_box
from models.experimental import attempt_load
from utils.general import (non_max_suppression, scale_coords, xyxy2xywh)
from utils.torch_utils import select_device, time_synchronized
import rospy
from std_msgs.msg import Int32
from time import sleep
from follow_common import *
device = select_device()
model_path = './best.pt'
model = attempt_load(model_path, map_location=device)
names = model.module.names if hasattr(model, 'module') else model.names
colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]
import threading
class Traffic_Identify:
    def __init__(self):
        self.TrafficPublisher = rospy.Publisher('traffic_logo', Int32, queue_size=1)
        
        self.result = Int32()
        self.num = 0
        self.FollowLinePID = (60, 0, 20)
        self.scale = 1000
        self.PID_init()
        self.ros_ctrl = ROSCtrl()
        self.thread_lock = threading.Lock()
        self.finish = False
        self.status = "Identify"
    def PID_init(self):
        self.PID_controller = simplePID(
            [0, 0],
            [self.FollowLinePID[0] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[1] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[2] / 1.0 / (self.scale), 0])
        
    def classify(self,str):
        if str == "Go_straight":
            self.result.data = 1
        elif str == "Turn_right":
            self.result.data = 2
            
        elif str == "Beep":
            self.result.data = 3
        elif str == "Sidewalk":
            self.result.data = 4
        elif str == "Stop":
            self.result.data = 5
        elif str == 'Slow_down':
            self.result.data = 6
        elif str == "Parking_lot":
            self.result.data = 7
        elif str == "School_decelerate":
            self.result.data = 8
        else:
            self.result.data = 0
        #print(self.result.data)
        self.TrafficPublisher.publish(self.result)
            
    def process(self,frame):
        
        sleep(0.02)
        img = frame.copy()
        img = np.transpose(img, (2, 0, 1))
        img = torch.from_numpy(img).to(device)
        img = img.float()
        img /= 255.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        pred = model(img)[0]
        # Apply NMS
        pred = non_max_suppression(pred, 0.4, 0.5)
        gn = torch.tensor(frame.shape)[[1, 0, 1, 0]]
        if self.status == "Identify":      
            if pred != [None]:
                for i, det in enumerate(pred):
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], frame.shape).round()
                    for *xyxy, conf, cls in reversed(det):
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()
                        #label = '%s ' % names[int(cls)]
                        label = names[int(cls)]
                        print(label)
                        #
                        conf = '%.2f' % conf
                        #b = xyxy.reshape(-1)
                        x1y1x2y2 = torch.tensor(xyxy).view(1, 4)
                        a = x1y1x2y2[0][0].item()
                        b = x1y1x2y2[0][1].item()
                        c = x1y1x2y2[0][2].item()
                        d = x1y1x2y2[0][3].item()
                        self.mid_x = (a+c)/2
                        self.mid_y = (b+d)/2
                        print("mid_x:",self.mid_x)
                        print("mid_y:",self.mid_y)
                        if( self.mid_y> 179 and self.mid_y<182):#223
                            plot_one_box(xyxy, frame, label=label, color=colors[int(cls)], line_thickness=3)
                            if label == "Turn_right":
                                self.status = "Excute"
                                #threading.Thread(target=self.execute, args=(self.mid_x,)).start()
            self.TrafficPublisher.publish(1)
            
        elif self.status == "Excute":
            threading.Thread(target=self.execute, args=(self.mid_x,)).start()
            self.status = "Identify"
        cv2.imshow("frame", frame)
        
    def execute(self,x_set):
        #self.thread_lock.acquire()
        twist = Twist()
        [z_Pid, _] = self.PID_controller.update([(320-x_set)*1.0/6, 0]) 
        if x_set>160:
            twist.angular.z = z_Pid *2
        else:
            twist.angular.z = z_Pid *0.5
        print("z: ",twist.angular.z)
        twist.linear.x = 0.4
        self.ros_ctrl.pub_cmdVel.publish(twist)
        #self.thread_lock.release()
        sleep(1.5)
        twist.angular.z = 0
        twist.linear.x = 0
        self.ros_ctrl.pub_cmdVel.publish(twist)
        
        
       
        
if __name__ == '__main__':   
    rospy.init_node("traffic_identify_node", anonymous=False)
    traffic_identify = Traffic_Identify()
    cap = cv2.VideoCapture(0)
    
    while cap.isOpened():
        _, frame = cap.read()
        traffic_identify.process(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
    cap.release()
    cv2.destroyAllWindows()



'''cap = cv2.VideoCapture(0)
traffic_logo = 0 
def classify(str):
    print(1111)
    if str == "Go_straight":
        return 1
    elif str == "Turn_right":
        return 2
    elif str == "Beep":
        return 3
    elif str == "Sidewalk":
        return 4
    elif str == "Stop":
        return 5
    elif str == 'Slow_down':
        return 6
    elif str == "Parking_lot":
        return 7
    elif str == "School_decelerate":
        return 8
while cap.isOpened():
    _, frame = cap.read()
    img = frame.copy()
    img = np.transpose(img, (2, 0, 1))
    img = torch.from_numpy(img).to(device)
    img = img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)
    # Inference
    pred = model(img)[0]
    # Apply NMS
    pred = non_max_suppression(pred, 0.4, 0.5)
    gn = torch.tensor(frame.shape)[[1, 0, 1, 0]]
    if pred != [None]:
        # Process detections
        for i, det in enumerate(pred):  # detections per image
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], frame.shape).round()
            # Write results
            for *xyxy, conf, cls in reversed(det):
                xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                label = '%s ' % names[int(cls)]
                print(label)
                plot_one_box(xyxy, frame, label=label, color=colors[int(cls)], line_thickness=3)
                result.data = classify(label)
                print(result.data)
    else:
        result = 0    
    #TrafficPublisher.publish()
    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break
cap.release()
cv2.destroyAllWindows()'''
