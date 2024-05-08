import cv2
import torch
import numpy as np
from tracker import *
import threading
import time
import sys
import serial

# Load YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
ser = serial.Serial('com9', 9600)
ambulance = 0
t=0
def switch(lane, gst):
    global ambulance,t
    print("amb ",ambulance,"t",t)
    if lane == 1:
        ser.write(b'a')  # Send character 'a' to start LED operation
    elif lane == 2:
        ser.write(b'b')
    elif lane == 3:
        ser.write(b'c')
    else:
        ser.write(b'd')
    time.sleep(0.1)  # Add a small delay (optional)
    ser.write(str(gst).encode() + b'\n')
    time.sleep(gst)

def rfid_detection():
    global ambulance
    while True:
        serial_data = ser.readline().decode().strip()
        if serial_data == "Tag Detected":
            if ambulance==0:
                print("RFID Tag Detected!")
            ambulance = 1
            
prev1 = 0
prev2 = 0
prev3 = 0
prev4 = 0
lane = 1
f = 0

cap1 = cv2.VideoCapture('video4.mp4')
cap2 = cv2.VideoCapture('video4.mp4')
cap3 = cv2.VideoCapture('video4.mp4')
cap4 = cv2.VideoCapture('video4.mp4')

area1 = [(175, 425), (175, 450), (630, 450), (630, 425)]
area2 = [(175, 425), (175, 450), (630, 450), (630, 425)]
area3 = [(175, 425), (175, 450), (630, 450), (630, 425)]
area4 = [(175, 425), (175, 450), (630, 450), (630, 425)]

area_1 = set()
area_2 = set()
area_3 = set()
area_4 = set()

tracker = Tracker()
combined_frame = np.zeros((620, 680, 3), dtype=np.uint8)
stop_threads = False

def process_frame(cap, area_1, area1, number):
    global f, lane, stop_threads, prev1, prev2, prev3, prev4,ambulance,t
    while not stop_threads:
        ret, frame = cap.read()
        if not ret:
            stop_threads = True
            break

        frame = cv2.resize(frame, (640, 480))

        results = model(frame)

        bbox_list = []
        for obj in results.xyxy[0]:
            x1, y1, x2, y2, conf, cls = obj.tolist()
            bbox_list.append([int(x1), int(y1), int(x2), int(y2)])

        idx_bbox = tracker.update(bbox_list)

        for bbox in idx_bbox:
            x2, y2, x3, y3, id = bbox
            cv2.rectangle(frame, (x2, y2), (x3, y3), (0, 0, 255), 2)
            cv2.circle(frame, (x3, y3), 4, (0, 255, 0), -1)
            result = cv2.pointPolygonTest(np.array(area1, np.int32), ((x3, y3)), False)

            if result > 0:
                area_1.add(id)

        cv2.putText(frame, str(len(area_1)), (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        cv2.polylines(frame, [np.array(area1, np.int32)], True, (0, 255, 255), 2)
        if number == 1:
            frame = cv2.resize(frame, (340, 300))
            combined_frame[:300, :340] = frame
        elif number == 2:
            frame = cv2.resize(frame, (340, 300))
            combined_frame[:300, 340:680] = frame

        elif number == 3:
            frame = cv2.resize(frame, (340, 300))
            combined_frame[320:620, :340] = frame

        else:
            frame = cv2.resize(frame, (340, 300))
            combined_frame[320:620, 340:680] = frame

        cv2.imshow("FRAME", combined_frame)
        if cv2.waitKey(1) & 0xFF == 27:
            stop_threads = True
            break
        if t==1 and number==1:
            f=1
            curr = len(area_1)
            temp = curr
            curr = curr - prev1
            prev1 = temp
            gst = curr // 5
            gst = gst + 10
            print(1, "has ambulance is green for ", gst, "seconds")
            switch(1, gst)
            ambulance=0
            t=0
            if(lane==1):
                lane=lane+1
            f=0
            
        elif f == 0 and number == lane:
            if ambulance == 1 and lane!=1:
                t=1
            else:
                f = 1
                curr = len(area_1)
                temp = curr
                gst = 4
                if lane == 1:
                    curr = curr - prev1
                    prev1 = temp
                    gst = curr // 5
                    ambulance = 0

                elif lane == 2:
                    curr = curr - prev2
                    prev2 = temp
                    gst = curr // 5

                elif lane == 3:
                    curr = curr - prev3
                    prev3 = temp
                    gst = curr // 5

                else:
                    curr = curr - prev4
                    prev4 = temp
                    gst = curr // 5
                gst = gst + 10
                print(lane, "is green for ", gst, "seconds")
                switch(lane, gst)
                lane = 1 + lane
                if lane == 5:
                    lane = 1
                f = 0
    cap.release()
    sys.exit()

# Create threads for processing frames and RFID detection
thread1 = threading.Thread(target=process_frame, args=(cap1, area_1, area1, 1))
thread2 = threading.Thread(target=process_frame, args=(cap2, area_2, area2, 2))
thread3 = threading.Thread(target=process_frame, args=(cap3, area_3, area3, 3))
thread4 = threading.Thread(target=process_frame, args=(cap4, area_4, area4, 4))
rfid_thread = threading.Thread(target=rfid_detection)

# Start threads
thread1.start()
thread2.start()
thread3.start()
thread4.start()
rfid_thread.start()

# Wait for all threads to finish
thread1.join()
thread2.join()
thread3.join()
thread4.join()
rfid_thread.join()

# Destroy OpenCV windows
cv2.destroyAllWindows()