
#!/usr/bin/python 
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import socket
import io
import sys
import struct
from PIL import Image
from multiprocessing import Process
from Command import COMMAND as cmd
import yolov5
import os
import time

class VideoStreaming:
    def __init__(self):
        self.face_cascade = cv2.CascadeClassifier(r'haarcascade_frontalface_default.xml')
        self.video_Flag=True
        self.connect_Flag=True
        self.face_x=0
        self.face_y=0
        self.lights = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80]
        #new
        self.Wheel_Flag = 1
        self.Rotate_Flag = 1
        self.intervalChar = '#'
        self.endChar = '\n'
        self.sequence = [1, 0, 2, 3]  # Red, Green, Blue, Yellow
        self.sequence_index = 0  # Track current position in the sequence
        self.backup = False #+Boundary Detect indicator +change B to I
        self.ignorebackup = False
        self.backupignorecnt = 0

    def StartTcpClient(self,IP):
        self.client_socket1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    def StopTcpcClient(self):
        try:
            self.client_socket.shutdown(2)
            self.client_socket1.shutdown(2)
            self.client_socket.close()
            self.client_socket1.close()
        except:
            pass
    
    def IsValidImage4Bytes(self,buf): 
        bValid = True
        if buf[6:10] in (b'JFIF', b'Exif'):     
            if not buf.rstrip(b'\0\r\n').endswith(b'\xff\xd9'):
                bValid = False
        else:        
            try:  
                Image.open(io.BytesIO(buf)).verify() 
            except:  
                bValid = False
        return bValid
    def light_led(self, R, G, B):
            color = '#' + str(R) + '#' + str(G) + '#' + str(B) 
            for light in self.lights:
                self.sendData(cmd.CMD_LED + '#' + str(light) + color + '\n')
    def forward(self):
        if self.Wheel_Flag:
            if self.Rotate_Flag:
                M_ForWard = self.intervalChar + str(0) + self.intervalChar + str(600) + self.intervalChar + str(
                    0) + self.intervalChar + str(0) + self.endChar
                self.sendData(cmd.CMD_M_MOTOR + M_ForWard)
            else:
                R_ForWard = self.intervalChar + str(0) + self.intervalChar + str(0) + self.intervalChar + str(
                    0) + self.intervalChar + str(600) + self.endChar
                self.sendData(cmd.CMD_CAR_ROTATE + R_ForWard)
        else:
            ForWard = self.intervalChar + str(600) + self.intervalChar + str(600) + self.intervalChar + str(
                600) + self.intervalChar + str(600) + self.endChar
            self.sendData(cmd.CMD_MOTOR + ForWard)
    def fastforward(self):
        if self.Wheel_Flag:
            if self.Rotate_Flag:
                M_ForWard = self.intervalChar + str(0) + self.intervalChar + str(1000) + self.intervalChar + str(
                    0) + self.intervalChar + str(0) + self.endChar
                self.sendData(cmd.CMD_M_MOTOR + M_ForWard)
            else:
                R_ForWard = self.intervalChar + str(0) + self.intervalChar + str(0) + self.intervalChar + str(
                    0) + self.intervalChar + str(1000) + self.endChar
                self.sendData(cmd.CMD_CAR_ROTATE + R_ForWard)
        else:
            ForWard = self.intervalChar + str(1000) + self.intervalChar + str(1000) + self.intervalChar + str(
                1000) + self.intervalChar + str(1000) + self.endChar
            self.sendData(cmd.CMD_MOTOR + ForWard)
    def backward(self):
        if self.Wheel_Flag:
            if self.Rotate_Flag:
                M_BackWard = self.intervalChar + str(180) + self.intervalChar + str(600) + self.intervalChar + str(
                    0) + self.intervalChar + str(0) + self.endChar
                self.sendData(cmd.CMD_M_MOTOR + M_BackWard)
            else:
                R_BackWard = self.intervalChar + str(0) + self.intervalChar + str(0) + self.intervalChar + str(
                    180) + self.intervalChar + str(600) + self.endChar
                self.sendData(cmd.CMD_CAR_ROTATE + R_BackWard)
        else:
            BackWard = self.intervalChar + str(-600) + self.intervalChar + str(-600) + self.intervalChar + str(
                -600) + self.intervalChar + str(-600) + self.endChar
            self.sendData(cmd.CMD_MOTOR + BackWard)
    def turnRight(self):
        if self.Wheel_Flag:
            M_Turn_Right = self.intervalChar + str(0) + self.intervalChar + str(0) + self.intervalChar + str(
                -90) + self.intervalChar + str(600) + self.endChar
            self.sendData(cmd.CMD_M_MOTOR + M_Turn_Right)
        else:
            Turn_Right = self.intervalChar + str(1500) + self.intervalChar + str(600) + self.intervalChar + str(
                -600) + self.intervalChar + str(-600) + self.endChar
            self.sendData(cmd.CMD_MOTOR + Turn_Right)

    def stop(self):
        if self.Wheel_Flag:
            if self.Rotate_Flag:
                M_Stop = self.intervalChar + str(0) + self.intervalChar + str(0) + self.intervalChar + str(
                    0) + self.intervalChar + str(0) + self.endChar
                self.sendData(cmd.CMD_M_MOTOR + M_Stop)
            else:
                R_Stop = self.intervalChar + str(0) + self.intervalChar + str(0) + self.intervalChar + str(
                    0) + self.intervalChar + str(0) + self.endChar
                self.sendData(cmd.CMD_CAR_ROTATE + R_Stop)
        else:
            Stop = self.intervalChar + str(0) + self.intervalChar + str(0) + self.intervalChar + str(
                0) + self.intervalChar + str(0) + self.endChar
            self.sendData(cmd.CMD_MOTOR + Stop)

    def _build_command(self, front_left, front_right, rear_left, rear_right):
        return (
            self.intervalChar + str(front_left) +
            self.intervalChar + str(front_right) +
            self.intervalChar + str(rear_left) +
            self.intervalChar + str(rear_right) +
            self.endChar
        )
    def adjust(self, middle):
        self.get_new_middle()
        STOP_CMD = self._build_command(0, 0, 0, 0)
        TURN_RIGHT_CMD = self._build_command(600, 600, -600, -600)
        TURN_LEFT_CMD = self._build_command(-600, -600, 600, 600)
        CENTER_MIN = 180
        CENTER_MAX = 230

        if middle is None:
            print("No ball detected to adjust.")
            return

        print(f"Adjusting... Center: {middle}")
        tries = 0
        max_try = 20

        while tries < max_try:
            if middle is None:
                print("Ball lost during adjustment.")
                return 

            if CENTER_MIN < middle < CENTER_MAX:
                print("Already centered.")
                self.drive()
                break
            else:
                if middle < CENTER_MIN:
                    print("Turning Left")
                    self.sendData(cmd.CMD_MOTOR + TURN_LEFT_CMD)
                    time.sleep(0.5)
                    self.sendData(cmd.CMD_MOTOR + STOP_CMD)
                    time.sleep(0.5)  # Small delay to ensure stability before checking again
                    middle = self.get_new_middle()  # Implement this method to get the new middle position
                    # self.drive()
                    # break
                elif middle > CENTER_MAX:
                    print("Turning Right")
                    self.sendData(cmd.CMD_MOTOR + TURN_RIGHT_CMD)
                    time.sleep(0.5)
                    self.sendData(cmd.CMD_MOTOR + STOP_CMD)
                    time.sleep(0.5)  # Small delay to ensure stability before checking again
                    middle = self.get_new_middle()  # Implement this method to get the new middle position
                    # self.drive()
                    # break
                tries += 1
            
    def drive(self):
        print("Adjustment complete, now moving forward...")
        self.fastforward
        time.sleep(0.1)
        self.forward
        time.sleep(5)
        self.stop()
        print("Movement complete.")

    def get_new_middle(self):
        cap = cv2.VideoCapture(0)  # Adjust the index based on your camera setup
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            return None

        # Detect the ball in the frame
        model_name = 'Yolov5_models'
        yolov5_model = 'balls5n.pt'
        model_labels = 'balls5n.txt'

        CWD_PATH = os.getcwd()
        PATH_TO_LABELS = os.path.join(CWD_PATH, model_name, model_labels)
        PATH_TO_YOLOV5_GRAPH = os.path.join(CWD_PATH, model_name, yolov5_model)

        with open(PATH_TO_LABELS, 'r') as f:
            labels = [line.strip() for line in f.readlines()]

        model = yolov5.load(PATH_TO_YOLOV5_GRAPH)

        stride, names, pt = model.stride, model.names, model.pt
        print('stride = ', stride, 'names = ', names)

        min_conf_threshold = 0.25
        model.conf = 0.25
        model.iou = 0.45
        model.agnostic = False
        model.multi_label = True
        model.max_det = 1000

        results = model(frame)
        predictions = results.pred[0]

        boxes = predictions[:, :4]
        scores = predictions[:, 4]
        classes = predictions[:, 5]
        results.render()

        imW, imH = int(640), int(640)

        for i in range(len(scores)):
            curr_score = scores.numpy()
            if i >= len(scores):
                print(f"Skipping index {i}, which is out of bounds for scores with length {len(scores)}")
                continue

            if ((curr_score[i] > min_conf_threshold) and (curr_score[i] <= 1.0)):
                print('Class: ', labels[int(classes[i])], ' Conf: ', curr_score[i])

                xmin = int(max(1, (boxes[i][0])))
                ymin = int(max(1, (boxes[i][1])))
                xmax = int(min(imW, (boxes[i][2])))
                ymax = int(min(imH, (boxes[i][3])))

                center = (xmin + xmax) / 2
                print(f"Detected center: {center}")  # Add this line for debugging
                return center

        return None

    def search(self):
        self._build_command(600, 600, -600, -600)
        time.sleep(2)
        

    def find_ball(self, img):
        
        model_name = 'Yolov5_models'
        yolov5_model = 'balls5n.pt'
        model_labels = 'balls5n.txt'



        CWD_PATH = os.getcwd()
        PATH_TO_LABELS = os.path.join(CWD_PATH, model_name, model_labels)
        PATH_TO_YOLOV5_GRAPH = os.path.join(CWD_PATH, model_name, yolov5_model)

        # Import Labels File
        with open(PATH_TO_LABELS, 'r') as f:
            labels = [line.strip() for line in f.readlines()]
        
    

        # Initialize Yolov5
        
        model = yolov5.load(PATH_TO_YOLOV5_GRAPH)

        stride, names, pt = model.stride, model.names, model.pt
        print('stride = ', stride, 'names = ', names)

        min_conf_threshold = 0.25
        model.conf = 0.25  # NMS confidence threshold
        model.iou = 0.45  # NMS IoU threshold
        model.agnostic = False  # NMS class-agnostic
        model.multi_label = True  # NMS multiple labels per box
        model.max_det = 1000  # maximum number of detections per image

        frame = img.copy()
        results = model(frame)
        predictions = results.pred[0]

        boxes = predictions[:, :4]
        scores = predictions[:, 4]
        classes = predictions[:, 5]
        results.render()

        imW, imH = int(640), int(640)

        max_score = 0
        max_index = 0

    

        for i in range(len(scores)):
            curr_score = scores.numpy()
            if i >= len(scores):
                print(f"Skipping index {i}, which is out of bounds for scores with length {len(scores)}")
                continue

            if ((curr_score[i] > min_conf_threshold) and (curr_score[i] <= 1.0)):
                print('Class: ', labels[int(classes[i])], ' Conf: ', curr_score[i])

                xmin = int(max(1, (boxes[i][0])))
                ymin = int(max(1, (boxes[i][1])))
                xmax = int(min(imW, (boxes[i][2])))
                ymax = int(min(imH, (boxes[i][3])))

                object_name = labels[int(classes[i])]
                label = '%s: %d%%' % (object_name, int(curr_score[i] * 100))
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                label_ymin = max(ymin, labelSize[1] + 10)

                croppedImage = frame[ymin:ymax, xmin:xmax]
                ccx = int((xmax - xmin) / 2)
                ccy = int((ymax - ymin) / 2)
                hsv_pixel = cv2.cvtColor(croppedImage, cv2.COLOR_BGR2HSV)
                pixel_center = hsv_pixel[ccy, ccx]
                ball_color = "No color detected"
                ball_color_num = -1
                self.light_led(0, 0, 0)
                hue_value = pixel_center[0]

                center = (xmin + xmax) / 2
                if hue_value in range(160, 180):
                    ball_color_num = 0
                    ball_color = "RED"
                    self.light_led(255, 0, 0)
                elif hue_value in range(70, 89):
                    ball_color_num = 1
                    ball_color = "GREEN"
                    self.light_led(0, 255, 0)
                elif hue_value in range(90, 105):
                    ball_color_num = 2
                    ball_color = "BLUE"
                    self.light_led(0, 0, 255)
                elif hue_value in range(22, 35):
                    ball_color_num = 3
                    ball_color = "YELLOW"
                    self.light_led(255, 180, 0)
                else:
                    self.light_led(0, 0, 0)

                print('Hue: ', ball_color)

                if ball_color_num == self.sequence[self.sequence_index]:
                    self.adjust(center)
                    self.fastforward()
                    time.sleep(0.1)
                    self.forward()
                    time.sleep(1.5)
                    self.stop()
                    self.fastforward()
                    time.sleep(0.1)
                    self.backward()
                    time.sleep(1.5)
                    self.stop()
                    self.sequence_index += 1  # Move to the next color in the sequence
                    
                

                max_score = curr_score[i]
                max_index = i

        if len (scores) == 0:
            self.light_led(0, 0, 0)
        cv2.imwrite('video.jpg', frame)


        
    def streaming(self,ip):
        stream_bytes = b' '
        try:
            self.client_socket.connect((ip, 8000))
            self.connection = self.client_socket.makefile('rb')
        except:
            #print "command port connect failed"
            pass
        while True:
            try:
                stream_bytes= self.connection.read(4) 
                leng=struct.unpack('<L', stream_bytes[:4])
                jpg=self.connection.read(leng[0])
                if self.IsValidImage4Bytes(jpg):
                            image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                            if self.video_Flag:
                                self.find_ball(image)
                                self.video_Flag=False
            except Exception as e:
                print (e)
                break
                  
    def sendData(self,s):
        if self.connect_Flag:
            self.client_socket1.send(s.encode('utf-8'))

    def recvData(self):
        data=""
        try:
            data=self.client_socket1.recv(1024).decode('utf-8')
        except:
            pass
        return data

    def socket1_connect(self,ip):
        try:
            self.client_socket1.connect((ip, 5000))
            self.connect_Flag=True
            print ("Connection Successful !")
        except Exception as e:
            print ("Connect to server Failed!: Server IP is right? Server is opened?")
            self.connect_Flag=False
    
    

if __name__ == '__main__':
    pass

