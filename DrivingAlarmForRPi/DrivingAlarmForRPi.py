# ---------------------------------- make2explore.com----------------------------------------------------------#
# Tutorial          - MediaPipe Machine Learning Solutions on Raspberry Pi - Example 2 : Face Mesh
# Created By        - info@make2explore.com
# Version - 1.0
# Last Modified     - 24/03/2022 15:00:00 @admin
# Software          - Python, Thonny IDE, Standard Python Libraries, Mediapipe Python Package
# Hardware          - Raspberry Pi 4 model B, Logitech c270 webcam
# Source Repo       - https://github.com/make2explore/MediaPipe-On-RaspberryPi
# Credits           - MediaPipe Framework by Google : https://mediapipe.dev/
# -------------------------------------------------------------------------------------------------------------#

import cv2
import mediapipe as mp
import numpy as np
import serial
import smbus
import math
import time

# MPU6050 I2C 物理地址
MPU6050_ADDR=0x68

# MPU6050 采样率的设置寄存器
MPU6050_SMPLRT_DIV=0x19

# MPU6050配置寄存器用来设置是否需要外置同步帧
# 设置对3轴加速度输出以及3轴陀螺仪输出进行数字低通滤波的方式
MPU6050_CONFIG=0x1a

#设置3轴陀螺仪输出量程的寄存器
MPU6050_GYRO_CONFIG=0x1b

# 设置3轴加速度输出量程的寄存器
MPU6050_ACCEL_CONFIG=0x1c

# 设置MPU6050的工作方式的寄存器 睡眠/循环/ 内部时钟。。。
MPU6050_PWR_MGMT_1=0x6b


# MPU6050 3轴加速计的数据地址
MPU6050_ACCX_DATA=0x3b
MPU6050_ACCY_DATA=0x3d
MPU6050_ACCZ_DATA=0x3f

# MPU6050 3轴陀螺仪的数据地址
MPU6050_GYROX_DATA=0x43
MPU6050_GYROY_DATA=0x45
MPU6050_GYROZ_DATA=0x47

class MPU6050(object):
    def __init__(self, address = MPU6050_ADDR, bus = 1,scal_acc=4,scal_gyro=1000):
        self.bus = smbus.SMBus(bus)
        self.address = address
        
        # 设置工作在默认的8kHZ下
        self.bus.write_byte_data(self.address,MPU6050_SMPLRT_DIV,0x00)
        # 不需要外置帧同步、滤波器工作在方式0
        self.bus.write_byte_data(self.address,MPU6050_CONFIG,0x00)
        # 设置陀螺仪的工作量程为  +- 500°/s
        self.bus.write_byte_data(self.address,MPU6050_GYRO_CONFIG,0x08)
        # 设置加速度计的量程为 +- 2g
        self.bus.write_byte_data(self.address,MPU6050_ACCEL_CONFIG,0x00)
        # 设置MPU6050的工作方式为采用内部8k时钟
        self.bus.write_byte_data(self.address,MPU6050_PWR_MGMT_1,0x00)
        
        self.scal_acc = 65536.0/scal_acc/9.8
        self.scal_gyro = 65536.0/scal_gyro
  
    def red_word_2c(self,address):
        high = self.bus.read_byte_data(self.address,address)
        low = self.bus.read_byte_data(self.address,address+1)
        val = (high<<8)+low
        if (val>=0x8000):
            return -((65535-val)+1)
        else: 
            return val
                
    def get_rawAcc(self):
        rawAccX = self.red_word_2c(MPU6050_ACCX_DATA);
        rawAccY = self.red_word_2c(MPU6050_ACCY_DATA);
        rawAccZ = self.red_word_2c(MPU6050_ACCZ_DATA);
        return rawAccX,rawAccY,rawAccZ
        
    def get_ACC(self):
        rawAccX,rawAccY,rawAccZ = self.get_rawAcc()
        accX = rawAccX/self.scal_acc
        accY = rawAccY/self.scal_acc
        accZ = rawAccZ/self.scal_acc
        return accX,accY,accZ
        
    def get_rawGyro(self):
        rawGyroX = self.red_word_2c(MPU6050_GYROX_DATA);
        rawGyroY = self.red_word_2c(MPU6050_GYROY_DATA);
        rawGyroZ = self.red_word_2c(MPU6050_GYROZ_DATA);
        return rawGyroX,rawGyroY,rawGyroZ
    
    def get_Gyro(self):
        rawGyroX,rawGyroY,rawGyroZ =self.get_rawGyro()        
        GyroX = rawGyroX/self.scal_gyro
        GyroY = rawGyroY/self.scal_gyro
        GyroZ = rawGyroZ/self.scal_gyro    
        return GyroX,GyroY,GyroZ
        
    def calc_GyroOffsets(self):
        x =0
        y =0
        z =0
        for i in range(3000):
            rx,ry,rz = self.get_Gyro()
            x = x + rx
            y = y + ry
            z = z + rz
            
        gyroXoffset = x/3000
        gyroYoffset = y/3000
        gyroZoffset = z/3000
        
        return gyroXoffset,gyroYoffset,gyroZoffset
        
    def  calc_angleAcc(self):
        accX,accY,accZ  = self.get_ACC()
        
        angleAccX = math.degrees(math.atan2(accY, math.sqrt(accZ * accZ + accX * accX)))
        angleAccY = math.degrees(math.atan2(accX, math.sqrt(accZ * accZ + accY * accY)))
        
        return angleAccX,angleAccY

def eye_aspect_ratio(pts):
    A = np.sqrt(np.dot(pts[1] - pts[5], pts[1] - pts[5]))
    B = np.sqrt(np.dot(pts[2] - pts[4], pts[2] - pts[4]))
    C = np.sqrt(np.dot(pts[0] - pts[3], pts[0] - pts[3]))
    ear = (A + B) / (2.0 * C)
    return ear

def open_bluetooth_serial():
    local_bluetooth_serial=None
    try:
        local_bluetooth_serial = serial.Serial("/dev/rfcomm0",9600)
        local_bluetooth_serial.write("0".encode())
    except Exception:
        local_bluetooth_serial=None
    return local_bluetooth_serial

m_MPU = MPU6050(address = 0x68)
GyroOffsets=m_MPU.calc_GyroOffsets()

#普通摄像头0.23,广角130度摄像头0.28
ear_threshold = 0.23
#ear_threshold = 0.28
count_threshold = 3
count = 0

bluetooth_serial=None
while bluetooth_serial==None:
    bluetooth_serial = open_bluetooth_serial()

cap = cv2.VideoCapture(0)
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(
                max_num_faces=1,
                refine_landmarks=True,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5)
#mp_drawing = mp.solutions.drawing_utils
#mp_drawing_styles = mp.solutions.drawing_styles
#drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)
#freq = cv2.getTickFrequency()  # 系统频率

while(True):
    acc_x,acc_y,acc_z=m_MPU.get_ACC()
    if abs(acc_y)>18.0:
        bluetooth_serial.write("5".encode())
    elif abs(acc_x)>5.0:
        bluetooth_serial.write("4".encode())
    elif acc_y>8.0:
        bluetooth_serial.write("3".encode())
    elif acc_y>5.0:
        bluetooth_serial.write("2".encode())
    else:
        #while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            # If loading a video, use 'break' instead of 'continue'.
            continue

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        #t1 = cv2.getTickCount()
        results = face_mesh.process(image)

        # Draw the face mesh annotations on the image.
        #image.flags.writeable = True
        #image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                #mp_drawing.draw_landmarks(
                    #image=image,
                    #landmark_list=face_landmarks,
                    #connections=mp_face_mesh.FACEMESH_TESSELATION,
                    #landmark_drawing_spec=None,
                    #connection_drawing_spec=mp_drawing_styles
                        #.get_default_face_mesh_tesselation_style())
                #mp_drawing.draw_landmarks(
                    #image=image,
                    #landmark_list=face_landmarks,
                    #connections=mp_face_mesh.FACEMESH_CONTOURS,
                    #landmark_drawing_spec=None,
                    #connection_drawing_spec=mp_drawing_styles
                        #.get_default_face_mesh_contours_style())
                #mp_drawing.draw_landmarks(
                    #image=image,
                    #landmark_list=face_landmarks,
                    #connections=mp_face_mesh.FACEMESH_IRISES,
                    #landmark_drawing_spec=None,
                    #connection_drawing_spec=mp_drawing_styles
                        #.get_default_face_mesh_iris_connections_style())
                ear = 0
                eye_indexes = [[33, 160, 158, 133, 153, 144], [362, 385, 387, 263, 373, 380]]
                for eye_index in eye_indexes:
                    points = []
                    for index in eye_index:
                        points.append((face_landmarks.landmark[index].x, face_landmarks.landmark[index].y,                                               face_landmarks.landmark[index].z))
                    ear = ear + eye_aspect_ratio(np.array(points))
                ear = ear / 2
                #cv2.putText(image, 'ear: %.2f' % ear, (200, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
                if ear < ear_threshold:
                    count = count + 1
                else:
                    count = 0

        if count > count_threshold:
            bluetooth_serial.write("1".encode())
        else:
            bluetooth_serial.write("0".encode())
        #t2 = cv2.getTickCount()

        #fps = freq / (t2 - t1)
        # 显示速度
        #cv2.putText(image, 'FPS: %.2f' % fps, (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))

        # Flip the image horizontally for a selfie-view display.
        #cv2.imshow('MediaPipe Face Mesh', cv2.flip(image, 1))
        #cv2.imshow('MediaPipe Face Mesh', image)
        #if cv2.waitKey(5) & 0xFF == 27:
            #break
cap.release()
#cv2.destroyAllWindows()

# ---------------------------------------support@make2explore.com-------------------------------#
