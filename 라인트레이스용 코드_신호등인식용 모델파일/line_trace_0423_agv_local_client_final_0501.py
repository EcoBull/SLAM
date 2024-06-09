# 노트북에서 실행되는 코드
import cv2
import numpy as np
import threading
import logging
from ultralytics import YOLO
import socket
import sys

#################### 소켓 통신 config ##########################
HOST = '172.30.1.88'
PORT = 12345
c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('Client created')
c.connect((HOST,PORT))
print("Connected")
###############################################################

model = YOLO('best_final.pt') # 코랩에서 학습 후 모델 (코랩해서 학습해서 신호등 초록, 빨강색 구분 모델용)
cap = cv2.VideoCapture(0)
result = ""



def camera_thread():
    try:
        global result
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Camera error")
                break
            
            class_id = 3
            result_yolo = model(frame)
            annotated_frame = result_yolo[0].plot()
            
            bndboxs = result_yolo[0].boxes.data
            # name = results[0].names
            
            for i, bndbox in enumerate(bndboxs):
                class_id = int(bndbox[5])
                # class_name = name[class_id]
            
            if class_id == 1: # 데이터 학습 yaml 파일 class_id : red_light
                result = "STOP"
                
            elif class_id == 0: # 데이터 학습 yaml 파일 class_id : green_light
                result = "GO"
    
            if result == "GO":
                print("==============AGV IS GOING ================")
                print(result)
                c.send(result.encode())
            elif result == "STOP":
                print("==============AGV IS STOP ================")
                print(result)
                c.send(result.encode())
                
            cv2.imshow("Yolo_model & line_tracer",annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
    except Exception as e:
        logging.error(f"Error occurred: {str(e)}")
        
    finally:
        cap.release()
        cv2.destroyAllWindows()

camera_thread = threading.Thread(target=camera_thread)
camera_thread.start()
# camera_thread.join()