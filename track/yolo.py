import cv2 
import numpy as np
import argparse
#from imageai.Detection import ObjectDetection 
import torch 
import base64
import io
from PIL import Image
import math

def capture_video_2fps(model):
    detection=[]
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS, 2)
    while True:
        ret, frame = cap.read()
        results = model(frame)
        # draw bounding boxes
        # display the result with the bounding boxes
        
        df=(results.pandas().xyxy[0])
        # polot the bounding boxes
        #print(df)
        gray = cv2.cvtColor(results, cv2.COLOR_BGR2GRAY)

        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        angles = []

        lines = cv2.HoughLinesP(edges, 1, math.pi / 180.0, 90)
        for [[x1, y1, x2, y2]] in lines:
            #cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 3)
            angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
            if(angle != 0):
                angles.append(angle)

        median_angle = np.median(angles)
        print(median_angle)
        for i in range(len(df)):
            frame = cv2.rectangle(frame, (int(df['xmin'][i]), int(df['ymin'][i])), (int(df['xmax'][i]), int(df['ymax'][i])),color = (0, 255, 0), thickness = 2)
            frame = cv2.putText(frame, str(df['name'][i]), (int(df['xmin'][i]), int(df['ymin'][i])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
        cv2.imshow('frame', frame)
        # use yolo to detect objects


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def read_video_2fps(model):
    cap = cv2.VideoCapture('C:/Users/rayendhahri/Desktop/mobile_robotics/track/test_vid/IMG-5696.MOV')
    cap.set(cv2.CAP_PROP_FPS, 30)
    while True:
        ret, frame = cap.read()
        # catch end of video
        if ret:
            results = model(frame)
            # draw bounding boxes
            # display the result with the bounding boxes
            df=(results.pandas().xyxy[0])
            # polot the bounding boxes
            #print(df[["xmin","ymin","xmax","ymax"]])
            
            for i in range(len(df)):
                frame = cv2.rectangle(frame, (int(df['xmin'][i]), int(df['ymin'][i])), (int(df['xmax'][i]), int(df['ymax'][i])),color = (0, 255, 0), thickness = 2)
                frame = cv2.putText(frame,f"thymio center at x= {(int(df['xmin'][i])+ int(df['xmax'][i]))/2} y= {(int(df['ymax'][i])+ int(df['ymin'][i]))/2}",
                (int(df['xmin'][i]), int(df['ymin'][i])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
            # make imshow window smaller 
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            edges = cv2.Canny(gray, 50, 150, apertureSize=3)
            angles = []

            lines = cv2.HoughLinesP(edges, 1, math.pi / 180.0, 90)
            for [[x1, y1, x2, y2]] in lines:
                #cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 3)
                angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
                if(angle != 0):
                    angles.append(angle)

            median_angle = np.median(angles)
            print(median_angle)
            frame = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)                   
            cv2.imshow('frame', frame)
            # use yolo to detect objects
            # save the video
          
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else: 
            break

    cap.release()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default='yolov3')
    parser.add_argument('--weights', type=str, default='pretrained_weights/yolo.h5')
    parser.add_argument('--source_yol', type=str, default='imageaif', help='source from where to take yolo') 
    
    args = parser.parse_args()
    if args.source_yol == 'imageai':
        #detector = ObjectDetection()
        #detector.setModelTypeAsYOLOv3()
        #detector.setModelPath(args.weights)
        #detector.loadModel()
        #capture_video_2fps(detector)
        x=5
    else:
        # import load state dict pytorch
        model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        #model.max_det = 2
        #model.classes = [0,1]
        #capture_video_2fps(model)
        read_video_2fps(model)
               
    

