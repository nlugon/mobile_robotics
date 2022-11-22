from TrajectoryPlannerV2 import Terrain, Node 
import matplotlib.pyplot as plt
import numpy as np
import cv2  



# cap video
def test_video():
    cap = cv2.VideoCapture(0)
    while True:
        frame_count = 0
        ret, frame = cap.read()
        cv2.imshow('frame', frame)
        if ret:
            frame_count += 1
            # use yolo to detect objects
            # save the video
            if frame_count == 1:
                # get obstacles
                obstacle_vertices = np.array([[0,0],[0,0]]) # will be replaced 
                initial_thymio_pos = np.array([[0,0],[0,0]]) # will be replaced
                goal_pos = np.array([[0,0],[0,0]]) # will be replaced
            obj_terrain = Terrain()
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

def test_image():
    # read image
    img = cv2.imread('image.jpg') # insert image path
    # use yolo to detect objects
    # save the video
    # get obstacles
    obstacle_vertices = np.array([[0,0],[0,0]]) # will be replaced 
    initial_thymio_pos = np.array([[0,0],[0,0]]) # will be replaced
    goal_pos = np.array([[0,0],[0,0]]) # will be replaced
    obj_terrain = Terrain()


if __name__ == '__main__':
    #test_video()
    test_image() 
