import cv2 as cv
import os

delay = 150
disp_width = 640
disp_height = 480
flip = 0
cam_port =  'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(disp_width)+', height='+str(disp_height)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

def save_frame(frame, path, count):
    cv.imwrite(path+'well_lit%d.jpg'% count, frame)


if __name__ == "__main__":
    count = 420
    camera = cv.VideoCapture(cam_port)  
    while True: 
        ret, frame = camera.read()
        frame = cv.GaussianBlur(frame,(5,5),0)
        save_frame(frame,"../data/raw/", count) 
        '''cv.imshow('frame', frame)'''
        count += 1
        if cv.waitKey(delay) & 0xFF == ord('q'):
            break

    camera.release()
    cv.destroyAllWindows()
