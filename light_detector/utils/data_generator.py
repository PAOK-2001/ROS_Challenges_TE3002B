import cv2 as cv
import os

delay = 150

def save_frame(frame, path, count):
    cv.imwrite(path+'frame%d.jpg'% count, frame)

if __name__ == "__main__":
    count = 0
    camera = cv.VideoCapture(1)  
    while True: 
        ret, frame = camera.read()
        save_frame(frame,"../data/raw/", count)
        cv.imshow('frame', frame)
        count += 1
        if cv.waitKey(delay) & 0xFF == ord('q'):
            break

    camera.release()
    cv.destroyAllWindows()