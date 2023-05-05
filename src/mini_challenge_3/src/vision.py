# import the opencv library
import cv2 as cv
import numpy as np
import rospy
from std_msgs.msg import String

class ColorDetector:
    
    def __init__(self):
        #self.color_pub = rospy.Publisher("/traffic_light", String, queue_size=1)
        self.min_pixel_count = 2500
        self.red_thresh_up = [340, 100, 100]
        self.red_thresh_down = [10, 75, 60]
        self.yellow_thresh_up = [31, 100, 100]
        self.yellow_thresh_down = [64, 65, 60]
        self.green_thresh_up = [160, 100, 100]
        self.green_thresh_down = [80, 80, 69]

    def detect_color(self, frame):
        names = ["green","yellow","red"]
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask_green = cv.inRange(frame_hsv, self.green_thresh_down, self.green_thresh_up)
        mask_yellow = cv.inRange(frame_hsv, self.yellow_thresh_down, self.yellow_thresh_up)
        mask_red = cv.inRange(frame_hsv, self.red_thresh_down, self.red_thresh_up)
        pixels = [np.sum(mask_green == 255),np.sum(mask_yellow == 255),np.sum(mask_red == 255)]    
        max_count = max(pixels)
        if max_count >= self.min_pixel_count:
            return names[pixels.index(max_count)]
        else:
            return "None"

if __name__ == "__main__":
    detector = ColorDetector
    #rospy.init_node("traffic_monitor")
    camera = cv.VideoCapture(1)  
    while True: 
        ret, frame = camera.read()
        cv.imshow('frame', frame)
        color = detector.detect_color(frame)
        print(color)
        #detector.color_pub.publish(color)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    camera.release()
    cv.destroyAllWindows()
    