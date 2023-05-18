import jetson.inference
import jetson.utils
import cv2
import numpy

net = jetson.inference.detectNet(argv=['--model=best.onnx', '--labels=labels.txt', ' --input-blob=input_0', '--output-cvg=scores', '--output-bbox=boxes'],  threshold=0.5)

cam_port =  'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=60/1 ! nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true'  
camera = cv2.VideoCapture(cam_port)
# if ~camera.isOpened():
#    print('Could not read from camera')  
#    exit()
while True: 
    ret, frame = camera.read()
    img_cuda = jetson.utils.cudaFromNumpy(frame)
    detections = net.Detect(img_cuda)
    img = jetson.utils.cudaToNumpy(img_cuda)
    cv2.imshow("Current Frame", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
