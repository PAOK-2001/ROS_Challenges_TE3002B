import cv2
import time
import numpy as np
import tensorrt as trt
import pycuda.autoinit
import pycuda.driver as cuda

EXPLICIT_BATCH = 1 << (int)(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
TRT_LOGGER = trt.Logger(trt.Logger.INFO)

batch = 1
host_inputs  = []
cuda_inputs  = []
host_outputs = []
cuda_outputs = []
bindings = []


def Inference(engine, image):
    image = (2.0 / 255.0) * image.transpose((2, 0, 1)) - 1.0
    np.copyto(host_inputs[0], image.ravel())
    stream = cuda.Stream()
    context = engine.create_execution_context()
    start_time = time.time()
    cuda.memcpy_htod_async(cuda_inputs[0], host_inputs[0], stream)
    context.execute_v2(bindings)
    cuda.memcpy_dtoh_async(host_outputs[0], cuda_outputs[0], stream)
    stream.synchronize()
    print("execute times "+str(time.time()-start_time))
    output = host_outputs[0]
    print(np.argmax(output))

def PrepareEngine():
    with open('trafficv2.engine', 'rb') as f:
        serialized_engine = f.read()

    runtime = trt.Runtime(TRT_LOGGER)
    engine = runtime.deserialize_cuda_engine(serialized_engine)

    for binding in engine:
        size = trt.volume(engine.get_tensor_shape(binding)) * batch
        host_mem = cuda.pagelocked_empty(shape=[size],dtype=np.float32)
        cuda_mem = cuda.mem_alloc(host_mem.nbytes)

        bindings.append(int(cuda_mem))
        if engine.get_tensor_mode(binding)==trt.TensorIOMode.INPUT:
            host_inputs.append(host_mem)
            cuda_inputs.append(cuda_mem)
        else:
            host_outputs.append(host_mem)
            cuda_outputs.append(cuda_mem)

    return engine


if __name__ == "__main__":
    engine = PrepareEngine()
    cam_port =  'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=60/1 ! nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true'  
    camera = cv2.VideoCapture(cam_port)
   
    while True: 
        ret, frame= camera.read()
        Inference(engine, frame)
    engine = []