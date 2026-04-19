import cv2
import time
import numpy as np
import serial
import threading
from ultralytics import YOLO


data = np.load('fisheye_calibration.npz')
K, D = data['K'], data['D']


model = YOLO('best_YOLOv11m.pt') 

class VideoStream:
    def __init__(self, src=1):
        self.cap = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        self.ret, self.frame = self.cap.read()
        self.stopped = False

    def start(self):
        threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if ret: self.frame = frame

    def stop(self):
        self.stopped = True
        self.cap.release()

class YOLOThread:
    def __init__(self, model):
        self.model = model
        self.frame = None
        self.results = None
        self.stopped = False
        self.lock = threading.Lock()

    def start(self):
        threading.Thread(target=self.infer, args=(), daemon=True).start()
        return self

    def infer(self):
        
        while not self.stopped:
            if self.frame is not None:
                self.results = self.model(self.frame, conf=0.5, verbose=False, device='cpu')
                self.frame = None 
            else:
                time.sleep(0.005)
        with self.lock:
            self.results = self.model(...)

def nothing(x): pass
cv2.namedWindow('Robot Vision')
cv2.createTrackbar('OFF_X', 'Robot Vision', 211, 1000, nothing) 
cv2.createTrackbar('OFF_Y', 'Robot Vision', 482, 1000, nothing)

# Arduino setup
arduino = None
try:
    arduino = serial.Serial('COM12', 9600, timeout=0.001)
    time.sleep(2)
except:
    print("Erreur Serial : Vérifie le port COM12")

# Initialisation Vision
vs = VideoStream(src=1).start()
h, w = vs.frame.shape[:2]
new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), np.eye(3), balance=0.0)
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, (w, h), cv2.CV_16SC2)

yolo_thread = YOLOThread(model).start()

SCALE = 1.0846 
robot_ready = True

while True:
    frame = vs.frame
    if frame is None: break

    undistorted = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
    
    if yolo_thread.frame is None:
        yolo_thread.frame = undistorted
    
    # C. Offsets and  Robot Base
    OFFSET_X = cv2.getTrackbarPos('OFF_X', 'Robot Vision') - 500
    OFFSET_Y = cv2.getTrackbarPos('OFF_Y', 'Robot Vision') - 500


    rb_px = int((w/2) - (OFFSET_X / SCALE))
    rb_py = int((h/2) - (OFFSET_Y / SCALE))


    #  VISUALISATION
    cv2.line(undistorted, (w//2 - 15, h//2), (w//2 + 15, h//2), (255, 255, 255), 1)
    cv2.line(undistorted, (w//2, h//2 - 15), (w//2, h//2 + 15), (255, 255, 255), 1)
    
    status_col = (0, 255, 0) if robot_ready else (0, 0, 255)
    #cv2.putText(undistorted, f"ROBOT: {'READY' if robot_ready else 'BUSY'}", (20, 40), 
    #            cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_col, 2)
    
    # Point robot (red)
    cv2.circle(undistorted, (rb_px, rb_py), 7, (0, 0, 255), -1)
    cv2.putText(undistorted, "BASE", (rb_px + 10, rb_py), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    axis_len = 200  # pixels

    # X axis (to the right)
    cv2.arrowedLine(undistorted,
                    (rb_px, rb_py),
                    (rb_px - axis_len, rb_py),
                    (0, 0, 255), 2)

    cv2.putText(undistorted, "X+",
                (rb_px - axis_len - 5, rb_py),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0,0,255), 2)

    # Y axis (UPWARD in image = negative real Y usually)
    cv2.arrowedLine(undistorted,
                    (rb_px, rb_py),
                    (rb_px, rb_py - axis_len),
                    (255, 0, 0), 2)

    cv2.putText(undistorted, "Y+",
                (rb_px, rb_py - axis_len - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255,0,0), 2)
    
    with yolo_thread.lock:
        results = yolo_thread.results
        yolo_thread.results = None
    if results:
        for r in results:
            for box in r.boxes:
                x_px, y_px, bw, bh = box.xywh[0].cpu().numpy()

                #center of detection
                cv2.circle(undistorted, (int(x_px), int(y_px)), 4, (0,255,255), -1)
                
                # TRANSFORMATION
                rel_x = x_px - (w / 2)
                rel_y = y_px - (h / 2)
                rx = (rel_x * SCALE) + OFFSET_X
                ry = -(rel_y * SCALE) + OFFSET_Y

                cv2.rectangle(undistorted, (int(x_px-bw/2), int(y_px-bh/2)), (int(x_px+bw/2), int(y_px+bh/2)), (0, 255, 0), 2)
                #cv2.putText(undistorted, f"X:{rx:.1f} Y:{ry:.1f}", (int(x_px), int(y_px)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = model.names[cls_id]

                cv2.putText(undistorted,
                            f"{label} ({conf:.2f}) X:{rx:.1f} Y:{ry:.1f}",
                            (int(x_px), int(y_px)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0,255,0),
                            2)                
            

                # Arduino 
                if robot_ready and arduino:
                    type_char = 'L' if int(box.cls[0]) == 37 else 'R'
                    msg = f"{type_char},{rx:.1f},{ry:.1f}\n"
                    arduino.write(msg.encode())
                    robot_ready = False
                    yolo_thread.results = None
                    break 

    # check READY
    if arduino and arduino.in_waiting > 0:
        line = arduino.readline().decode('utf-8', errors='ignore')
        if "READY" in line:
            robot_ready = True

    cv2.imshow('Robot Vision', undistorted)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

vs.stop()
yolo_thread.stopped = True
cv2.destroyAllWindows()