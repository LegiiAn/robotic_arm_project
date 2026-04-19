import cv2
import numpy as np
import os
import glob

# --- CONFIGURATION ---
CHECKERBOARD = (10, 7)  # Internal vertices
SQUARE_SIZE = 25        # mm
folder = 'calib_images'
if not os.path.exists(folder): os.makedirs(folder)

# Prepare object points (0,0,0), (25,0,0), etc.
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = [] 
imgpoints = [] 

# --- STEP 1: CAPTURE IMAGES ---
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW) # 1 for external webcam
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
print("Position checkerboard and press 'S' to capture. Need 10 images. 'Q' to quit.")

count = 0
while count < 10:
    ret, frame = cap.read()
    if not ret: break
    cv2.imshow('Calibration Capture', frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        count += 1
        cv2.imwrite(f'{folder}/image_{count}.jpg', frame)
        print(f"Captured {count}/10")
    elif key == ord('q'): break

cap.release()
cv2.destroyAllWindows()

# --- STEP 2: CALCULATE CALIBRATION ---
images = glob.glob(f'{folder}/*.jpg')
img_shape = None

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_shape = gray.shape[::-1]
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners2)

if len(objpoints) > 0:
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    # Using Fisheye model as per your request
    ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        objpoints, imgpoints, img_shape, K, D,
        flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )
    np.savez("fisheye_calibration.npz", K=K, D=D)
    print("Success! Calibration saved to fisheye_calibration.npz")
else:
    print("Error: No corners detected in images.")
    