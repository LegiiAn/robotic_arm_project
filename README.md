# Computer Vision Pick-and-Place Robot

This repository contains the final code for a 6-servo robotic arm system that uses computer vision to sort electronic components (LEDs and Resistors). 

The system uses a custom-trained YOLOv11 AI model to detect components via a webcam, calculates their real-world coordinates using a calibrated fisheye lens, and sends inverse kinematic (IK) commands to an Arduino to pick up and sort the items into designated bins.

## Repository Structure

* **`ElectroCom61_YOLO11.ipynb`**: Google Colab Jupyter Notebook used to train the YOLO model on a custom dataset.
* **`2_Camera_Calibration/`**: Contains the Python script (`Once_calibration.py`) and matrix (`fisheye_calibration.npz`) used to un-distort the fisheye webcam lens using a checkerboard pattern.
* **`3_Main_Python_Controller/`**: Contains the main execution script (`brain2.py`) which runs the live YOLO inference and communicates with the Arduino over serial. Includes the trained model weights (`best_YOLOv11m.pt`).
* **`4_finalarduinocode/`**: Contains the C++ script (`robot_controller.ino`) that calculates dynamic Inverse Kinematics and smoothly drives the 6 servos.

## How to Run
1. Upload the Arduino code to your board and ensure the servos are connected to the correct pins (Base=9, Shoulder=8, Elbow=10, WristPitch=6, WristRotate=7, Gripper=11).
2. Ensure your webcam is mounted overhead and connected to your PC.
3. Run `brain2.py` in your Python environment.