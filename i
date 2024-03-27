import cv2
import numpy as np
import RPi.GPIO as GPIO

# GPIO setup for pump control
PUMP_PIN_CAMERA1 = 18  # Adjust GPIO pins based on your setup
PUMP_PIN_CAMERA2 = 23
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUMP_PIN_CAMERA1, GPIO.OUT)
GPIO.setup(PUMP_PIN_CAMERA2, GPIO.OUT)

# Define green color range for each camera
lower_range_camera1 = np.array([35, 98, 44])
upper_range_camera1 = np.array([170, 255, 255])

lower_range_camera2 = np.array([35, 98, 44])  # Adjust based on your setup
upper_range_camera2 = np.array([170, 255, 255])  # Adjust based on your setup

# Open cameras
cap1 = cv2.VideoCapture(0)  # Adjust camera indices based on your setup
cap2 = cv2.VideoCapture(1)

while True:
    # Capture frames from both cameras
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    # Resize frames for consistency
    frame1 = cv2.resize(frame1, (640, 480))
    frame2 = cv2.resize(frame2, (640, 480))

    # Convert frames to HSV color space
    hsv1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
    hsv2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

    # Threshold the HSV images to get only green colors
    mask1 = cv2.inRange(hsv1, lower_range_camera1, upper_range_camera1)
    mask2 = cv2.inRange(hsv2, lower_range_camera2, upper_range_camera2)

    # Check if significant portion of the frames is green and control pumps accordingly
    green_pixel_count1 = cv2.countNonZero(mask1)
    green_pixel_count2 = cv2.countNonZero(mask2)

    # Adjust threshold based on your application
    if green_pixel_count1 > 1000:  # Adjust threshold as needed
        GPIO.output(PUMP_PIN_CAMERA1, GPIO.HIGH)  # Start pump for camera 1
    else:
        GPIO.output(PUMP_PIN_CAMERA1, GPIO.LOW)  # Stop pump for camera 1

    if green_pixel_count2 > 1000:  # Adjust threshold as needed
        GPIO.output(PUMP_PIN_CAMERA2, GPIO.HIGH)  # Start pump for camera 2
    else:
        GPIO.output(PUMP_PIN_CAMERA2, GPIO.LOW)  # Stop pump for camera 2

    # Display frames with green detection
    cv2.imshow("Camera 1", frame1)
    cv2.imshow("Camera 2", frame2)

    # Break the loop if 'Esc' key is pressed
    if cv2.waitKey(1) & 0xFF == 27:
        break

# Release cameras and cleanup GPIO
cap1.release()
cap2.release()
cv2.destroyAllWindows()
GPIO.cleanup()
