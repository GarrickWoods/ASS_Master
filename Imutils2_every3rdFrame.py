import cv2
import numpy as np
import serial
import time
import imutils

# Initialize list of class labels MobileNet SSD was trained to detect
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow",
           "diningtable", "dog", "horse", "motorbike", "person",
           "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

# load model
net = cv2.dnn.readNetFromCaffe('deploy.prototxt', 'mobilenet_iter_73000.caffemodel')

# Connect to Arduino via serial port
# You might need to change the port name
arduino = serial.Serial('COM5', 9600)

# Start video capture from webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Timer variables
last_detection_time = time.time()
no_person_timeout = 2.0  # Timeout in seconds

# Counter for frames processed
frame_counter = 0

while True:
    ret, frame = cap.read()

    # Create blob from the frame
    blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)

    # Set the blob as input to the network
    net.setInput(blob)

    # Perform object detection
    detections = net.forward()

    # Increment frame counter
    frame_counter += 1

    if frame_counter % 2 == 0:  # Track a person every 3rd frame
        # Find person detections
        person_detections = []
        for i in range(detections.shape[2]):
            class_id = int(detections[0, 0, i, 1])
            if CLASSES[class_id] == "person":
                person_detections.append(detections[0, 0, i, :])

        if len(person_detections) > 0:
            # Reset timer on person detection
            last_detection_time = time.time()

            # Sort person detections by confidence score (descending order)
            person_detections = sorted(person_detections, key=lambda x: x[2], reverse=True)

            # Get the detection with highest confidence score
            person_detection = person_detections[0]
            confidence = person_detection[2]
            if confidence > 0.5:
                # Get the bounding box coordinates
                box = person_detection[3:7] * np.array([frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])
                (startX, startY, endX, endY) = box.astype("int")

                # Draw rectangle around the person on the frame
                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)

                # Calculate the center of the box and send coordinates to Arduino
                center_x = (startX + endX) // 2
                center_y = (startY + endY) // 2

                # Scale the coordinates to fit the pitch and yaw range of your laser mount
                # For simplicity, we're assuming here the range of pitch and yaw is 0 to 180
                pitch = ((1 - center_y / frame.shape[0]) * 20) + 75  # Range: 60 to 90, inverted pitch axis
                yaw = ((1 - center_x / frame.shape[1]) * 53) + 47 # Range: 0 to 100, inverted travel direction

                print(yaw)

                # Send to Arduino
                arduino.write(bytes(f'{pitch},{yaw}\n', 'utf-8'))

    # Check timeout condition and reset laser pointer coordinates
    current_time = time.time()
    elapsed_time = current_time - last_detection_time

    if elapsed_time >= no_person_timeout:
        # Reset laser pointer coordinates to center
        pitch = 85
        yaw = 80
        # Send to Arduino
        arduino.write(bytes(f'{pitch},{yaw}\n', 'utf-8'))

    # Display the frame with the detected person
    cv2.imshow('Person Tracking', frame)

    # Quit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
