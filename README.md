# ESP32_cam_motion_detection
The ESP32-CAM Motion Detection Using Frame Differencing is a smart surveillance system that detects movement by comparing consecutive camera frames. Unlike systems that depend only on a PIR sensor, this method uses image-processing techniques to identify changes directly from the camera feed.

In this system, the ESP32-CAM continuously captures a sequence of images or frames. A current frame is compared with a previously captured reference frame. The system calculates the difference between the two frames and determines whether a significant change has occurred. If the difference exceeds a predefined threshold, motion is detected.

The basic process involves converting the captured frames into a suitable format, comparing corresponding pixels, and calculating the amount of change between frames. Small changes caused by image noise can be ignored by using a threshold value, while significant changes caused by a moving person or object are identified as motion.

When motion is detected, the ESP32-CAM can capture and save an image, activate an LED or buzzer, or send an alert through Wi-Fi. The captured images can also be transmitted to a server or stored on a microSD card for further analysis.

Working Principle

Camera Frame 1 → Camera Frame 2 → Frame Difference Calculation → Threshold Comparison → Motion Detection → Image Capture/Alert

For example, if the difference between two consecutive frames is small, the system considers the scene stationary. If the difference is greater than the predefined threshold, the system identifies the change as motion.

Main Components
ESP32-CAM – Camera and processing unit
OV2640 Camera – Captures video frames
MicroSD Card – Stores detected-motion images
Wi-Fi – Enables remote monitoring and notifications
LED/Buzzer – Provides motion alerts
Advantages
Does not require a separate PIR sensor
Detects motion directly from camera images
Low-cost and compact solution
Can capture images when movement occurs
Threshold can be adjusted for different environments
Suitable for wireless security and surveillance
Applications
Home and office security
Door and entrance monitoring
Restricted-area surveillance
Smart security cameras
Remote monitoring systems
Wildlife and outdoor activity monitoring

The proposed system provides a low-cost camera-based motion detection solution by using frame differencing on the ESP32-CAM. With suitable thresholding and noise reduction, it can efficiently identify changes in a monitored scene and initiate an appropriate security response.
