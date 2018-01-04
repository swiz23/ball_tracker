# Camera Ball Tracker

Created by Solomon Wiznitzer during the Hackathon at Northwestern University

## Description

**Objective:** To move a red ball in front of a camera mounted on a pan/tilt mechanism and watch how the camera tracks it.

**How It Works:** The camera is mounted on a mechanism with two micro servo motors. One servo motor controls the panning of the camera and the other controls the tilt. Using computer vision, the program moves the camera using the motors so that the ball is in the center of the camera frame.

**Requirements:** Needs ROS, the OpenCV and PySerial libraries, a [Micro Maestro Control Board](https://www.pololu.com/product/1350), two micro-servo motors, a pan/tilt mechanism, and a webcam.

**Get Started:** To run, clone the repo, and type `roslaunch ball_tracker ball_tracker.launch` in the terminal window.

## Technical Overview

### Image Processing

**Node**: `image_converter` from the [cam_listener.py](src/cam_listener.py) script
**Subscriber:** `usb_cam/image_raw` as `sensor_msgs/Image` message defintion
**Publishes to topic:** `ball_center` as `geometry_msgs/Point` message definition

To track the red ball, each incoming frame (converted from a ROS image to an OpenCV image using CVBridge) from the camera is first converted to the HSV color-space. Using OpenCV's `cv2.inRange` function, a mask is generated so that only pixels that have values within a prespecified range are shown. For this project, the range was:
```
lower_red = np.array([0,50,50])
upper_red = np.array([4,255,255])
```
Then, OpenCV's `cv2.findContours` function is used to find the largest contour in the image (which is assumed to be the ball). From this, the centroid of the ball in the camera frame can be calculated. The 'x' and 'y' values are then published.

### Servomotor Control

**Node**: `servo_controller` from the [servo_move.py](src/servo_move.py) script
**Subscriber:** `ball_center` as `geometry_msgs/Point` message defintion

This node subtracts the 'x' and 'y' coordinates that define the center of the camera frame from the 'x' and 'y' values that define the centroid of the red ball. If the resulting values are below a certain threshold, nothing happens so as to prevent the camera from jittering too much. However, if the difference in 'x' is greater than the threshold, the 'pan' servo motor is set to move a fixed step size. Similarly, if the difference in 'y' is greater than the threshold, the 'tilt' servo motor is set to move a fixed step size. 

A demonstration of the project can be seen below.

![demo](media/ballTracker.gif)