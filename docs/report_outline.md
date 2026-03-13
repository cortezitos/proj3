# Report Outline

## Cover

- Course, project title, semester.
- Your name and team members.
- Short paragraph describing the Raspberry Pi + BerryIMU + Qt client system.

## 1. Requirements Summary

- Brief summary of Task 1 to Task 4.
- Mention that Project 2 concepts were reused for HAL, scaling, Madgwick fusion, and quaternion orientation.

## 2. Task 1. Sensor Web Server

- Screenshot of Apache CGI page opened from the laptop browser at `http://192.168.0.179/imu_quaternion.cgi`.
- Screenshot with the IMU in one orientation.
- Screenshot with the IMU rotated to a different orientation.
- Short explanation:
  - Apache chosen instead of Nginx because it was already active on the Pi.
  - CGI page reads `/var/www/html/imu-data/imu_quaternion.txt`.
  - `imu_web_publisher` reads IMU data, converts to physical units, runs Madgwick, and writes normalized quaternion values.

## 3. Task 2. Basic Qt 6 Widgets App

- Screenshot of the mouse tracker application.
- Mention Qt concepts used:
  - Widgets layout
  - custom painting
  - mouse event handling
  - signals and slots

## 4. Task 3. IMU Client/Server GUI

- Screenshot of Raspberry Pi server terminal running `imu_socket_server`.
- Screenshot of Qt client connected to the Pi.
- Screenshot of the orientation widget while rotating the IMU.
- Explain the data flow:
  - raw IMU -> physical units -> Madgwick -> quaternion -> TCP socket -> Qt client -> visualization

## 5. Task 4. Threaded Server

- Diagram or screenshot of the three-thread design.
- Explain:
  - Thread 1 acquires IMU data and converts units.
  - Thread 2 runs fusion and normalizes the quaternion.
  - Thread 3 sends quaternion data to the socket and updates the web output file.
- Mention mutexes and condition variables for shared state handoff.

## 6. Build and Run Commands

- Include the exact commands used on the Pi and on the Qt machine.

## 7. Validation

- Describe how each task was tested.
- Mention browser test, socket connection test, and Qt build verification.

## 8. Individual Contribution

- State clearly what you implemented and tested yourself.
