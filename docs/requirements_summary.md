# Project 3 Requirements Summary

## 1. Required deliverables from Project 3

- Individual PDF report with screenshots of completed tasks and program code.
- `1-3` minute demo video of the working application.
- Code submitted to GitHub and report/video submitted to Moodle.
- Class presentation and live demonstration of the working Raspberry Pi + IMU setup.

## 2. Task-by-task technical requirements

### Task 1. Creating a sensor web server

Project 3 requires:

- Configure `Nginx` or `Apache` on the Raspberry Pi.
- Create a simple IMU sensor web page similar to the textbook example in Chapter 12.
- Use `CGI and/or PHP` scripts.
- Output sensor orientation quaternion readings.
- Show the page from the laptop/client side and include screenshots for different sensor orientations.

Implementation consequence:

- The Raspberry Pi must continuously produce quaternion data.
- The web page must visibly display `w`, `x`, `y`, and `z`.
- The values must refresh over time without manual page edits.

### Task 2. Getting started with Qt 6

Project 3 requires:

- Obtain and use Qt 6.
- Learn basic Qt Widgets GUI development.
- Implement a modest but working GUI application.
- Demonstrate it running on the PC/laptop during presentation.

Implementation consequence:

- This task can be separate from the IMU client.
- The application should clearly demonstrate Qt Widgets layouts, events, and interaction.

### Task 3. IMU GUI development

Project 3 requires:

- Raspberry Pi acts as the server.
- Laptop/PC acts as the client.
- Server side must:
  - read raw IMU measurements
  - convert them to physical values
  - run a sensor fusion algorithm
  - normalize and produce quaternion orientation data
  - send quaternion data through socket communication
- Client side must:
  - connect to the server socket
  - read the quaternion data
  - visualize the IMU 3D orientation in a Qt GUI
- Visualization can use Qt/OpenGL or another solid Qt-compatible approach.

Implementation consequence:

- The server must expose a TCP socket protocol.
- The client must parse the stream, maintain connection state, and render orientation live.

### Task 4. Threaded IMU interfacing

Project 3 requires a multithreaded Raspberry Pi server with exactly this logical pipeline:

- Thread 1: read raw IMU measurements and convert them to physical values.
- Thread 2: run sensor fusion and produce normalized quaternion estimates.
- Thread 3: send quaternion data through socket communication.
- Shared variables protected by mutexes should be used to pass data between threads.

Implementation consequence:

- The server design should show clear thread responsibilities.
- The handoff between stages must be explainable during defense.

## 3. What can be reused conceptually from Project 2

Project 2 provides the conceptual backbone for Project 3:

- A hardware abstraction layer separating low-level IMU access from application logic.
- OOP C++ wrapping of I2C devices.
- LSM6DS33 accelerometer/gyroscope register configuration and 16-bit raw reads.
- LIS3MDL magnetometer register configuration and 16-bit raw reads.
- Conversion of raw readings to physical units using sensor scaling constants.
- Madgwick AHRS fusion from `gyro + accel + mag` input.
- Quaternion normalization and terminal visualization pipeline.
- Quaternion-driven 3D orientation rendering logic.

## 4. Proposed project structure

- `rpi_server/imu_core.h`
  Common BerryIMU HAL, scaling, quaternion, and Madgwick code reused by the Pi-side programs.
- `rpi_server/imu_web_publisher.cpp`
  Task 1 backend that publishes quaternion values for Apache CGI.
- `rpi_server/imu_socket_server.cpp`
  Task 3 single-thread client/server implementation.
- `rpi_server/imu_multithread_server.cpp`
  Task 4 threaded Raspberry Pi server.
- `web/imu_quaternion.cgi`
  Apache CGI page that reads the latest published quaternion and renders the browser page.
- `qt_basics/`
  Separate Qt 6 Widgets application for Task 2.
- `qt_client/`
  Qt 6 TCP client and orientation viewer for Task 3.
