# Defense Cheat Sheet

## Likely questions and short answers

### Why did you use Apache and CGI for Task 1?

The assignment explicitly asks for `Nginx or Apache` and `CGI and/or PHP`. Apache was already active on the Raspberry Pi, so CGI was the smallest and most direct path to a compliant web page.

### What exactly does the Task 1 backend do?

`imu_web_publisher` reads the BerryIMU through I2C, converts raw measurements to physical units, runs the Madgwick filter, normalizes the quaternion, and writes the latest result to a file that the CGI page displays.

### Which sensors are used in the BerryIMU?

`LSM6DS33` for accelerometer and gyroscope, and `LIS3MDL` for magnetometer.

### How do you get 16-bit measurements from the IMU?

Each axis value comes from two 8-bit registers, low byte and high byte, combined into one signed 16-bit integer.

### Why do you convert raw values to physical units before Madgwick?

The fusion algorithm expects physical quantities, not raw ADC counts. Raw IMU values must be scaled using the configured full-scale sensitivities.

### Why normalize the quaternion?

A valid orientation quaternion should have magnitude `1`. Normalization prevents drift from numerical integration and matches the assignment requirement before transmission.

### What is sent over the socket?

A text line containing `w x y z` followed by a newline. The Qt client parses each line, normalizes again defensively, and updates the orientation widget.

### Why is the Task 3 visualization implemented with a custom Qt widget?

The assignment allows Qt/OpenGL or another solid Qt-compatible visualization approach. A custom painted wireframe cube is lightweight, portable, and easy to explain during defense.

### How is Task 4 threaded?

- Thread 1 acquires IMU data and converts to physical units.
- Thread 2 waits for the latest sample and runs Madgwick fusion.
- Thread 3 waits for the latest quaternion and sends it to the socket.

### How do threads communicate safely?

Through shared state protected by `std::mutex`, with `std::condition_variable` used to notify the next stage when new data is ready.

### What can fail during the demo?

- IMU not connected or wrong I2C bus.
- Apache not running.
- Another process already using port `5555`.
- Qt client pointed at the wrong IP address.

### Fast recovery steps

- Restart Task 1 publisher:
  `./imu_web_publisher --output /var/www/html/imu-data/imu_quaternion.txt`
- Restart Task 3 server:
  `./imu_socket_server --port 5555 --web-output /var/www/html/imu-data/imu_quaternion.txt`
- Restart Task 4 server:
  `./imu_multithread_server --port 5555 --web-output /var/www/html/imu-data/imu_quaternion.txt`
- Re-open:
  `http://192.168.0.179/imu_quaternion.cgi`
