# Project 3

This folder contains all coding deliverables for Project 3:

- `rpi_server/`: Raspberry Pi IMU server for Tasks 3 and 4.
- `web/`: Apache CGI assets for Task 1.
- `qt_basics/`: separate Qt Widgets starter app for Task 2.
- `qt_client/`: Qt client that reads the quaternion stream and visualizes orientation.

## Task 1: sensor web server on Raspberry Pi

This deployment uses Apache + CGI. On this Raspberry Pi, Apache runs with a private `/tmp`, so the web page and the IMU server share data through `/var/www/html/imu-data/imu_quaternion.txt` instead of `/tmp`.

### Raspberry Pi install steps

```bash
sudo apt update
sudo apt install -y apache2 g++ make
sudo a2enmod cgid
sudo install -m 644 ~/proj3/proj3/web/imu-cgi.conf /etc/apache2/conf-available/imu-cgi.conf
sudo a2enconf imu-cgi
sudo install -m 755 ~/proj3/proj3/web/imu_quaternion.cgi /var/www/html/imu_quaternion.cgi
sudo mkdir -p /var/www/html/imu-data
sudo chown $USER:www-data /var/www/html/imu-data
sudo chmod 775 /var/www/html/imu-data
sudo systemctl restart apache2
```

### Build and run the IMU server on the Pi

```bash
cd ~/proj3/proj3/rpi_server
make
make start
```

Useful control commands:

```bash
make status
make logs
make stop
make run
```

The server:

- Thread 1 reads raw IMU data and converts it to physical units.
- Thread 2 runs Madgwick fusion and normalizes the quaternion.
- Thread 3 serves `w x y z\n` over TCP to clients.
- Thread 2 also updates the quaternion file that Apache reads.

### Open the web page from the laptop

```text
http://<rpi-ip>/imu_quaternion.cgi
```

Example for your board:

```text
http://192.168.0.179/imu_quaternion.cgi
```

Take screenshots of that page while rotating the IMU.

## Task 2: basic Qt Widgets app

Open `qt_basics/CMakeLists.txt` in Qt Creator on the laptop/PC.

The app is a mouse tracker that shows:

- current cursor position inside the canvas
- recent trail points
- reset button
- live status text

## Task 3: Qt IMU client/server app

Open `qt_client/CMakeLists.txt` in Qt Creator on the laptop/PC.

Run the Raspberry Pi server first, then launch the Qt client and connect to:

- host: Raspberry Pi IP, for example `192.168.0.179`
- port: `5555`

The Qt client includes:

- TCP socket reader for `w x y z` lines
- live quaternion text display
- a custom orientation widget that renders a rotated 3D cube projection

## Task 4: threaded server design

The server implementation in `rpi_server/imu_multithread_server.cpp` uses exactly three worker threads:

1. IMU sampling thread
2. fusion thread
3. socket server thread

Shared state is protected with `std::mutex` and `std::condition_variable`.

## Local dry-run without hardware

You can test the server on any Linux machine in mock mode:

```bash
cd rpi_server
make
./imu_server --mock
```

This generates synthetic IMU values so the Qt client can be tested before using the real Raspberry Pi.


## Why the page can look stuck

The web page refreshes every second, but it only shows the latest quaternion file written by `imu_server`. If the numbers stop changing, usually `imu_server` is not running anymore and the page is showing the last saved values. Use:

```bash
cd ~/proj3/proj3/rpi_server
make status
make start
```
