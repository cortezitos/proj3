# Project 3

This repository contains the full CSCI502/702 Project 3 implementation.

- `docs/requirements_summary.md`: extracted requirements from `Project_3.pdf` and `CSCI502_Project_2-1.pdf`
- `docs/report_outline.md`: report structure and screenshot checklist
- `docs/defense_cheat_sheet.md`: likely presentation questions and short answers
- `rpi_server/`: Raspberry Pi programs for Task 1, Task 3, and Task 4
- `web/`: Apache CGI assets for the Task 1 sensor web page
- `qt_basics/`: standalone Qt 6 Widgets app for Task 2
- `qt_client/`: Qt 6 client for Task 3
- `Makefile`: one-command task launcher from either the Pi or your laptop

The old top-level `task1.cpp` and `task2.cpp` files are kept as Project 2-style reference code. The runnable Project 3 deliverables live in the folders above.

## One-time setup on Raspberry Pi

Install required packages:

```bash
sudo apt-get update
sudo apt-get install -y apache2 g++ make cmake qt6-base-dev qt6-base-dev-tools
```

Install the Apache CGI assets:

```bash
sudo install -m 644 ~/proj3/web/imu-cgi.conf /etc/apache2/conf-available/imu-cgi.conf
sudo a2enmod cgid
sudo a2enconf imu-cgi
sudo mkdir -p /var/www/html/imu-data
sudo chown $USER:www-data /var/www/html/imu-data
sudo chmod 775 /var/www/html/imu-data
sudo install -m 755 ~/proj3/web/imu_quaternion.cgi /var/www/html/imu_quaternion.cgi
sudo systemctl restart apache2
```

## One-time setup on laptop/PC

For the Qt client and the Qt basics app, install Qt 6 and CMake locally. On your WSL machine this is:

```bash
sudo apt-get update
sudo apt-get install -y cmake qt6-base-dev qt6-base-dev-tools pkg-config
```

## Fastest way to run tasks

### If you are sitting on the Raspberry Pi terminal

From `~/proj3`:

```bash
make task1
make task2
make task3
make task4
```

Behavior on the Pi:

- `make task1` starts the Task 1 web publisher locally
- `make task2` builds and runs the Qt basics app locally
- `make task3` starts the Task 3 socket server locally
- `make task4` starts the Task 4 threaded server locally

### If you are on your laptop/PC

From your local project root:

```bash
make task1 PI_HOST=<current-pi-ip>
make task2
make task3 PI_HOST=<current-pi-ip>
make task4 PI_HOST=<current-pi-ip>
```

Behavior on the laptop:

- `make task1` starts the Pi Task 1 publisher through SSH
- `make task2` builds and runs the local Qt basics app
- `make task3` starts the Pi Task 3 server through SSH, then runs the local Qt client
- `make task4` starts the Pi Task 4 server through SSH, then runs the local Qt client

## Important: Pi IP can change

The Raspberry Pi server IP is not fixed. Before running Task 1, Task 3, or Task 4 from your laptop, check the current Pi IP on the Pi:

```bash
hostname -I
```

Then use that address in the make command:

```bash
make task3 PI_HOST=192.168.0.179
```

If you do not want to repeat it every time:

```bash
export PI_HOST=192.168.0.179
make task3
make task4
```

The Qt client also reads `IMU_HOST` and `IMU_PORT` from the environment if you want to launch it manually.

## How to run every task

### Task 1. Sensor web server

From the Pi:

```bash
cd ~/proj3
make task1
```

From the laptop:

```bash
cd /mnt/c/Users/uteso/OneDrive/Desktop/proj3
make task1 PI_HOST=<current-pi-ip>
```

Open in a browser:

```text
http://<current-pi-ip>/imu_quaternion.cgi
```

Validation:

- `/var/www/html/imu-data/imu_quaternion.txt` should keep changing
- the browser page should show live `w x y z`

### Task 2. Basic Qt 6 Widgets app

From whichever machine you want to display the GUI on:

```bash
cd <project-root>
make task2
```

Validation:

- move the cursor inside the canvas
- verify coordinates update
- verify the trail clears with the button

### Task 3. IMU GUI client/server

Option A: start only the server on the Pi:

```bash
cd ~/proj3
make task3
```

Then on the laptop run the client manually:

```bash
cd /mnt/c/Users/uteso/OneDrive/Desktop/proj3
make task3-client CLIENT_HOST=<current-pi-ip>
```

Option B: start both from the laptop:

```bash
cd /mnt/c/Users/uteso/OneDrive/Desktop/proj3
make task3 PI_HOST=<current-pi-ip>
```

Validation:

- Pi should have `imu_socket_server` running on port `5555`
- Qt client should connect and show a rotating orientation cube

### Task 4. Threaded IMU server

Option A: start only the threaded server on the Pi:

```bash
cd ~/proj3
make task4
```

Then on the laptop run the client:

```bash
cd /mnt/c/Users/uteso/OneDrive/Desktop/proj3
make task3-client CLIENT_HOST=<current-pi-ip>
```

Option B: start both from the laptop:

```bash
cd /mnt/c/Users/uteso/OneDrive/Desktop/proj3
make task4 PI_HOST=<current-pi-ip>
```

Validation:

- Pi should have `imu_multithread_server` running
- the Qt client should still receive live quaternion data
- the web quaternion file should still update

## Status and stop commands

On the Pi:

```bash
make local-status
make local-stop
```

From the laptop:

```bash
make remote-status PI_HOST=<current-pi-ip>
make remote-stop PI_HOST=<current-pi-ip>
```

## Important runtime note

Run only one of these Pi-side publishers at a time:

- `imu_web_publisher`
- `imu_socket_server`
- `imu_multithread_server`

They all publish to the same quaternion file for the CGI page.

## Supporting docs

Use these when preparing the report and presentation:

- `docs/requirements_summary.md`
- `docs/report_outline.md`
- `docs/defense_cheat_sheet.md`
