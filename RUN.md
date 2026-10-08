# Run instructions

This project is designed to run on a Raspberry Pi with the required hardware connected.

## 1. Connect to the Raspberry Pi

Connect to the Raspberry Pi from another computer using SSH:

```bash
ssh pi@<raspberry-pi-ip>
```

Replace `<raspberry-pi-ip>` with the IP address of your Raspberry Pi.

After connecting, navigate to the project directory:

```bash
cd ball-on-plate-control-system
```

## 2. Install dependencies

Install the required Python libraries using:

```bash
pip install -r requirements.txt
```

## 3. Hardware connection

Before starting the system, make sure that the following components are properly connected:

- Raspberry Pi 4
- Picamera2 camera
- PCA9685 servo controller
- 2 servomotors
- MPU6050 IMU sensor
- Ball-on-Plate mechanical platform

## 4. Raspberry Pi configuration

Make sure that the required interfaces are enabled on the Raspberry Pi:

- Camera interface
- I2C interface

These settings can be checked using:

```bash
sudo raspi-config
```

## 5. Run the system

Start the main application with:

```bash
python src/main.py
```

## 6. Stop the application

To stop the program, use:

```text
Ctrl + C
```

The system will terminate the control loop and stop the application.

## Notes

The application requires access to the Raspberry Pi camera and I2C interface.

The Raspberry Pi, camera, PCA9685 controller, MPU6050 sensor and servomotors should be connected and configured before running the system.
