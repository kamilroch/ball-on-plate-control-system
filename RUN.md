# Run instructions

This project is designed to run on a Raspberry Pi with the required hardware connected.

## 1. Install dependencies

Install the required Python libraries using:

```bash
pip install -r requirements.txt
```

## 2. Hardware connection

Before starting the system, make sure that the following components are properly connected:

- Raspberry Pi 4
- Picamera2 camera
- PCA9685 servo controller
- 2 servomotors
- MPU6050 IMU sensor
- Ball-on-Plate mechanical platform

## 3. Run the system

Start the main application with:

```bash
python src/main.py
```

## 4. Stop the application

To stop the program, use:

```text
Ctrl + C
```

The system will then terminate the control loop and stop the application.

## Notes

The application requires access to the Raspberry Pi camera and I2C interface.

Make sure that the camera and I2C interfaces are enabled in the Raspberry Pi configuration before running the system.
