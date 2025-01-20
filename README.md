Step Counter Using BMI160 Sensor on nRF52 📱💪

This repository showcases the implementation of a step counter using the BMI160 motion sensor and an nRF52 Development Kit. The project leverages the I2C interface to communicate with the BMI160 sensor and uses Bluetooth Low Energy (BLE) to send real-time step count data to the nRF Connect app.

Features ✨
BMI160 Sensor Integration: Configured for step detection and counting using its motion-sensing capabilities. 🦵

I2C Communication: Interfacing the BMI160 sensor with the nRF52 via I2C (SCL: pin 27, SDA: pin 26). 🔌

Real-Time Step Data Transmission: Step count data is transmitted to the nRF Connect app over BLE. 📡

Interrupt-Driven Design: Utilizes the BMI160's interrupt pin for efficient and responsive step counting. ⚡

Components Used 🧰
nRF52 Development Kit (nRF52832 SoC)
BMI160 Sensor (connected via I2C)
nRF5 SDK v15.3 and SoftDevice S132
nRF Connect App (to visualize step count in real time)

How It Works 🔧
The BMI160 is configured to detect steps and trigger an interrupt for each step detected. 🦶
The nRF52 reads the step count data from the BMI160 sensor via I2C. 🔄
The step count is updated and transmitted via BLE to the nRF Connect app. 📲

Setup Instructions ⚙️
Connect the BMI160 sensor to the nRF52 development kit:
SCL: Pin 27
SDA: Pin 26
VDD: 3.3V
GND: Ground
Interrupt Pin: Connect to Pin 27 (configurable).

Clone this repository and open the project in SEGGER Embedded Studio (v5.30). 💻
Compile and flash the firmware onto the nRF52 Development Kit. 🔥
Open the nRF Connect app and connect to the device to view step count updates in real time. 📶

Future Improvements 🚀
Add support for more motion-related features (e.g., activity recognition). 🏃‍♂️
Integrate additional BLE services for enhanced data visualization. 📊
Optimize power consumption for portable use cases. 🔋

Acknowledgments 🙏
This project was developed as part of an exploration into sensor integration and BLE-based data transmission using Nordic Semiconductor's nRF52 platform. 🌍
