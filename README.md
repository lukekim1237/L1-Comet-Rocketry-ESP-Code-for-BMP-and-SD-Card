# L1-Comet-Rocketry-ESP-Code-for-BMP-and-SD-Card


My main code collects and logs sensor data from a BMP280 (measuring temperature, pressure, and altitude) and an MPU6050 (measuring acceleration and gyroscopic data) onto an SD card during the rocket's flight. It uses a buffer to store multiple sensor readings and appends them to a CSV file in chunks, ensuring efficient data logging while providing a timestamp for each record.
