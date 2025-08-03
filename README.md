# Low-Level-MPU6050-Arduino
https://github.com/user-attachments/assets/b1870fec-6dee-4ed4-876d-0eb12d9ec74f

## Intro
This project meant as a technical assessment for HyPrix Aviation implements Custom Low Level driver for I2C as well as MPU6050 sensor instead of relying on the standard libraries, bit-banging the I2C protocol, giving us full control over the communication process. This also gives SPEED that's prime requirement in Aviation (as the unneccessary code is avoided).

A set of high-level functions designed specifically to interact with the MPU6050, handling register reads, writes, and data acquisition, the code reads raw 16-bit data from the sensor and then converts it into human-readable engineering units:
- Acceleration in G's
- Angular velocity in degrees per second (dps)
- Temperature in Celsius

## Technical Breakdown
The project is thoughtfully structured into two main layers, organized by Doxygen-style comments in the code.

### I2C Bit-Banging Layer
This is the core of the low-level communication. Functions like `i2c_start()`, `i2c_stop()`, and `i2c_write_byte()` manually manipulate the digital pins on the Arduino to create the specific timing and voltage transitions required for the I2C protocol.

### MPU6050 Driver Layer
Sitting on top of the I2C layer, this driver provides an easy-to-use interface for the `MPU6050`. It handles the specific addresses and register maps of the sensor. For example, `mpu6050_read_accel_data()` uses the low-level I2C functions to fetch and combine the correct bytes from the sensor's registers.

## Expected Output
Upon a successful connection (as provided in the demo video), the serial monitor will display MPU6050 found and initialized successfully.

The program will then continuously print the converted accelerometer, gyroscope, and temperature data.

---

My Portfolio: https://karthikdani.vercel.app
