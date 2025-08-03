/**
 * @file low_level_mpu6050_interfacing.ino
 * @brief Implementing bit-banging I2C driver to interface
 * with MPU6050 sensor without using standard libraries.
 *
 * Low-level I2C protocol, MPU6050 driver functions to read accelerometer and gyroscope data directly from the sensor.
 * @author Karthik M Dani (https://karthikdani.vercel.app)
 * @date August 2, 2025
 */

// -- I2C Bit-Banging Functions ---

/**
 * @defgroup i2c_bit_banging I2C Bit-Banging
 * @brief Low-level functions for manual I2C communication.
 */

/// @ingroup i2c_bit_banging
/// @brief I2C Serial Data (SDA) line.
#define I2C_SDA_PIN A4

/// @ingroup i2c_bit_banging
/// @brief I2C Serial Clock (SCL) line.
#define I2C_SCL_PIN A5

/// @ingroup i2c_bit_banging
/// @brief delay in uSeconds for I2C timing specs.
#define I2C_DELAY_US 5

/**
 * @defgroup mpu6050_driver MPU6050 Driver
 * @brief High-level functions for communicating with the MPU6050 sensor.
 */

/// @ingroup mpu6050_driver
/// @brief The default I2C address of the MPU6050 (when AD0 is grounded).
#define MPU6050_ADDRESS 0x68

/// @ingroup mpu6050_driver
/// @brief The address of the WHO_AM_I register
#define MPU6050_WHO_AM_I_REG 0x75

/// @ingroup mpu6050_driver
/// @brief The address of the Power Management 1 register.
#define MPU6050_PWR_MGMT_1_REG 0x6B

/// @ingroup mpu6050_driver
/// @brief The address of the high byte of the X-axis accelerometer data.
#define MPU6050_ACCEL_XOUT_H_REG 0x3B

/// @ingroup mpu6050_driver
/// @brief The address of the low byte of the X-axis accelerometer data.
#define MPU6050_ACCEL_XOUT_L_REG 0x3C

/// @ingroup mpu6050_driver
/// @brief The address of the high byte of the X-axis gyroscope data.
#define MPU6050_GYRO_XOUT_H_REG 0x43

/// @ingroup mpu6050_driver
/// @brief The address of the low byte of the X-axis gyroscope data.
#define MPU6050_GYRO_XOUT_L_REG 0x44

/// @ingroup mpu6050_driver
/// @brief The address of the high byte of the temperature data.
#define MPU6050_TEMP_OUT_H_REG 0x41

/**
 * @ingroup mpu6050_driver
 * @struct accel_data_t
 * @brief struct to hold raw 16-bit accelerometer data.
 */
typedef struct {
  int16_t x; ///< Raw X-axis accelerometer value.
  int16_t y; ///< Raw Y-axis accelerometer value.
  int16_t z; ///< Raw Z-axis accelerometer value.
} accel_data_t;

/**
 * @ingroup mpu6050_driver
 * @struct gyro_data_t
 * @brief struct to hold the raw 16-bit gyroscope data.
 */
typedef struct {
  int16_t x; ///< Raw X-axis gyroscope value.
  int16_t y; ///< Raw Y-axis gyroscope value.
  int16_t z; ///< Raw Z-axis gyroscope value.
} gyro_data_t;

/**
 * @ingroup mpu6050_driver
 * @struct accel_float_data_t
 * @brief struct to hold the converted float accelerometer data in g's.
 */
typedef struct {
  float x; ///< X-axis acceleration in g's.
  float y; ///< Y-axis acceleration in g's.
  float z; ///< Z-axis acceleration in g's.
} accel_float_data_t;

/**
 * @ingroup mpu6050_driver
 * @struct gyro_float_data_t
 * @brief struct to hold converted float gyroscope data (in degrees per second).
 */
typedef struct {
  float x; ///< X-axis angular velocity in degrees per second.
  float y; ///< Y-axis angular velocity in degrees per second.
  float z; ///< Z-axis angular velocity in degrees per second.
} gyro_float_data_t;

/**
 * @ingroup mpu6050_driver
 * @struct temp_float_data_t
 * @brief struct to hold the converted float temperature data (in Celsius).
 */
typedef struct {
  float c; ///< Temperature in degrees Celsius.
} temp_float_data_t;

/**
 * @ingroup i2c_bit_banging
 * @brief Sets the SDA line high by configuring the pin as an input with the internal pull-up disabled.
 *
 * I2C requires the bus lines to be open-drain, so setting a pin as an input
 * "releases" it, allowing an external pull-up resistor to pull it high.
 */
void i2c_sda_high() {
  pinMode(I2C_SDA_PIN, INPUT);
}

/**
 * @ingroup i2c_bit_banging
 * @brief Sets the SDA line low.
 *
 * This is achieved by setting the pin as an output and driving it low.
 */
void i2c_sda_low() {
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);
}

/**
 * @ingroup i2c_bit_banging
 * @brief Sets the SCL line high.
 *
 * This is achieved by setting the pin as an input to release it.
 */
void i2c_scl_high() {
  pinMode(I2C_SCL_PIN, INPUT);
}

/**
 * @ingroup i2c_bit_banging
 * @brief Sets the SCL line low.
 *
 * This is achieved by setting the pin as an output and driving it low.
 */
void i2c_scl_low() {
  pinMode(I2C_SCL_PIN, OUTPUT);
  digitalWrite(I2C_SCL_PIN, LOW);
}

/**
 * @ingroup i2c_bit_banging
 * @brief Reads the current state of the SDA pin.
 *
 * @return `true` if the pin is high, `false` if it is low.
 */
bool i2c_read_sda() {
  pinMode(I2C_SDA_PIN, INPUT);
  return digitalRead(I2C_SDA_PIN);
}

/**
 * @ingroup i2c_bit_banging
 * @brief Generates the I2C START condition.
 *
 * An I2C START condition is a high-to-low transition on the SDA line
 * while the SCL line is high.
 */
void i2c_start() {
  i2c_sda_high();
  i2c_scl_high();
  delayMicroseconds(I2C_DELAY_US);
  i2c_sda_low();
  delayMicroseconds(I2C_DELAY_US);
  i2c_scl_low();
  delayMicroseconds(I2C_DELAY_US);
}

/**
 * @ingroup i2c_bit_banging
 * @brief Generates the I2C STOP condition.
 *
 * An I2C STOP condition is a low-to-high transition on the SDA line
 * while the SCL line is high.
 */
void i2c_stop() {
  i2c_sda_low();
  i2c_scl_high();
  delayMicroseconds(I2C_DELAY_US);
  i2c_sda_high();
  delayMicroseconds(I2C_DELAY_US);
}

/**
 * @ingroup i2c_bit_banging
 * @brief Writes a single byte over the I2C bus.
 *
 * This function sends a single byte of data, most significant bit (MSB) first,
 * and then waits for an acknowledgment (ACK) from the slave device.
 *
 * @param data The byte to write.
 * @return `true` if an ACK is received (SDA is pulled low by the slave),
 * `false` otherwise (NACK).
 */
bool i2c_write_byte(uint8_t data) {
  // Loop through each of the 8 bits, MSB first.
  for (int i = 7; i >= 0; i--) {
    // Set the SDA line based on the current bit.
    if ((data >> i) & 0x01) {
      i2c_sda_high();
    } else {
      i2c_sda_low();
    }
    delayMicroseconds(I2C_DELAY_US);

    // Clock the bit out by pulsing SCL high.
    i2c_scl_high();
    delayMicroseconds(I2C_DELAY_US);
    i2c_scl_low();
    delayMicroseconds(I2C_DELAY_US);
  }

  // The 9th clock pulse is for the ACK/NACK.
  // The slave will pull SDA low for an ACK.
  i2c_sda_high(); // Release the SDA line.
  delayMicroseconds(I2C_DELAY_US);
  i2c_scl_high();
  delayMicroseconds(I2C_DELAY_US);
  bool ack = !i2c_read_sda(); // Read the ACK bit.
  i2c_scl_low();
  delayMicroseconds(I2C_DELAY_US);
  i2c_sda_low(); // Restore SDA to low for the next operation.

  return ack;
}

/**
 * @ingroup i2c_bit_banging
 * @brief Reads a single byte from the I2C bus.
 *
 * This function reads a byte from the slave device and then sends an
 * acknowledgment (ACK) or non-acknowledgment (NACK) based on the `ack`
 * parameter.
 *
 * @param ack A boolean flag to determine whether to send an ACK (`true`)
 * or a NACK (`false`) after reading the byte.
 * @return The 8-bit data byte read from the bus.
 */
uint8_t i2c_read_byte(bool ack) {
  uint8_t data = 0;
  i2c_sda_high(); // Release the SDA line to allow the slave to drive it.

  // Read 8 bits.
  for (int i = 7; i >= 0; i--) {
    i2c_scl_high();
    delayMicroseconds(I2C_DELAY_US);
    if (i2c_read_sda()) {
      data |= (1 << i);
    }
    i2c_scl_low();
    delayMicroseconds(I2C_DELAY_US);
  }

  // Send ACK or NACK.
  if (ack) {
    i2c_sda_low(); // Pull SDA low for an ACK.
  } else {
    i2c_sda_high(); // Keep SDA high for a NACK.
  }
  delayMicroseconds(I2C_DELAY_US);
  i2c_scl_high();
  delayMicroseconds(I2C_DELAY_US);
  i2c_scl_low();
  delayMicroseconds(I2C_DELAY_US);
  i2c_sda_low(); // Restore SDA low for the next operation.

  return data;
}

/**
 * @ingroup mpu6050_driver
 * @brief Writes a single byte to a specified register on the MPU6050.
 *
 * This function performs a standard I2C write transaction:
 * START -> (slave address + WRITE) -> (register address) -> (data) -> STOP.
 *
 * @param reg The register address to write to.
 * @param value The 8-bit value to write to the register.
 */
void mpu6050_write_register(uint8_t reg, uint8_t value) {
  i2c_start();
  i2c_write_byte(MPU6050_ADDRESS << 1 | 0x00); // Write address (0)
  i2c_write_byte(reg);                       // Register address
  i2c_write_byte(value);                     // Data to write
  i2c_stop();
}

/**
 * @ingroup mpu6050_driver
 * @brief Reads a single byte from a specified register on the MPU6050.
 *
 * This function performs a standard I2C read transaction:
 * 1. Write: START -> (slave address + WRITE) -> (register address) -> STOP
 * 2. Read: START -> (slave address + READ) -> (read data) -> NACK -> STOP
 *
 * @param reg The register address to read from.
 * @return The 8-bit value read from the register.
 */
uint8_t mpu6050_read_register(uint8_t reg) {
  uint8_t data;
  i2c_start();
  i2c_write_byte(MPU6050_ADDRESS << 1 | 0x00); // Write address (0)
  i2c_write_byte(reg);                       // Register address
  i2c_stop();

  i2c_start();
  i2c_write_byte(MPU6050_ADDRESS << 1 | 0x01); // Read address (1)
  data = i2c_read_byte(false);               // Read the byte and send NACK
  i2c_stop();

  return data;
}

/**
 * @ingroup mpu6050_driver
 * @brief Reads multiple consecutive bytes from the MPU6050, starting from a given register.
 *
 * @param reg The starting register address to read from.
 * @param buffer A pointer to a buffer where the read data will be stored.
 * @param count The number of bytes to read.
 */
void mpu6050_read_bytes(uint8_t reg, uint8_t* buffer, uint8_t count) {
  i2c_start();
  i2c_write_byte(MPU6050_ADDRESS << 1 | 0x00); // Write address (0)
  i2c_write_byte(reg);                       // Starting register address
  i2c_stop();

  i2c_start();
  i2c_write_byte(MPU6050_ADDRESS << 1 | 0x01); // Read address (1)
  for (int i = 0; i < count; i++) {
    buffer[i] = i2c_read_byte(i < count - 1); // ACK for all but the last byte
  }
  i2c_stop();
}

/**
 * @ingroup mpu6050_driver
 * @brief Reads and combines the raw 16-bit accelerometer data into a struct.
 *
 * @param accel A pointer to an `accel_data_t` struct to store the raw data.
 */
void mpu6050_read_accel_data(accel_data_t* accel) {
  uint8_t buffer[6];
  mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H_REG, buffer, 6);

  accel->x = (int16_t)(buffer[0] << 8 | buffer[1]);
  accel->y = (int16_t)(buffer[2] << 8 | buffer[3]);
  accel->z = (int16_t)(buffer[4] << 8 | buffer[5]);
}

/**
 * @ingroup mpu6050_driver
 * @brief Reads and combines the raw 16-bit gyroscope data into a struct.
 *
 * @param gyro A pointer to a `gyro_data_t` struct to store the raw data.
 */
void mpu6050_read_gyro_data(gyro_data_t* gyro) {
  uint8_t buffer[6];
  mpu6050_read_bytes(MPU6050_GYRO_XOUT_H_REG, buffer, 6);

  gyro->x = (int16_t)(buffer[0] << 8 | buffer[1]);
  gyro->y = (int16_t)(buffer[2] << 8 | buffer[3]);
  gyro->z = (int16_t)(buffer[4] << 8 | buffer[5]);
}

/**
 * @ingroup mpu6050_driver
 * @brief Reads raw accelerometer data and converts it to G's.
 *
 * The conversion assumes the sensor is configured for a +/-2g range.
 *
 * @param accel_g A pointer to an `accel_float_data_t` struct to store the converted data.
 */
void mpu6050_read_accel_g(accel_float_data_t* accel_g) {
  accel_data_t raw_accel;
  mpu6050_read_accel_data(&raw_accel);

  // The MPU6050 is configured for +/-2g range, with a resolution of 16 bits.
  // The conversion factor is 2.0g / 32768.0 LSB.
  const float scale_factor = 2.0 / 32768.0;

  accel_g->x = (float)raw_accel.x * scale_factor;
  accel_g->y = (float)raw_accel.y * scale_factor;
  accel_g->z = (float)raw_accel.z * scale_factor;
}

/**
 * @ingroup mpu6050_driver
 * @brief Reads raw gyroscope data and converts it to degrees per second.
 *
 * The conversion assumes the sensor is configured for a +/-250 dps range.
 *
 * @param gyro_dps A pointer to a `gyro_float_data_t` struct to store the converted data.
 */
void mpu6050_read_gyro_dps(gyro_float_data_t* gyro_dps) {
  gyro_data_t raw_gyro;
  mpu6050_read_gyro_data(&raw_gyro);

  // The MPU6050 is configured for +/-250 dps range, with a resolution of 16 bits.
  // The conversion factor is 250.0 dps / 32768.0 LSB.
  const float scale_factor = 250.0 / 32768.0;

  gyro_dps->x = (float)raw_gyro.x * scale_factor;
  gyro_dps->y = (float)raw_gyro.y * scale_factor;
  gyro_dps->z = (float)raw_gyro.z * scale_factor;
}

/**
 * @ingroup mpu6050_driver
 * @brief Reads temperature data and converts it to Celsius.
 *
 * The conversion uses the formula provided in the MPU6050 datasheet.
 *
 * @param temp_c A pointer to a `temp_float_data_t` struct to store the converted data.
 */
void mpu6050_read_temp_celsius(temp_float_data_t* temp_c) {
  uint8_t buffer[2];
  mpu6050_read_bytes(MPU6050_TEMP_OUT_H_REG, buffer, 2);

  int16_t raw_temp = (int16_t)(buffer[0] << 8 | buffer[1]);

  // The conversion formula for MPU6050 temperature is from the datasheet:
  // Temperature in C = ((raw_temp) / 340.0) + 36.53
  temp_c->c = ((float)raw_temp / 340.0) + 36.53;
}

// --- Part 3: Main Sketch ---

/**
 * @defgroup main_sketch Main Sketch
 * @brief The setup and loop functions for the Arduino program.
 */

/**
 * @ingroup main_sketch
 * @brief The setup function, called once at the start of the program.
 *
 * It initializes the serial monitor, sets up the I2C pins, and
 * verifies communication with the MPU6050 sensor by checking its
 * WHO_AM_I register.
 */
void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for the Serial port to connect.

  // Set the SCL and SDA pins as outputs initially, and then let the
  // I2C functions manage them.
  pinMode(I2C_SDA_PIN, OUTPUT);
  pinMode(I2C_SCL_PIN, OUTPUT);

  // Initialize the MPU6050.
  // The sensor starts in sleep mode, so we need to wake it up.
  // We'll write a 0 to the PWR_MGMT_1 register to wake it up.
  mpu6050_write_register(MPU6050_PWR_MGMT_1_REG, 0);

  // Verify the MPU6050 is connected by reading the WHO_AM_I register.
  // It should return 0x68.
  uint8_t who_am_i = mpu6050_read_register(MPU6050_WHO_AM_I_REG);
  if (who_am_i != MPU6050_ADDRESS) {
    Serial.print("MPU6050 not found! WHO_AM_I returned: 0x");
    Serial.println(who_am_i, HEX);
    while (1); // Halt the program if the sensor is not found.
  } else {
    Serial.println("MPU6050 found and initialized successfully.");
  }
}

/**
 * @ingroup main_sketch
 * @brief The main loop, which runs repeatedly after setup.
 *
 * It continuously reads accelerometer, gyroscope, and temperature data from the
 * MPU6050, converts it to engineering units (g's, dps, and Celsius),
 * and prints the results to the serial monitor every 100 milliseconds.
 */
void loop() {
  accel_float_data_t accel_g;
  gyro_float_data_t gyro_dps;
  temp_float_data_t temp_c;

  // Read the latest sensor data and convert to float.
  mpu6050_read_accel_g(&accel_g);
  mpu6050_read_gyro_dps(&gyro_dps);
  mpu6050_read_temp_celsius(&temp_c);

  // converted sensor data
  Serial.print("Accel: ");
  Serial.print("X=");
  Serial.print(accel_g.x);
  Serial.print("g, Y=");
  Serial.print(accel_g.y);
  Serial.print("g, Z=");
  Serial.print(accel_g.z);
  Serial.print("g");

  Serial.print(" | Gyro: ");
  Serial.print("X=");
  Serial.print(gyro_dps.x);
  Serial.print("dps, Y=");
  Serial.print(gyro_dps.y);
  Serial.print("dps, Z=");
  Serial.print(gyro_dps.z);
  Serial.print("dps");
  
  Serial.print(" | Temp: ");
  Serial.print(temp_c.c);
  Serial.println(" C");

  delay(100);
}
