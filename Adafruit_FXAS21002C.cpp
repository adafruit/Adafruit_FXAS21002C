/*!
 * @file Adafruit_FXAS21002C.cpp
 *
 * @mainpage Adafruit FXAS21002C gyroscope sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's FXAS21002C driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit FXAS21002C breakout: https://www.adafruit.com/products/3463
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a
 * href="https://github.com/adafruit/Adafruit_Sensor"> Adafruit_Sensor</a> being
 * present on your system. Please make sure you have installed the latest
 * version before using this library.
 *
 * @section author Author
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#include "Adafruit_FXAS21002C.h"
#include <limits.h>

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
     @brief  Initializes the hardware to a default state.

     @return True if the device was successfully initialized, otherwise false.
*/
/**************************************************************************/
bool Adafruit_FXAS21002C::initialize() {
  Adafruit_BusIO_Register CTRL_REG0(i2c_dev, GYRO_REGISTER_CTRL_REG0);
  Adafruit_BusIO_Register CTRL_REG1(i2c_dev, GYRO_REGISTER_CTRL_REG1);

  /* Set the range the an appropriate value */
  _range = GYRO_RANGE_250DPS;

  /* Clear the raw sensor data */
  raw.x = 0;
  raw.y = 0;
  raw.z = 0;

  /* Reset then switch to active mode with 100Hz output */
  CTRL_REG1.write(0x00);   // Standby
  CTRL_REG1.write(1 << 6); // Reset
  CTRL_REG0.write(0x03);   // Set full scale range to +-250 dps
  _ODR = GYRO_ODR_100HZ;   // Update global ODR variable
  CTRL_REG1.write(0x0E);   // Active
  delay(100);              // 60ms + 1/ODR

  return true;
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_FXAS21002C class, including assigning
            a unique ID to the gyroscope for logging purposes.

    @param sensorID The unique ID to associate with the gyroscope.
*/
/**************************************************************************/
Adafruit_FXAS21002C::Adafruit_FXAS21002C(int32_t sensorID) {
  _sensorID = sensorID;
}

/***************************************************************************
 DESTRUCTOR
 ***************************************************************************/

Adafruit_FXAS21002C::~Adafruit_FXAS21002C() {
  if (i2c_dev)
    delete i2c_dev;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setup the HW

    @param addr The I2C address of the sensor.
    @param wire Pointer to Wire instance

    @return True if the device was successfully initialized, otherwise false.
*/
/**************************************************************************/
bool Adafruit_FXAS21002C::begin(uint8_t addr, TwoWire *wire) {

  if (i2c_dev)
    delete i2c_dev;
  i2c_dev = new Adafruit_I2CDevice(addr, wire);
  if (!i2c_dev->begin())
    return false;

  Adafruit_BusIO_Register WHO_AM_I(i2c_dev, GYRO_REGISTER_WHO_AM_I);
  if (WHO_AM_I.read() != FXAS21002C_ID)
    return false;

  return initialize();
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event

    @param[out] event
                A reference to the sensors_event_t instances where the
                accelerometer data should be written.

     @return True if the event was successfully read, otherwise false.
*/
/**************************************************************************/
bool Adafruit_FXAS21002C::getEvent(sensors_event_t *event) {
  // bool readingValid = false;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  /* Clear the raw data placeholder */
  raw.x = 0;
  raw.y = 0;
  raw.z = 0;

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_GYROSCOPE;
  event->timestamp = millis();

  /* Read 7 bytes from the sensor */
  uint8_t buffer[7] = {0};
  buffer[0] = GYRO_REGISTER_STATUS;
  i2c_dev->write_then_read(buffer, 1, buffer, 7);

  /* Shift values to create properly formed integer */
  event->gyro.x = (int16_t)((buffer[1] << 8) | buffer[2]);
  event->gyro.y = (int16_t)((buffer[3] << 8) | buffer[4]);
  event->gyro.z = (int16_t)((buffer[5] << 8) | buffer[6]);

  /* Assign raw values in case someone needs them */
  raw.x = event->gyro.x;
  raw.y = event->gyro.y;
  raw.z = event->gyro.z;

  /* Compensate values depending on the resolution */
  switch (_range) {
  case GYRO_RANGE_250DPS:
    event->gyro.x *= GYRO_SENSITIVITY_250DPS;
    event->gyro.y *= GYRO_SENSITIVITY_250DPS;
    event->gyro.z *= GYRO_SENSITIVITY_250DPS;
    break;
  case GYRO_RANGE_500DPS:
    event->gyro.x *= GYRO_SENSITIVITY_500DPS;
    event->gyro.y *= GYRO_SENSITIVITY_500DPS;
    event->gyro.z *= GYRO_SENSITIVITY_500DPS;
    break;
  case GYRO_RANGE_1000DPS:
    event->gyro.x *= GYRO_SENSITIVITY_1000DPS;
    event->gyro.y *= GYRO_SENSITIVITY_1000DPS;
    event->gyro.z *= GYRO_SENSITIVITY_1000DPS;
    break;
  case GYRO_RANGE_2000DPS:
    event->gyro.x *= GYRO_SENSITIVITY_2000DPS;
    event->gyro.y *= GYRO_SENSITIVITY_2000DPS;
    event->gyro.z *= GYRO_SENSITIVITY_2000DPS;
    break;
  }

  /* Convert values to rad/s */
  event->gyro.x *= SENSORS_DPS_TO_RADS;
  event->gyro.y *= SENSORS_DPS_TO_RADS;
  event->gyro.z *= SENSORS_DPS_TO_RADS;

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data

    @param[out] sensor
                A reference to the sensor_t instances where the
                gyroscope sensor info should be written.
*/
/**************************************************************************/
void Adafruit_FXAS21002C::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "FXAS21002C", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_GYROSCOPE;
  sensor->min_delay = 0;
  sensor->max_value = (float)this->_range * SENSORS_DPS_TO_RADS;
  sensor->min_value = ((float)this->_range * -1.0) * SENSORS_DPS_TO_RADS;
  sensor->resolution = 0.0F; // TBD
}

/**************************************************************************/
/*!
    @brief  Set the gyroscope full scale range.
    @param  range gyroscope full scale range
*/
/**************************************************************************/
void Adafruit_FXAS21002C::setRange(gyroRange_t range) {
  Adafruit_BusIO_Register CTRL_REG0(i2c_dev, GYRO_REGISTER_CTRL_REG0);
  Adafruit_BusIO_RegisterBits range_bits(&CTRL_REG0, 2, 0);

  standby(true);

  /* write FS[1:0] bits (bits controlling the full scale range) with correct
   * values according to page 40 of the datasheet */
  switch (range) {
  case GYRO_RANGE_250DPS:
    range_bits.write(0b11);
    break;
  case GYRO_RANGE_500DPS:
    range_bits.write(0b10);
    break;
  case GYRO_RANGE_1000DPS:
    range_bits.write(0b01);
    break;
  case GYRO_RANGE_2000DPS:
    range_bits.write(0b00);
    break;
  }

  standby(false);

  _range = range;
}

/**************************************************************************/
/*!
    @brief  Get the gyroscope full scale range.
    @return  gyroscope full scale range
*/
/**************************************************************************/
gyroRange_t Adafruit_FXAS21002C::getRange() { return _range; }

/**************************************************************************/
/*!
    @brief  Puts device into/out of standby mode
    @param  standby Whether we want to go into standby!
*/
/**************************************************************************/
void Adafruit_FXAS21002C::standby(boolean standby) {
  Adafruit_BusIO_Register CTRL_REG1(i2c_dev, GYRO_REGISTER_CTRL_REG1);
  Adafruit_BusIO_RegisterBits active_bit(&CTRL_REG1, 2, 0);

  if (standby) {
    active_bit.write(0x00);
    delay(100);
  } else {
    active_bit.write(0x03);
  }
}

/**************************************************************************/
/*!
    @brief  Configures the device with certain output data rate(ODR)
            Supports ODRs: 800.0Hz, 400.0Hz, 200.0Hz,
   100.0Hz, 50.0Hz, 25.0Hz, 12.5Hz
    @param   ODR : the output data rate to be set to the gyroscope
*/
/**************************************************************************/
void Adafruit_FXAS21002C::setODR(float ODR) {
  Adafruit_BusIO_Register CTRL_REG1(i2c_dev, GYRO_REGISTER_CTRL_REG1);
  Adafruit_BusIO_RegisterBits datarate_bits(&CTRL_REG1, 3, 2);

  /* CTRL_REG1 should only be set in Standby or Ready mode. First enter Standby
   * mode */
  standby(true);
  /* _ODR is only updated if the input ODR is one of the valid ODRs */
  if (ODR == GYRO_ODR_800HZ) {
    datarate_bits.write(0b000);
  } else if (ODR == GYRO_ODR_400HZ) {
    datarate_bits.write(0b001);
  } else if (ODR == GYRO_ODR_200HZ) {
    datarate_bits.write(0b010);
  } else if (ODR == GYRO_ODR_100HZ) {
    datarate_bits.write(0b011);
  } else if (ODR == GYRO_ODR_50HZ) {
    datarate_bits.write(0b100);
  } else if (ODR == GYRO_ODR_25HZ) {
    datarate_bits.write(0b101);
  } else if (ODR == GYRO_ODR_12_5HZ) {
    datarate_bits.write(0b110);
  }
  // update internal _ODR variable. Note that this update happens regardless of
  // the validity of ODR
  _ODR = ODR;
  standby(false);
}

/**************************************************************************/
/*!
    @brief  Obtain the current output data rate(ODR) from the gyroscope's
   register
    @return The Output Data Rate(ODR) in Hz
*/
/**************************************************************************/
float Adafruit_FXAS21002C::getODR() { return _ODR; }
