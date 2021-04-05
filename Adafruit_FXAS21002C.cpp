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
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <limits.h>

#include "Adafruit_FXAS21002C.h"

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
    @param reg The register address to write to
    @param value The value to write to the specified register
*/
/**************************************************************************/
void Adafruit_FXAS21002C::write8(byte reg, byte value) {
  Wire.beginTransmission(_sensorAddr);
#if ARDUINO >= 100
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
#else
  Wire.send(reg);
  Wire.send(value);
#endif
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
    @param reg The register address to read from
    @returns The byte read from the I2C bus at the specified reg
*/
/**************************************************************************/
byte Adafruit_FXAS21002C::read8(byte reg) {
  byte value;

  Wire.beginTransmission((byte)_sensorAddr);
#if ARDUINO >= 100
  Wire.write((uint8_t)reg);
#else
  Wire.send(reg);
#endif
  if (Wire.endTransmission(false) != 0)
    return 0;
  Wire.requestFrom((byte)_sensorAddr, (byte)1);
#if ARDUINO >= 100
  value = Wire.read();
#else
  value = Wire.receive();
#endif

  return value;
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_FXAS21002C class, including assigning
            a unique ID to the gyroscope for logging purposes.

    @param sensorID The unique ID to associate with the gyroscope.
    @param addr The I2C address of the sensor.
*/
/**************************************************************************/
Adafruit_FXAS21002C::Adafruit_FXAS21002C(int32_t sensorID, byte addr) {
  _sensorID = sensorID;
  _sensorAddr = addr;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setup the HW

    @param  rng
            The range to set for the gyroscope, based on gyroRange_t

    @return True if the device was successfully initialized, otherwise false.
*/
/**************************************************************************/
bool Adafruit_FXAS21002C::begin(gyroRange_t rng) {
  /* Enable I2C */
  Wire.begin();

  /* Set the range the an appropriate value */
  _range = rng;

  /* Clear the raw sensor data */
  raw.x = 0;
  raw.y = 0;
  raw.z = 0;

  /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
  uint8_t id = read8(GYRO_REGISTER_WHO_AM_I);
  // Serial.print("WHO AM I? 0x"); Serial.println(id, HEX);
  if (id != FXAS21002C_ID) {
    return false;
  }

  /* Set CTRL_REG1 (0x13)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     6  RESET     Reset device on 1                                   0
     5  ST        Self test enabled on 1                              0
   4:2  DR        Output data rate                                  000
                  000 = 800 Hz
                  001 = 400 Hz
                  010 = 200 Hz
                  011 = 100 Hz
                  100 = 50 Hz
                  101 = 25 Hz
                  110 = 12.5 Hz
                  111 = 12.5 Hz
     1  ACTIVE    Standby(0)/Active(1)                                0
     0  READY     Standby(0)/Ready(1)                                 0
  */

  /* Set CTRL_REG0 (0x0D)  Default value 0x00
  =====================================================================
  BIT  Symbol     Description                                   Default
  7:6  BW         cut-off frequency of low-pass filter               00
    5  SPIW       SPI interface mode selection                        0
  4:3  SEL        High-pass filter cutoff frequency selection        00
    2  HPF_EN     High-pass filter enable                             0
  1:0  FS         Full-scale range selection
                  00 = +-2000 dps
                  01 = +-1000 dps
                  10 = +-500 dps
                  11 = +-250 dps
  The bit fields in CTRL_REG0 should be changed only in Standby or Ready modes.
  */

  uint8_t ctrlReg0 = 0x00;

  switch (_range) {
  case GYRO_RANGE_250DPS:
    ctrlReg0 = 0x03;
    break;
  case GYRO_RANGE_500DPS:
    ctrlReg0 = 0x02;
    break;
  case GYRO_RANGE_1000DPS:
    ctrlReg0 = 0x01;
    break;
  case GYRO_RANGE_2000DPS:
    ctrlReg0 = 0x00;
    break;
  }

  /* Reset then switch to active mode with 100Hz output */
  write8(GYRO_REGISTER_CTRL_REG1, 0x00);     // Standby
  write8(GYRO_REGISTER_CTRL_REG1, (1 << 6)); // Reset
  write8(GYRO_REGISTER_CTRL_REG0, ctrlReg0); // Set sensitivity
  write8(GYRO_REGISTER_CTRL_REG1, 0x0E);     // Active
  delay(100);                                // 60 ms + 1/ODR

  return true;
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
  bool readingValid = false;

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
  Wire.beginTransmission((byte)_sensorAddr);
#if ARDUINO >= 100
  Wire.write(GYRO_REGISTER_STATUS | 0x80);
#else
  Wire.send(GYRO_REGISTER_STATUS | 0x80);
#endif
  Wire.endTransmission();
  Wire.requestFrom((byte)_sensorAddr, (byte)7);

#if ARDUINO >= 100
  uint8_t status = Wire.read();
  uint8_t xhi = Wire.read();
  uint8_t xlo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t ylo = Wire.read();
  uint8_t zhi = Wire.read();
  uint8_t zlo = Wire.read();
#else
  uint8_t status = Wire.receive();
  uint8_t xhi = Wire.receive();
  uint8_t xlo = Wire.receive();
  uint8_t yhi = Wire.receive();
  uint8_t ylo = Wire.receive();
  uint8_t zhi = Wire.receive();
  uint8_t zlo = Wire.receive();
#endif

  /* Shift values to create properly formed integer */
  event->gyro.x = (int16_t)((xhi << 8) | xlo);
  event->gyro.y = (int16_t)((yhi << 8) | ylo);
  event->gyro.z = (int16_t)((zhi << 8) | zlo);

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
  sensor->min_value = (this->_range * -1.0) * SENSORS_DPS_TO_RADS;
  sensor->resolution = 0.0F; // TBD
}

/**************************************************************************/
/*!
    @brief  Puts devince into/out of standby mode
    @param  standby Whether we want to go into standby!
*/
/**************************************************************************/
void Adafruit_FXAS21002C::standby(boolean standby) {

  uint8_t reg1 = read8(GYRO_REGISTER_CTRL_REG1);
  if (standby) {
    reg1 &= ~(0x03);
  } else {
    reg1 |= (0x03);
  }
  write8(GYRO_REGISTER_CTRL_REG1, reg1);

  if (!standby) {
    delay(100);
  }
}
