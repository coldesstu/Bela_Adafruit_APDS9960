#include <math.h>
#include <stdlib.h>
#include <cstring>
#include <I2c.h>
#include "Bela_Adafruit_APDS9960.h"

I2c i2c;

/*!
 *  @brief  Implements missing powf function
 *  @param  x
 *          Base number
 *  @param  y
 *          Exponent
 *  @return x raised to the power of y
 */
float powf(const float x, const float y) {
  return (float)(pow((double)x, (double)y));
}

/*!
 *  @brief  Enables the device
 *          Disables the device (putting it in lower power sleep mode)
 *  @param  en
 *          Enable (True/False)
 */
void Adafruit_APDS9960::enable(bool en) {
  _enable.PON = en;
  this->write8(APDS9960_ENABLE, _enable.get());
}

/*!
 *  @brief  Initializes I2C and configures the sensor
 *  @param  iTimeMS
 *          Integration time
 *  @param  aGain
 *          Gain
 *  @param  addr
 *          I2C address
 *  @param  *theWire
 *          Wire object
 *  @return True if initialization was successful, otherwise false.
 */
bool Adafruit_APDS9960::begin(uint16_t iTimeMS, apds9960AGain_t aGain,
                                 uint8_t addr) {

  //if (i2c_dev)
  //  delete i2c_dev;
  //i2c_dev = new Adafruit_I2CDevice(addr, theWire);
  //if (!i2c_dev->begin()) {
  //  return false;
  //}
//   if (i2c) delete i2c;
// i2c = new I2c();
// if (i2c->initI2C_RW(1, addr, 0) != 0) {
//     delete i2c;
//     i2c = nullptr;
//     return false;
// }

  /* Make sure we're actually connected */
  uint8_t x = read8(APDS9960_ID);
  if (x != 0xAB) {
    return false;
  }

  /* Set default integration time and gain */
  setADCIntegrationTime(iTimeMS);
  setADCGain(aGain);

  // disable everything to start
  enableGesture(false);
  enableProximity(false);
  enableColor(false);

  disableColorInterrupt();
  disableProximityInterrupt();
  clearInterrupt();

  /* Note: by default, the device is in power down mode on bootup */
  enable(false);
  //delay(10); //can't use delay for real time audio processing
  enable(true);
  //delay(10);

  // default to all gesture dimensions
  setGestureDimensions(APDS9960_DIMENSIONS_ALL);
  setGestureFIFOThreshold(APDS9960_GFIFO_4);
  setGestureGain(APDS9960_GGAIN_4);
  setGestureProximityThreshold(50);
  resetCounts();

  _gpulse.GPLEN = APDS9960_GPULSE_32US;
  _gpulse.GPULSE = 9; // 10 pulses
  this->write8(APDS9960_GPULSE, _gpulse.get());

  return true;
}

/*!
 *  @brief  Sets the integration time for the ADC of the APDS9960, in millis
 *  @param  iTimeMS
 *          Integration time
 */
void Adafruit_APDS9960::setADCIntegrationTime(uint16_t iTimeMS) {
  float temp;

  // convert ms into 2.78ms increments
  temp = iTimeMS;
  temp /= 2.78;
  temp = 256 - temp;
  if (temp > 255)
    temp = 255;
  if (temp < 0)
    temp = 0;

  /* Update the timing register */
  write8(APDS9960_ATIME, (uint8_t)temp);
}

/*!
 *  @brief  Returns the integration time for the ADC of the APDS9960, in millis
 *  @return Integration time
 */
float Adafruit_APDS9960::getADCIntegrationTime() {
  float temp;

  temp = read8(APDS9960_ATIME);

  // convert to units of 2.78 ms
  temp = 256 - temp;
  temp *= 2.78;
  return temp;
}

/*!
 *  @brief  Adjusts the color/ALS gain on the APDS9960 (adjusts the sensitivity
 *          to light)
 *  @param  aGain
 *          Gain
 */
void Adafruit_APDS9960::setADCGain(apds9960AGain_t aGain) {
  _control.AGAIN = aGain;

  /* Update the timing register */
  write8(APDS9960_CONTROL, _control.get());
}

/*!
 *  @brief  Returns the ADC gain
 *  @return ADC gain
 */
apds9960AGain_t Adafruit_APDS9960::getADCGain() {
  return (apds9960AGain_t)(read8(APDS9960_CONTROL) & 0x03);
}

/*!
 *  @brief  Adjusts the Proximity gain on the APDS9960
 *  @param  pGain
 *          Gain
 */
void Adafruit_APDS9960::setProxGain(apds9960PGain_t pGain) {
  _control.PGAIN = pGain;

  /* Update the timing register */
  write8(APDS9960_CONTROL, _control.get());
}

/*!
 *  @brief  Returns the Proximity gain on the APDS9960
 *  @return Proxmity gain
 */
apds9960PGain_t Adafruit_APDS9960::getProxGain() {
  return (apds9960PGain_t)((read8(APDS9960_CONTROL) & 0x0C) >> 2);
}

/*!
 *  @brief  Sets number of proxmity pulses
 *  @param  pLen
 *          Pulse Length
 *  @param  pulses
 *          Number of pulses
 */
void Adafruit_APDS9960::setProxPulse(apds9960PPulseLen_t pLen, uint8_t pulses) {
  if (pulses < 1)
    pulses = 1;
  if (pulses > 64)
    pulses = 64;
  pulses--;

  _ppulse.PPLEN = pLen;
  _ppulse.PPULSE = pulses;

  write8(APDS9960_PPULSE, _ppulse.get());
}

/*!
 *  @brief  Enable proximity readings on APDS9960
 *  @param  en
 *          Enable (True/False)
 */
void Adafruit_APDS9960::enableProximity(bool en) {
  _enable.PEN = en;

  write8(APDS9960_ENABLE, _enable.get());
}

/*!
 *  @brief  Enable proximity interrupts
 */
void Adafruit_APDS9960::enableProximityInterrupt() {
  _enable.PIEN = 1;
  write8(APDS9960_ENABLE, _enable.get());
  clearInterrupt();
}

/*!
 *  @brief  Disable proximity interrupts
 */
void Adafruit_APDS9960::disableProximityInterrupt() {
  _enable.PIEN = 0;
  write8(APDS9960_ENABLE, _enable.get());
}

/*!
 *  @brief  Set proxmity interrupt thresholds
 *  @param  low
 *          Low threshold
 *  @param  high
 *          High threshold
 *  @param  persistence
 *          Persistence
 */
void Adafruit_APDS9960::setProximityInterruptThreshold(uint8_t low,
                                                       uint8_t high,
                                                       uint8_t persistence) {
  write8(APDS9960_PILT, low);
  write8(APDS9960_PIHT, high);

  if (persistence > 7)
    persistence = 7;
  _pers.PPERS = persistence;
  write8(APDS9960_PERS, _pers.get());
}

/*!
 *  @brief  Returns proximity interrupt status
 *  @return True if enabled, false otherwise.
 */
bool Adafruit_APDS9960::getProximityInterrupt() {
  _status.set(this->read8(APDS9960_STATUS));
  return _status.PINT;
};

/*!
 *  @brief  Read proximity data
 *  @return Proximity
 */
uint8_t Adafruit_APDS9960::readProximity() { return read8(APDS9960_PDATA); }

/*!
 *  @brief  Returns validity status of a gesture
 *  @return Status (True/False)
 */
bool Adafruit_APDS9960::gestureValid() {
  _gstatus.set(this->read8(APDS9960_GSTATUS));
  return _gstatus.GVALID;
}

/*!
 *  @brief  Sets gesture dimensions
 *  @param  dims
 *          Dimensions (APDS9960_DIMENSIONS_ALL, APDS9960_DIMENSIONS_UP_DOWN,
 *          APGS9960_DIMENSIONS_LEFT_RIGHT)
 */
void Adafruit_APDS9960::setGestureDimensions(uint8_t dims) {
  _gconf3.GDIMS = dims;
  this->write8(APDS9960_GCONF3, _gconf3.get());
}

/*!
 *  @brief  Sets gesture FIFO Threshold
 *  @param  thresh
 *          Threshold (APDS9960_GFIFO_1, APDS9960_GFIFO_4, APDS9960_GFIFO_8,
 *          APDS9960_GFIFO_16)
 */
void Adafruit_APDS9960::setGestureFIFOThreshold(uint8_t thresh) {
  _gconf1.GFIFOTH = thresh;
  this->write8(APDS9960_GCONF1, _gconf1.get());
}

/*!
 *  @brief  Sets gesture sensor gain
 *  @param  gain
 *          Gain (APDS9960_GAIN_1, APDS9960_GAIN_2, APDS9960_GAIN_4,
 *          APDS9960_GAIN_8)
 */
void Adafruit_APDS9960::setGestureGain(uint8_t gain) {
  _gconf2.GGAIN = gain;
  this->write8(APDS9960_GCONF2, _gconf2.get());
}

/*!
 *  @brief  Sets gesture sensor threshold
 *  @param  thresh
 *          Threshold
 */
void Adafruit_APDS9960::setGestureProximityThreshold(uint8_t thresh) {
  this->write8(APDS9960_GPENTH, thresh);
}

/*!
 *  @brief  Sets gesture sensor offset
 *  @param  offset_up
 *          Up offset
 *  @param  offset_down
 *          Down offset
 *  @param  offset_left
 *          Left offset
 *  @param  offset_right
 *          Right offset
 */
void Adafruit_APDS9960::setGestureOffset(uint8_t offset_up, uint8_t offset_down,
                                         uint8_t offset_left,
                                         uint8_t offset_right) {
  this->write8(APDS9960_GOFFSET_U, offset_up);
  this->write8(APDS9960_GOFFSET_D, offset_down);
  this->write8(APDS9960_GOFFSET_L, offset_left);
  this->write8(APDS9960_GOFFSET_R, offset_right);
}

/*!
 *  @brief  Enable gesture readings on APDS9960
 *  @param  en
 *          Enable (True/False)
 */
void Adafruit_APDS9960::enableGesture(bool en) {
  if (!en) {
    _gconf4.GMODE = 0;
    write8(APDS9960_GCONF4, _gconf4.get());
  }
  _enable.GEN = en;
  write8(APDS9960_ENABLE, _enable.get());
  resetCounts();
}

/*!
 *  @brief  Resets gesture counts
 */
void Adafruit_APDS9960::resetCounts() {
  gestCnt = 0;
  UCount = 0;
  DCount = 0;
  LCount = 0;
  RCount = 0;
}

/*!
 *  @brief  Set LED brightness for proximity/gesture
 *  @param  drive
 *          LED Drive
 *  @param  boost
 *          LED Boost
 */
void Adafruit_APDS9960::setLED(apds9960LedDrive_t drive,
                               apds9960LedBoost_t boost) {
  // set BOOST
  _config2.LED_BOOST = boost;
  write8(APDS9960_CONFIG2, _config2.get());

  _control.LDRIVE = drive;
  write8(APDS9960_CONTROL, _control.get());
}

/*!
 *  @brief  Enable color readings on APDS9960
 *  @param  en
 *          Enable (True/False)
 */
void Adafruit_APDS9960::enableColor(bool en) {
  _enable.AEN = en;
  write8(APDS9960_ENABLE, _enable.get());
}

/*!
 *  @brief  Returns status of color data
 *  @return True if color data ready, False otherwise
 */
bool Adafruit_APDS9960::colorDataReady() {
  _status.set(this->read8(APDS9960_STATUS));
  return _status.AVALID;
}

/*!
 *  @brief  Reads the raw red, green, blue and clear channel values
 *  @param  *r
 *          Red value
 *  @param  *g
 *          Green value
 *  @param  *b
 *          Blue value
 *  @param  *c
 *          Clear channel value
 */
void Adafruit_APDS9960::getColorData(uint16_t *r, uint16_t *g, uint16_t *b,
                                     uint16_t *c) {

  *c = read16R(APDS9960_CDATAL);
  *r = read16R(APDS9960_RDATAL);
  *g = read16R(APDS9960_GDATAL);
  *b = read16R(APDS9960_BDATAL);
}

/*!
 *  @brief  Converts the raw R/G/B values to color temperature in degrees Kelvin
 *  @param  r
 *          Red value
 *  @param  g
 *          Green value
 *  @param  b
 *          Blue value
 *  @return Color temperature
 */
uint16_t Adafruit_APDS9960::calculateColorTemperature(uint16_t r, uint16_t g,
                                                      uint16_t b) {
  float X, Y, Z; /* RGB to XYZ correlation      */
  float xc, yc;  /* Chromaticity co-ordinates   */
  float n;       /* McCamy's formula            */
  float cct;

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + (0.56332F * b);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
  n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
  cct =
      (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return (uint16_t)cct;
}
/*!
 *  @brief  Calculate ambient light values
 *  @param  r
 *          Red value
 *  @param  g
 *          Green value
 *  @param  b
 *          Blue value
 *  @return LUX value
 */
uint16_t Adafruit_APDS9960::calculateLux(uint16_t r, uint16_t g, uint16_t b) {
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}

/*!
 *  @brief  Enables color interrupt
 */
void Adafruit_APDS9960::enableColorInterrupt() {
  _enable.AIEN = 1;
  write8(APDS9960_ENABLE, _enable.get());
}

/*!
 *  @brief  Disables color interrupt
 */
void Adafruit_APDS9960::disableColorInterrupt() {
  _enable.AIEN = 0;
  write8(APDS9960_ENABLE, _enable.get());
}

/*!
 *  @brief  Clears interrupt
 */
void Adafruit_APDS9960::clearInterrupt() {
  this->write(APDS9960_AICLEAR, NULL, 0);
}
/*!
 *  @brief  Sets interrupt limits
 *  @param  low
 *          Low limit
 *  @param  high
 *          High limit
 */
void Adafruit_APDS9960::setIntLimits(uint16_t low, uint16_t high) {
  write8(APDS9960_AILTIL, low & 0xFF);
  write8(APDS9960_AILTH, low >> 8);
  write8(APDS9960_AIHTL, high & 0xFF);
  write8(APDS9960_AIHTH, high >> 8);
}

/*!
 *  @brief  Writes specified value to given register
 *  @param  reg
 *          Register to write to
 *  @param  value
 *          Value to write
 */
void Adafruit_APDS9960::write8(byte reg, byte value) {
  this->write(reg, &value, 1);
}

/*!
 *  @brief  Reads 8 bits from specified register
 *  @param  reg
 *          Register to write to
 *  @return Value in register
 */
uint8_t Adafruit_APDS9960::read8(byte reg) {
  uint8_t ret;
  this->read(reg, &ret, 1);

  return ret;
}

/*!
 *  @brief  Reads 32 bits from specified register
 *  @param  reg
 *          Register to write to
 *  @return Value in register
 */
uint32_t Adafruit_APDS9960::read32(uint8_t reg) {
  uint8_t ret[4];
  uint32_t ret32;
  this->read(reg, ret, 4);
  ret32 = ret[3];
  ret32 |= (uint32_t)ret[2] << 8;
  ret32 |= (uint32_t)ret[1] << 16;
  ret32 |= (uint32_t)ret[0] << 24;
  return ret32;
}

/*!
 *  @brief  Reads 16 bites from specified register
 *  @param  reg
 *          Register to write to
 *  @return Value in register
 */
uint16_t Adafruit_APDS9960::read16(uint8_t reg) {
  uint8_t ret[2];
  this->read(reg, ret, 2);

  return (ret[0] << 8) | ret[1];
}

/*!
 *  @brief  Reads 16 bites from specified register
 *  @param  reg
 *          Register to write to
 *  @return Value in register
 */
uint16_t Adafruit_APDS9960::read16R(uint8_t reg) {
  uint8_t ret[2];
  this->read(reg, ret, 2);

  return (ret[1] << 8) | ret[0];
}

/*!
 *  @brief  Reads num bytes from specified register into a given buffer
 *  @param  reg
 *          Register
 *  @param  *buf
 *          Buffer
 *  @param  num
 *          Number of bytes
 *  @return Position after reading
 */
//  uint8_t Adafruit_APDS9960::read(uint8_t reg, uint8_t *buf, uint8_t num) {
//   buf[0] = reg;
//   i2c_dev->write_then_read(buf, 1, buf, num);
//   return num;
// }
 // created by ChatGPT since no 'read then write function' in the Bela I2c class
uint8_t Adafruit_APDS9960::read(uint8_t reg, uint8_t *buf, uint8_t num) {
    // First, write the register address we want to read from
    if(i2c.writeBytes(&reg, 1) <= 0) {
        fprintf(stderr, "Failed to write to the I2C device\n");
        return 0;
    }

    // Then, read the specified number of bytes from that register into the buffer
    if(i2c.readBytes(buf, num) <= 0) {
        fprintf(stderr, "Failed to read from the I2C device\n");
        return 0;
    }

    return num;
}

/*!
 *  @brief  Writes num bytes from specified buffer into a given register
 *  @param  reg
 *          Register
 *  @param  *buf
 *          Buffer
 *  @param  num
 *          Number of bytes
 */
// void Adafruit_APDS9960::write(uint8_t reg, uint8_t *buf, uint8_t num) {
//   uint8_t prefix[1] = {reg};
//   i2c_dev->write(buf, num, true, prefix, 1);
// }
void Adafruit_APDS9960::write(uint8_t reg, uint8_t *buf, uint8_t num) {
    // First, write the register address we want to write to
    if(i2c.writeBytes(&reg, 1) <= 0) {
        fprintf(stderr, "Failed to write to the I2C device\n");
        return;
    }

    // Then, write the data
    if(i2c.writeBytes(buf, num) <= 0) {
        fprintf(stderr, "Failed to write to the I2C device\n");
    }
}


