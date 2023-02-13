/*!
 * @file Adafruit_AIS2IH.cpp
 *
 *  @mainpage Adafruit AIS2IH breakout board
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the Adafruit AIS2IH Accel breakout board
 *
 *  Designed specifically to work with the Adafruit AIS2IH Accel breakout board.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2809
 *
 *  This sensor communicates over I2C or SPI (our library code supports both) so
 * you can share it with a bunch of other sensors on the same I2C bus.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  K. Townsend / Limor Fried (Adafruit Industries)
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include "Arduino.h"

#include <Adafruit_AIS2IH.h>
#include <Wire.h>

/*!
 *  @brief  Instantiates a new AIS2IH class in I2C
 *  @param  Wi
 *          optional wire object
 */
Adafruit_AIS2IH::Adafruit_AIS2IH(TwoWire *Wi)
    : _cs(-1), _mosi(-1), _miso(-1), _sck(-1), _sensorID(-1) {
  I2Cinterface = Wi;
}

/*!
 *   @brief  Instantiates a new AIS2IH class using hardware SPI
 *   @param  cspin
 *           number of CSPIN (Chip Select)
 *   @param  *theSPI
 *           optional parameter contains spi object
 */
Adafruit_AIS2IH::Adafruit_AIS2IH(int8_t cspin, SPIClass *theSPI) {
  _cs = cspin;
  _mosi = -1;
  _miso = -1;
  _sck = -1;
  _sensorID = -1;
  SPIinterface = theSPI;
}

/*!
 *   @brief  Instantiates a new AIS2IH class using software SPI
 *   @param  cspin
 *           number of CSPIN (Chip Select)
 *   @param  mosipin
 *           number of pin used for MOSI (Master Out Slave In))
 *   @param  misopin
 *           number of pin used for MISO (Master In Slave Out)
 *   @param  sckpin
 *           number of pin used for CLK (clock pin)
 */
Adafruit_AIS2IH::Adafruit_AIS2IH(int8_t cspin, int8_t mosipin, int8_t misopin,
                                 int8_t sckpin) {
  _cs = cspin;
  _mosi = mosipin;
  _miso = misopin;
  _sck = sckpin;
  _sensorID = -1;
}

/*!
 *  @brief  Setups the HW (reads coefficients values, etc.)
 *  @param  i2caddr
 *          i2c address (optional, fallback to default)
 *  @param  nWAI
 *          Who Am I register value - defaults to 0x33 (AIS2IH)
 *  @return true if successful
 */
bool Adafruit_AIS2IH::begin(uint8_t i2caddr, uint8_t nWAI) {
  _i2caddr = i2caddr;
  _wai = nWAI;
  if (I2Cinterface) {
    i2c_dev = new Adafruit_I2CDevice(_i2caddr, I2Cinterface);

    if (!i2c_dev->begin()) {
      return false;
    }
  } else if (_cs != -1) {

    // SPIinterface->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    if (_sck == -1) {
      spi_dev = new Adafruit_SPIDevice(_cs,
                                       500000,                // frequency
                                       SPI_BITORDER_MSBFIRST, // bit order
                                       SPI_MODE0,             // data mode
                                       SPIinterface);
    } else {
      spi_dev = new Adafruit_SPIDevice(_cs, _sck, _miso, _mosi,
                                       500000,                // frequency
                                       SPI_BITORDER_MSBFIRST, // bit order
                                       SPI_MODE0);            // data mode
    }

    if (!spi_dev->begin()) {
      return false;
    }
  }

  /* Check connection */
  if (getDeviceID() != _wai) {
    /* No AIS2IH detected ... return false */
    // Serial.println(deviceid, HEX);
    return false;
  }
  Adafruit_BusIO_Register _ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_CTRL1, 1);
  _ctrl1.write(0b10010100); // enable all axes, normal mode

  // 400Hz rate
  setDataRate(AIS2IH_DATARATE_1600_HZ);

  Adafruit_BusIO_Register _ctrl2 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_CTRL2, 1);
  _ctrl2.write(0b00011100); // BDU enabled & auto-increment

  Adafruit_BusIO_Register _ctrl3 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_CTRL3, 1);
  _ctrl3.write(0b00011000); 

  Adafruit_BusIO_Register _ctrl6 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_CTRL6, 1);
  _ctrl6.write(0b00000100); // ODR/2 & +-2g & Low-pass & Low-Noise

  Adafruit_BusIO_Register _ctrl7 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_CTRL7, 1);
  _ctrl7.write(0b00100000); // Interrupt enable

  Adafruit_BusIO_Register _fifo = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_FIFOCTRL, 1);
  _fifo.write(0b00000000); // FIFO off & FIFO threshold 0

  enableDRDY(true, 1);

  return true;
}

/*!
 *  @brief  Get Device ID from AIS2IH_REG_WHOAMI
 *  @return WHO AM I value
 */
uint8_t Adafruit_AIS2IH::getDeviceID(void) {
  Adafruit_BusIO_Register _chip_id = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_WHOAMI, 1);

  return _chip_id.read();
}
/*!
 *  @brief  Check to see if new data available
 *  @return true if there is new data available, false otherwise
 */
bool Adafruit_AIS2IH::haveNewData(void) {
  Adafruit_BusIO_Register status_2 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_STATUS, 1);
  Adafruit_BusIO_RegisterBits zyx_data_available =
      Adafruit_BusIO_RegisterBits(&status_2, 1, 0);
  return zyx_data_available.read();
}

/*!
 *  @brief  Check to see if new data available
 *  @return true if there is new data available, false otherwise
 */
bool Adafruit_AIS2IH::setBandwidth(ais2ih_bandwidth_t bw) {
  Adafruit_BusIO_Register ctrl6 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_CTRL6, 1);
      
  Adafruit_BusIO_RegisterBits bw_bits =
      Adafruit_BusIO_RegisterBits(&ctrl6, 2, 6);
  return bw_bits.write(bw);
}

void Adafruit_AIS2IH::writeReg(uint8_t reg, uint8_t val) {
  Adafruit_BusIO_Register r = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, reg, 1);
  r.write(val);
}
/*!
 *  @brief  Check to see if new data available
 *  @return true if there is new data available, false otherwise
 */
int Adafruit_AIS2IH::getTap() {
  Adafruit_BusIO_Register tap_src = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_TAP_SRC, 1);
  
  uint8_t v = tap_src.read();
  if((v>>5)&1){
    return 1;
  }
  if((v>>4)&1){
    return 2;
  }
  return 0;
}

static uint8_t convert(float v){
  if(isnan(v)){
    return 0;
  }
  else{
    if(v<0){
      v=-v;
    }
    if(v>2){
      v=2;
    }
    return v/2*((2<<4)-1);
  }
}
/*!
 *  @brief  Check to see if new data available
 *  @return true if there is new data available, false otherwise
 */
bool Adafruit_AIS2IH::tapSetup(float x, float y, float z, unsigned int latency, unsigned int quiet, unsigned int shock, bool double_tap) {
  if(latency >= (2<<2)){
    latency = (2<<2)-1;
  }
  if(quiet >= (2<<1)){
    quiet = (2<<1)-1;
  }
  if(shock >= (2<<1)){
    shock = (2<<1)-1;
  }
  uint8_t ux = convert(x);
  uint8_t uy = convert(y);
  uint8_t uz = convert(z);
  Adafruit_BusIO_Register ths_x = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_TAP_THS_X, 1);
  Adafruit_BusIO_Register ths_y = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_TAP_THS_Y, 1);
  Adafruit_BusIO_Register ths_z = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_TAP_THS_Z, 1);
  Adafruit_BusIO_RegisterBits ths_x_bits =
      Adafruit_BusIO_RegisterBits(&ths_x, 4, 0);
  
  ths_x_bits.write(ux);
  ths_y.write(uy);
  ths_z.write(
    (!isnan(x))<<7 | 
    (!isnan(y))<<6 | 
    (!isnan(z))<<5 |
    uz
  );
  
  {
  Adafruit_BusIO_Register int_dur = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_INT_DUR, 1);
  int_dur.write((latency<<4) | (quiet<<2) | shock);
  }

  {
  Adafruit_BusIO_Register wake_up = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_WAKE_UP_THS, 1);
  Adafruit_BusIO_RegisterBits wake_up_bits =
      Adafruit_BusIO_RegisterBits(&wake_up, 1, 7);
  wake_up_bits.write(double_tap);
  }

  return true;
}

void Adafruit_AIS2IH::dump() {
  static constexpr unsigned int offset = 0x30;
  static constexpr unsigned int lastreg = 0x3F;
  static constexpr unsigned int len = lastreg-offset+1;
  uint8_t abuf[1] = {offset};
  uint8_t buffer[len];
  if(!i2c_dev->write_then_read(abuf, 1, buffer, len)){
    Serial.println("Readf ailed");
  }

  for(unsigned int i=0; i<len; i++){
    Serial.print("0x");
    Serial.print(i+offset,HEX);
    Serial.print(": ");
    Serial.println(buffer[i],BIN);
  }
}

/*!
 *  @brief  Reads x y z values at once
 */
void Adafruit_AIS2IH::read(void) {

  uint8_t register_address = AIS2IH_REG_OUT_X_L;

  Adafruit_BusIO_Register xl_data = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, register_address, 6);

  uint8_t buffer[6];
  xl_data.read(buffer, 6);

  x = buffer[0];
  x |= ((uint16_t)buffer[1]) << 8;
  y = buffer[2];
  y |= ((uint16_t)buffer[3]) << 8;
  z = buffer[4];
  z |= ((uint16_t)buffer[5]) << 8;

  uint8_t range = getRange();

  // this scaling process accounts for the shift due to actually being 10 bits
  // (normal mode) as well as the lsb=> mg conversion and the mg=> g conversion
  // final value is raw_lsb => 10-bit lsb -> milli-gs -> gs

  // regardless of the range, we'll always convert the value to 10 bits and g's
  // so we'll always divide by AIS2IH_LSB16_TO_KILO_LSB10 (16000):

  // then we can then multiply the resulting value by the lsb value to get the
  // value in g's

  float lsb_value = 1.0f;
  if (range == AIS2IH_RANGE_2_G)
    lsb_value = 2.0f;
  if (range == AIS2IH_RANGE_4_G)
    lsb_value = 4.0f;
  if (range == AIS2IH_RANGE_8_G)
    lsb_value = 8.0f;
  if (range == AIS2IH_RANGE_16_G)
    lsb_value = 32.0f;
  x_g = lsb_value * ((float)x / (2<<14));
  y_g = lsb_value * ((float)y / (2<<14));
  z_g = lsb_value * ((float)z / (2<<14));
}

/*!
 *   @brief  Get uint8_t for INT1 source and clear interrupt
 *   @return register AIS2IH_REG_INT1SRC
 */
uint8_t Adafruit_AIS2IH::readAndClearInterrupt(void) {
  Adafruit_BusIO_Register int_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_ALL_INT_SRC, 1);

  return int_reg.read();
}

/**
 * @brief Enable or disable the Data Ready interupt
 *
 * @param enable_drdy true to enable the given Data Ready interrupt on INT1,
 * false to disable it
 * @param int_pin which DRDY interrupt to enable; 1 for DRDY1, 2 for DRDY2
 * @return true: success false: failure
 */
bool Adafruit_AIS2IH::enableDRDY(bool enable_drdy, uint8_t int_pin) {
  int address;
  switch(int_pin){
    case 1:
      address = AIS2IH_REG_CTRL4;
    break;
    case 2:
      address = AIS2IH_REG_CTRL5;
    break;
    default:
      return false;
    break;
  }
  Adafruit_BusIO_Register _ctrl = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, address, 1);
  Adafruit_BusIO_RegisterBits _drdy1_int_enable =
      Adafruit_BusIO_RegisterBits(&_ctrl, 1, 0);

  return _drdy1_int_enable.write(enable_drdy);
}

/*!
 *   @brief  Sets the g range for the accelerometer
 *   @param  range
 *           range value
 */
void Adafruit_AIS2IH::setRange(ais2ih_range_t range) {

  Adafruit_BusIO_Register _ctrl4 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_CTRL4, 1);

  Adafruit_BusIO_RegisterBits range_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl4, 2, 4);
  range_bits.write(range);
  delay(15); // delay to let new setting settle
}

/*!
 *  @brief  Gets the g range for the accelerometer
 *  @return Returns g range value
 */
ais2ih_range_t Adafruit_AIS2IH::getRange(void) {
  Adafruit_BusIO_Register _ctrl6 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_CTRL6, 1);

  Adafruit_BusIO_RegisterBits range_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl6, 2, 4);
  return (ais2ih_range_t)range_bits.read();
}

/*!
 *  @brief  Sets the data rate for the AIS2IH (controls power consumption)
 *  @param  dataRate
 *          data rate value
 */
void Adafruit_AIS2IH::setDataRate(ais2ih_dataRate_t dataRate) {
  Adafruit_BusIO_Register _ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_CTRL1, 1);
  Adafruit_BusIO_RegisterBits data_rate_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl1, 4, 4);

  data_rate_bits.write(dataRate);
}

/*!
 *   @brief  Gets the data rate for the AIS2IH (controls power consumption)
 *   @return Returns Data Rate value
 */
ais2ih_dataRate_t Adafruit_AIS2IH::getDataRate(void) {
  Adafruit_BusIO_Register _ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AIS2IH_REG_CTRL1, 1);
  Adafruit_BusIO_RegisterBits data_rate_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl1, 4, 4);

  return (ais2ih_dataRate_t)data_rate_bits.read();
}

/*!
 *  @brief  Gets the most recent sensor event
 *  @param  *event
 *          sensor event that we want to read
 *  @return true if successful
 */
bool Adafruit_AIS2IH::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;

  read();

  event->acceleration.x = x_g * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = y_g * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = z_g * SENSORS_GRAVITY_STANDARD;

  return true;
}

/*!
 *   @brief  Gets the sensor_t data
 *   @param  *sensor
 *           sensor that we want to write data into
 */
void Adafruit_AIS2IH::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "AIS2IH", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay = 0;
  sensor->max_value = 0;
  sensor->min_value = 0;
  sensor->resolution = 0;
}
