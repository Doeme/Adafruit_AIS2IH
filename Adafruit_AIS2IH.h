/*!
 *  @file Adafruit_AIS2IH.h
 *
 *  This is a library for the Adafruit AIS2IH Accel breakout board
 *
 *  Designed specifically to work with the Adafruit AIS2IH Triple-Axis
 *Accelerometer
 *	(+-2g/4g/8g/16g)
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2809
 *
 *	This sensor communicates over I2C or SPI (our library code supports
 *both) so you can share it with a bunch of other sensors on the same I2C bus.
 *  There's an address selection pin so you can have two accelerometers share an
 *I2C bus.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  K. Townsend / Limor Fried (Ladyada) - (Adafruit Industries).
 *
 *  BSD license, all text above must be included in any redistribution
 */

#ifndef ADAFRUIT_AIS2IH_H
#define ADAFRUIT_AIS2IH_H

#include "Arduino.h"

#include <SPI.h>
#include <Wire.h>

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_Sensor.h>

/** I2C ADDRESS/BITS **/
#define AIS2IH_DEFAULT_ADDRESS (0x18) // if SDO/SA0 is 3V, its 0x19

/*!
 *  STATUS_REG_AUX register
 *   321OR  1, 2 and 3 axis data overrun. Default value: 0
 *          (0: no overrun has occurred; 1: a new set of data has overwritten
 * the previous ones) 3OR    3 axis data overrun. Default value: 0 (0: no
 * overrun has occurred; 1: a new data for the 3-axis has overwritten the
 * previous one) 2OR    2 axis data overrun. Default value: 0 (0: no overrun has
 * occurred; 1: a new data for the 4-axis has overwritten the previous one) 1OR
 * 1 axis data overrun. Default value: 0 (0: no overrun has occurred; 1: a new
 * data for the 1-axis has overwritten the previous one) 321DA  1, 2 and 3 axis
 * new data available. Default value: 0 (0: a new set of data is not yet
 * available; 1: a new set of data is available) 3DA:   3 axis new data
 * available. Default value: 0 (0: a new data for the 3-axis is not yet
 * available; 1: a new data for the 3-axis is available) 2DA:   2 axis new data
 * available. Default value: 0 (0: a new data for the 2-axis is not yet
 * available; 1: a new data for the 2-axis is available) 1DA    1 axis new data
 * available. Default value: 0 (0: a new data for the 1-axis is not yet
 * available; 1: a new data for the 1-axis is available)
 */
#define AIS2IH_REG_STATUS1 0x27
#define AIS2IH_REG_OUTADC1_L 0x28 /**< 1-axis acceleration data. Low value */
#define AIS2IH_REG_OUTADC1_H 0x29 /**< 1-axis acceleration data. High value */
#define AIS2IH_REG_OUTADC2_L 0x2A /**< 2-axis acceleration data. Low value */
#define AIS2IH_REG_OUTADC2_H 0x2B /**< 2-axis acceleration data. High value */
#define AIS2IH_REG_OUTADC3_L 0x2C /**< 3-axis acceleration data. Low value */
#define AIS2IH_REG_OUTADC3_H 0x2D /**< 3-axis acceleration data. High value */
#define AIS2IH_REG_WHOAMI                                                      \
  0x0F /**< Device identification register. [0, 0, 1, 1, 0, 0, 1, 1] */
/*!
 *  TEMP_CFG_REG
 *  Temperature configuration register.
 *   ADC_PD   ADC enable. Default value: 0
 *            (0: ADC disabled; 1: ADC enabled)
 *   TEMP_EN  Temperature sensor (T) enable. Default value: 0
 *            (0: T disabled; 1: T enabled)
 */
#define AIS2IH_REG_TEMPCFG 0x1F
/*!
 *  CTRL_REG1
 *  [ODR3, ODR2, ODR1, ODR0, LPen, Zen, Yen, Xen]
 *   ODR3-0  Data rate selection. Default value: 00
 *           (0000:50 Hz; Others: Refer to Datasheet Table 26, “Data rate
 * configuration”) LPen    Low power mode enable. Default value: 0 (0: normal
 * mode, 1: low power mode) Zen     Z axis enable. Default value: 1 (0: Z axis
 * disabled; 1: Z axis enabled) Yen     Y axis enable. Default value: 1 (0: Y
 * axis disabled; 1: Y axis enabled) Xen     X axis enable. Default value: 1 (0:
 * X axis disabled; 1: X axis enabled)
 */
#define AIS2IH_REG_CTRL1 0x20
/*!
 *  CTRL_REG2
 *  [HPM1, HPM0, HPCF2, HPCF1, FDS, HPCLICK, HPIS2, HPIS1]
 *   HPM1-0  High pass filter mode selection. Default value: 00
 *           Refer to Table 29, "High pass filter mode configuration"
 *   HPCF2-1 High pass filter cut off frequency selection
 *   FDS     Filtered data selection. Default value: 0
 *					 (0: internal filter bypassed; 1: data
 *from internal filter sent to output register and FIFO) HPCLICK High pass
 *filter enabled for CLICK function. (0: filter bypassed; 1: filter enabled)
 *HPIS2   X axis enable. Default value: 1 (0: X axis disabled; 1: X axis
 *enabled) HPIS1 High pass filter enabled for AOI function on interrupt 1, (0:
 *filter bypassed; 1: filter enabled)
 */
#define AIS2IH_REG_CTRL2 0x21
/*!
 *  CTRL_REG3
 *  [I1_CLICK, I1_AOI1, I1_AOI2, I1_DRDY1, I1_DRDY2, I1_WTM, I1_OVERRUN, --]
 *   I1_CLICK    CLICK interrupt on INT1. Default value 0.
 *						   (0: Disable; 1: Enable)
 *   I1_AOI1     AOI1 interrupt on INT1. Default value 0.
 *						   (0: Disable; 1: Enable)
 *   I1_AOI2     AOI2 interrupt on INT1. Default value 0.
 *               (0: Disable; 1: Enable)
 *   I1_DRDY1    DRDY1 interrupt on INT1. Default value 0.
 *               (0: Disable; 1: Enable)
 *   I1_DRDY2    DRDY2 interrupt on INT1. Default value 0.
 *               (0: Disable; 1: Enable)
 *   I1_WTM      FIFO Watermark interrupt on INT1. Default value 0.
 *               (0: Disable; 1: Enable)
 *   I1_OVERRUN  FIFO Overrun interrupt on INT1. Default value 0.
 * 							 (0: Disable; 1: Enable)
 */
#define AIS2IH_REG_CTRL3 0x22
/*!
 *  CTRL_REG4
 *  [BDU, BLE, FS1, FS0, HR, ST1, ST0, SIM]
 *   BDU      Block data update. Default value: 0
 *            (0: continuos update; 1: output registers not updated until MSB
 * and LSB reading) BLE      Big/little endian data selection. Default value 0.
 *            (0: Data LSB @ lower address; 1: Data MSB @ lower address)
 *   FS1-FS0  Full scale selection. default value: 00
 *            (00: +/- 2G; 01: +/- 4G; 10: +/- 8G; 11: +/- 16G)
 *   HR       High resolution output mode: Default value: 0
 *            (0: High resolution disable; 1: High resolution Enable)
 *   ST1-ST0  Self test enable. Default value: 00
 *            (00: Self test disabled; Other: See Table 34)
 *   SIM      SPI serial interface mode selection. Default value: 0
 *            (0: 4-wire interface; 1: 3-wire interface).
 */
#define AIS2IH_REG_CTRL4 0x23
/*!
 *  CTRL_REG5
 *  [BOOT, FIFO_EN, --, --, LIR_INT1, D4D_INT1, 0, 0]
 *   BOOT     Reboot memory content. Default value: 0
 *            (0: normal mode; 1: reboot memory content)
 *   FIFO_EN  FIFO enable. Default value: 0
 *            (0: FIFO disable; 1: FIFO Enable)
 *   LIR_INT1 Latch interrupt request on INT1_SRC register, with INT1_SRC
 * register cleared by reading INT1_SRC itself. Default value: 0. (0: interrupt
 * request not latched; 1: interrupt request latched) D4D_INT1 4D enable: 4D
 * detection is enabled on INT1 when 6D bit on INT1_CFG is set to 1.
 */
#define AIS2IH_REG_CTRL5 0x24

/*!
 *  CTRL_REG6
 *  [I2_CLICKen, I2_INT1, 0, BOOT_I1, 0, --, H_L, -]
 */
#define AIS2IH_REG_CTRL6 0x25
/*!
 *  STATUS_REG
 *  [ZYXOR, ZOR, YOR, XOR, ZYXDA, ZDA, YDA, XDA]
 *   ZYXOR    X, Y and Z axis data overrun. Default value: 0
 *            (0: no overrun has occurred; 1: a new set of data has overwritten
 * the previous ones) ZOR      Z axis data overrun. Default value: 0 (0: no
 * overrun has occurred; 1: a new data for the Z-axis has overwritten the
 * previous one) YOR      Y axis data overrun. Default value: 0 (0: no overrun
 * has occurred;  1: a new data for the Y-axis has overwritten the previous one)
 *   XOR      X axis data overrun. Default value: 0
 *            (0: no overrun has occurred; 1: a new data for the X-axis has
 * overwritten the previous one) ZYXDA    X, Y and Z axis new data available.
 * Default value: 0 (0: a new set of data is not yet available; 1: a new set of
 * data is available) ZDA      Z axis new data available. Default value: 0 (0: a
 * new data for the Z-axis is not yet available; 1: a new data for the Z-axis is
 * available) YDA      Y axis new data available. Default value: 0 (0: a new
 * data for the Y-axis is not yet available; 1: a new data for the Y-axis is
 * available)
 */
#define AIS2IH_REG_STATUS 0x27
#define AIS2IH_REG_OUT_X_L 0x28 /**< X-axis acceleration data. Low value */
#define AIS2IH_REG_OUT_X_H 0x29 /**< X-axis acceleration data. High value */
#define AIS2IH_REG_OUT_Y_L 0x2A /**< Y-axis acceleration data. Low value */
#define AIS2IH_REG_OUT_Y_H 0x2B /**< Y-axis acceleration data. High value */
#define AIS2IH_REG_OUT_Z_L 0x2C /**< Z-axis acceleration data. Low value */
#define AIS2IH_REG_OUT_Z_H 0x2D /**< Z-axis acceleration data. High value */
/*!
 *  FIFO_CTRL_REG
 *  [FM1, FM0, TR, FTH4, FTH3, FTH2, FTH1, FTH0]
 *   FM1-FM0  FIFO mode selection. Default value: 00 (see Table 44)
 *   TR       Trigger selection. Default value: 0
 *            0: Trigger event liked to trigger signal on INT1
 *            1: Trigger event liked to trigger signal on INT2
 *   FTH4:0   Default value: 0
 */
#define AIS2IH_REG_FIFOCTRL 0x2E
#define AIS2IH_REG_FIFOSRC                                                     \
  0x2F /**< FIFO_SRC_REG [WTM, OVRN_FIFO, EMPTY, FSS4, FSS3, FSS2, FSS1, FSS0] \
        */
#define AIS2IH_REG_ALL_INT_SRC 0x3B

#define AIS2IH_REG_TAP_THS_X 0x30 /**< 4D configuration enable and TAP threashold configuration */
#define AIS2IH_REG_TAP_THS_Y 0x31 /**< 4D configuration enable and TAP threashold configuration */
#define AIS2IH_REG_TAP_THS_Z 0x32 /**< 4D configuration enable and TAP threashold configuration */
#define AIS2IH_REG_INT_DUR 0x33 /**< 4D configuration enable and TAP threashold configuration */
#define AIS2IH_REG_TAP_SRC 0x39 /**< 4D configuration enable and TAP threashold configuration */
#define AIS2IH_REG_WAKE_UP_THS 0x34 /**< 4D configuration enable and TAP threashold configuration */
#define AIS2IH_REG_CTRL7 0x3F

/** A structure to represent scales **/
typedef enum {
  AIS2IH_RANGE_16_G = 0b11, // +/- 16g
  AIS2IH_RANGE_8_G = 0b10,  // +/- 8g
  AIS2IH_RANGE_4_G = 0b01,  // +/- 4g
  AIS2IH_RANGE_2_G = 0b00   // +/- 2g (default value)
} ais2ih_range_t;

typedef enum {
  AIS2IH_BANDWIDTH_1_20 = 0b11, // +/- 16g
  AIS2IH_BANDWIDTH_1_10 = 0b10,  // +/- 8g
  AIS2IH_BANDWIDTH_1_4 = 0b01,  // +/- 4g
  AIS2IH_BANDWIDTH_1_2 = 0b00   // +/- 2g (default value)
} ais2ih_bandwidth_t;


/** Used with register 0x2A (AIS2IH_REG_CTRL_REG1) to set bandwidth **/
typedef enum {
  AIS2IH_DATARATE_1600_HZ = 0b1001, //  400Hz
  AIS2IH_DATARATE_800_HZ = 0b1000, //  200Hz
  AIS2IH_DATARATE_400_HZ = 0b0111, //  100Hz
  AIS2IH_DATARATE_200_HZ = 0b0110,  //   50Hz
  AIS2IH_DATARATE_100_HZ = 0b0101,  //   25Hz
  AIS2IH_DATARATE_50_HZ = 0b0100,  // 10 Hz
  AIS2IH_DATARATE_25_HZ = 0b0011,   // 1 Hz
  AIS2IH_DATARATE_12_HZ_5 = 0b0010,   // 1 Hz
  AIS2IH_DATARATE_LOWPOWER_1_HZ_6 = 0b0001,
  AIS2IH_DATARATE_POWERDOWN = 0,

} ais2ih_dataRate_t;

/*!
 *  @brief  Class that stores state and functions for interacting with
 *          Adafruit_AIS2IH
 */
class Adafruit_AIS2IH : public Adafruit_Sensor {
public:
  Adafruit_AIS2IH(TwoWire *Wi = &Wire);
  Adafruit_AIS2IH(int8_t cspin, SPIClass *theSPI = &SPI);
  Adafruit_AIS2IH(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);

  bool begin(uint8_t addr = AIS2IH_DEFAULT_ADDRESS, uint8_t nWAI = 0x44);

  bool setBandwidth(ais2ih_bandwidth_t bw);
  bool tapSetup(float x,float y, float z, unsigned int latency = 0, unsigned int quiet = 0b01, unsigned int shock=0b10, bool double_tap = false);
  int getTap();

  uint8_t getDeviceID(void);
  bool haveNewData(void);
  bool enableDRDY(bool enable_drdy = true, uint8_t int_pin = 1);

  void read(void);
  int16_t readADC(uint8_t a);

  void setRange(ais2ih_range_t range);
  ais2ih_range_t getRange(void);

  void setDataRate(ais2ih_dataRate_t dataRate);
  ais2ih_dataRate_t getDataRate(void);

  bool getEvent(sensors_event_t *event);
  void getSensor(sensor_t *sensor);
  void dump();
  uint8_t readAndClearInterrupt(void);

  int16_t x; /**< x axis value */
  int16_t y; /**< y axis value */
  int16_t z; /**< z axis value */

  float x_g; /**< x_g axis value (calculated by selected range) */
  float y_g; /**< y_g axis value (calculated by selected range) */
  float z_g; /**< z_g axis value (calculated by selected range) */

private:
  TwoWire *I2Cinterface;
  SPIClass *SPIinterface;

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface

  uint8_t _wai;

  int8_t _cs, _mosi, _miso, _sck;

  int8_t _i2caddr;

  int32_t _sensorID;
  void writeReg(uint8_t reg, uint8_t val);
};

#endif
