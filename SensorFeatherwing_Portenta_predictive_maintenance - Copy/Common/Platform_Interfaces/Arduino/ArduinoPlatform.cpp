/**
 * \file
 * \brief Aduino platform drivers for Adafruit M0 feather.
 *
 * This code is abstraction of arduino peripheral drivers for Adafruit feather
 * MO board.
 *
 * \copyright (c) 2020 Würth Elektronik eiSos GmbH & Co. KG
 *
 * \page License
 *
 * THE SOFTWARE INCLUDING THE SOURCE CODE IS PROVIDED “AS IS”. YOU ACKNOWLEDGE
 * THAT WÜRTH ELEKTRONIK EISOS MAKES NO REPRESENTATIONS AND WARRANTIES OF ANY
 * KIND RELATED TO, BUT NOT LIMITED TO THE NON-INFRINGEMENT OF THIRD PARTIES’
 * INTELLECTUAL PROPERTY RIGHTS OR THE MERCHANTABILITY OR FITNESS FOR YOUR
 * INTENDED PURPOSE OR USAGE. WÜRTH ELEKTRONIK EISOS DOES NOT WARRANT OR
 * REPRESENT THAT ANY LICENSE, EITHER EXPRESS OR IMPLIED, IS GRANTED UNDER ANY
 * PATENT RIGHT, COPYRIGHT, MASK WORK RIGHT, OR OTHER INTELLECTUAL PROPERTY
 * RIGHT RELATING TO ANY COMBINATION, MACHINE, OR PROCESS IN WHICH THE PRODUCT
 * IS USED. INFORMATION PUBLISHED BY WÜRTH ELEKTRONIK EISOS REGARDING
 * THIRD-PARTY PRODUCTS OR SERVICES DOES NOT CONSTITUTE A LICENSE FROM WÜRTH
 * ELEKTRONIK EISOS TO USE SUCH PRODUCTS OR SERVICES OR A WARRANTY OR
 * ENDORSEMENT THEREOF
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS PACKAGE
 */
#include <stdlib.h>
#include <stdarg.h> // for printf
#include <Wire.h>
#include "ArduinoPlatform.h"


#define TIMEOUT 1000
int deviceAddress = 0; // device Address

/**
 * @brief  Software reset for the MCU
 * @retval none
 */
void soft_reset()
{
  NVIC_SystemReset();
}

/**
 * @brief  Create a serial port object for handling strings and allocate memory
 * @param  ser Pointer to serial object
 * @retval Created serial port
 */
TypeSerial *SSerial_create(void *ser)
{
  TypeSerial *m;

  m = (typeof(m))malloc(sizeof(*m));
  m->obj = ser;

  return m;
}

/**
 * @brief  Free memory allocated to serial port
 * @param  m Pointer to serial object
 * @retval none
 */
void SSerial_destroy(TypeSerial *m)
{
  /* We do not create any instance of HardwareSerial with new, so we don't need to
   * delete. In addition HardwareSerial has no callable destructor
   * Only needs to clean up pointer m.*/

  if (m == NULL)
  {
    return;
  }

  free(m);
}

/**
 * @brief  Serial write byte
 * @param  m Pointer to serial object
 * @param  byte Byte to be written
 * @retval Return 1 if the byte is successfully written
 */
size_t SSerial_write(TypeSerial *m, uint8_t byte)
{
  HardwareSerial *obj;

  if (m == NULL)
  {
    return 0;
  }

  obj = static_cast<HardwareSerial *>(m->obj);
  return obj->write(byte);
}

/**
 * @brief  Serial write an array of chars
 * @param  m Pointer to serial object
 * @param  buffer Bytes to be written
 * @param  size Number of bytes to write
 * @retval Return the number of bytes successfully written
 */
size_t SSerial_writeB(TypeSerial *m, const char *buffer, size_t size)
{
  HardwareSerial *obj;

  if (m == NULL)
  {
    return 0;
  }

  obj = static_cast<HardwareSerial *>(m->obj);
  return obj->write(buffer, size);
}

/**
 * @brief  Serial begin
 * @param  m Pointer to serial object
 * @param  baud_count Baud rate
 * @retval none
 */
void SSerial_begin(TypeSerial *m, uint32_t baud_count)
{
  HardwareSerial *obj;

  if (m == NULL)
  {
    return;
  }

  obj = static_cast<HardwareSerial *>(m->obj);
  obj->begin(baud_count);
}

/**
 * @brief  Serial begin
 * @param  m Pointer to serial object
 * @param  baud_count Baud rate
 * @param  parameter Parameters parity, flowcontrol etc
 * @retval none
 */
void SSerial_beginP(TypeSerial *m, uint32_t baud_count, uint8_t parameter)
{
  HardwareSerial *obj;

  if (m == NULL)
  {
    return;
  }

  obj = static_cast<HardwareSerial *>(m->obj);
  obj->begin(baud_count, parameter);
}

/**
 * @brief  Serial check availability
 * @param  m Pointer to serial object
 * @retval Number of bytes available
 */
int SSerial_available(TypeSerial *m)
{
  HardwareSerial *obj;

  if (m == NULL)
    return 0;

  obj = static_cast<HardwareSerial *>(m->obj);
  return obj->available();
}

/**
 * @brief  Serial flush
 * @param  m Pointer to serial object
 * @retval none
 */
void SSerial_flush(TypeSerial *m)
{
  HardwareSerial *obj;

  if (m == NULL)
  {
    return;
  }

  obj = static_cast<HardwareSerial *>(m->obj);
  obj->flush();
}

/**
 * @brief  Serial printf
 * @param  m Pointer to serial object
 * @param  format Formatted string with optional arguments
 * @retval none
 */
void SSerial_printf(TypeSerial *m, const char format[], ...)
{
  HardwareSerial *obj;
  char buf[MAX_PRINT_LEN];
  va_list ap;

  if (m == NULL)
    return;

  obj = static_cast<HardwareSerial *>(m->obj);

  va_start(ap, format);
  vsnprintf(buf, sizeof(buf), format, ap);
  obj->write(buf);
  va_end(ap);
}

/**
 * @brief  Serial read
 * @param  m Pointer to serial object
 * @retval none
 */
int SSerial_read(TypeSerial *m)
{
  HardwareSerial *obj;

  if (m == NULL)
    return 0;

  obj = static_cast<HardwareSerial *>(m->obj);
  return obj->read();
}

/**
 * @brief  Create a serial port object for handling bytes and allocate memory
 * @param  ser Pointer to serial object
 * @retval Created serial port
 */
TypeHardwareSerial *HSerial_create(void *ser)
{
  TypeHardwareSerial *m;

  m = (typeof(m))malloc(sizeof(*m));
  m->obj = ser;

  return m;
}

/**
 * @brief  Free memory allocated to serial port
 * @param  m Pointer to serial object
 * @retval none
 */
void HSerial_destroy(TypeHardwareSerial *m)
{
  /* We do not create any instance of HardwareSerial with new, so we don't
   * need to delete. In addition HardwareSerial has no callable destructor
   * Only needs to clean up pointer m.*/

  if (m == NULL)
  {
    return;
  }

  free(m);
}

/**
 * @brief  Serial write byte
 * @param  m Pointer to serial object
 * @param  byte Byte to be written
 * @retval Return 1 if the byte is successfully written
 */
size_t HSerial_write(TypeHardwareSerial *m, uint8_t byte)
{
  HardwareSerial *obj;

  if (m == NULL)
  {
    return 0;
  }

  obj = static_cast<HardwareSerial *>(m->obj);
  return obj->write(byte);
}

/**
 * @brief  Serial write an array of chars
 * @param  m Pointer to serial object
 * @param  buffer Bytes to be written
 * @param  size Number of bytes to write
 * @retval Return the number of bytes successfully written
 */
size_t HSerial_writeB(TypeHardwareSerial *m, const char *buffer, size_t size)
{
  HardwareSerial *obj;

  if (m == NULL)
  {
    return 0;
  }

  obj = static_cast<HardwareSerial *>(m->obj);
  return obj->write(buffer, size);
}

/**
 * @brief  Serial begin
 * @param  m Pointer to serial object
 * @param  baud_count Baud rate
 * @retval none
 */
void HSerial_begin(TypeHardwareSerial *m, uint32_t baud_count)
{
  HardwareSerial *obj;

  if (m == NULL)
  {
    return;
  }

  obj = static_cast<HardwareSerial *>(m->obj);
  obj->begin(baud_count);
}

/**
 * @brief  Serial begin
 * @param  m Pointer to serial object
 * @param  baud_count Baud rate
 * @param  parameter Parameters parity, flowcontrol etc
 * @retval none
 */
void HSerial_beginP(TypeHardwareSerial *m, uint32_t baud_count,
                    uint8_t parameter)
{
  HardwareSerial *obj;

  if (m == NULL)
  {
    return;
  }

  obj = static_cast<HardwareSerial *>(m->obj);
  obj->begin(baud_count, parameter);
}

/**
 * @brief  Serial end
 * @param  m Pointer to serial object
 * @retval none
 */
void HSerial_end(TypeHardwareSerial *m)
{
  HardwareSerial *obj;

  if (m == NULL)
  {
    return;
  }

  obj = static_cast<HardwareSerial *>(m->obj);
  obj->end();
}

/**
 * @brief  Serial check availability
 * @param  m Pointer to serial object
 * @retval Number of bytes available
 */
int HSerial_available(TypeHardwareSerial *m)
{
  HardwareSerial *obj;

  if (m == NULL)
    return 0;

  obj = static_cast<HardwareSerial *>(m->obj);
  return obj->available();
}

/**
 * @brief  Serial check availability for write
 * @param  m Pointer to serial object
 * @retval Number of bytes available
 */
int HSerial_availableForWrite(TypeHardwareSerial *m)
{
  HardwareSerial *obj;

  if (m == NULL)
    return 0;

  obj = static_cast<HardwareSerial *>(m->obj);
  return obj->availableForWrite();
}

/**
 * @brief  Serial flush
 * @param  m Pointer to serial object
 * @retval none
 */
void HSerial_flush(TypeHardwareSerial *m)
{
  HardwareSerial *obj;

  if (m == NULL)
  {
    return;
  }

  obj = static_cast<HardwareSerial *>(m->obj);
  obj->flush();
}

/**
 * @brief  Serial read
 * @param  m Pointer to serial object
 * @retval none
 */
int HSerial_read(TypeHardwareSerial *m)
{
  HardwareSerial *obj;

  if (m == NULL)
    return 0;

  obj = static_cast<HardwareSerial *>(m->obj);
  return obj->read();
}

/**
 * @brief  Initialize the I2C Interface
 * @param  I2C address
 * @retval Error Code
 */
int8_t I2CInit(int address)
{
  deviceAddress = address;
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED_FAST);
  return WE_SUCCESS;
}

/**
 * @brief  Set I2C clock
 * @param  clock values accepted Standard - 100000, Fast - 400000
 * @retval Error Code
 */
void I2CSetClock(uint32_t baudrate) { Wire.setClock(baudrate); }
/**
 * @brief  Set I2C bus Address
 * @param  I2C address
 * @retval None
 */
void I2CSetAddress(int address) { deviceAddress = address; }

/**
 * @brief  Send data over I2C bus
 * @param  data : data to send
 *         datalen : data length
 * @retval Error Code
 */
int8_t I2CSend(uint8_t *data, int datalen)
{
  Wire.beginTransmission(deviceAddress);

  if (datalen != 0)
  {
    Wire.write(data, datalen);
  }
  if (Wire.endTransmission() != 0)
  {
    return WE_FAIL;
  }
  return WE_SUCCESS;
}
/**
 * @brief  Receive data over I2C bus
 * @param  data : data received
 *         datalen : data length
 * @retval Error Code
 */
int8_t I2CReceive(uint8_t *data, int datalen)
{
  int bytesAvailable =
      Wire.requestFrom(deviceAddress, datalen); // request Bytes
  if (bytesAvailable == 0)
  {
    return WE_FAIL;
  }
  for (int index = 0; index < bytesAvailable; index++)
  {
    data[index] = Wire.read();
  }
  return WE_SUCCESS;
}

/**
 * @brief   Read data starting from the addressed register
 * @param  -RegAdr : the register addresse to read from
 *         -NumByteToRead : number of bytes to read
 *         -pointer Data : the address store the data
 * @retval Error Code
 */
int8_t ReadReg(uint8_t RegAdr, int NumByteToRead, uint8_t *Data)
{
  Wire.beginTransmission(deviceAddress); // set sensor target
  Wire.write(RegAdr);                    // set memory pointer
  unsigned long start = millis();
  while (Wire.endTransmission() && millis() - start <= TIMEOUT)
    ;
  int n = Wire.requestFrom(deviceAddress, NumByteToRead); // request Bytes
  for (int i = 0; i < n; i++)
  {
    Data[i] = Wire.read();
  }

  return WE_SUCCESS;
}

/**
 * @brief  Write data strarting from the addressed register
 * @param  -RegAdr : Address to write in
 *         -NumByteToWrite : number of bytes to write
 *         -pointer Data : Address of the data to be written
 * @retval Error Code
 */
int8_t WriteReg(int RegAdr, int NumByteToWrite, uint8_t *Data)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(RegAdr);
  Wire.write(*Data);
  if (Wire.endTransmission())
  {
    return WE_FAIL;
  }

  return WE_SUCCESS;
}

/**         EOF         */