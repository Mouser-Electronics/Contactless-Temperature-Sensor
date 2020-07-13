/*
 * TSD305.c
 *
 *  Created on: May 13, 2020
 *      Author: djhoo
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "TSD305.h"
#include "i2cspm.h"

// TSD305 device address
//#define TSD305_ADDR 0x1e // 0b0011110
#define TSD305_ADDR 0x0

// TSD305 device commands
#define TSD305_CONVERT_ADCS_COMMAND 0xaf

#define TSD305_STATUS_BUSY_MASK 0x20
#define TSD305_STATUS_MEMOTY_ERROR_MASK 0x04

#define TSD305_CONVERSION_TIME 100


typedef union mut {
   uint32_t raw;
   float floatVal;
} ieee754Union;

enum status_code {
  STATUS_OK = 0x00,
  STATUS_ERR_OVERFLOW = 0x01,
  STATUS_ERR_TIMEOUT = 0x02,
};

static I2CSPM_Init_TypeDef TSD305I2cInit = {
  I2C0,                     // Use I2C instance 2
  gpioPortD,                // SCL port
  6,                        // SCL pin
  gpioPortD,                // SDA port
  7,                        // SDA pin
  1,                        // Location of SCL
  1,                        // Location of SDA
  0,                        // Use currently configured reference clock
  I2C_FREQ_STANDARD_MAX,    // Set to standard rate
  i2cClockHLRStandard,      // Set to use 4:4 low/high duty cycle
};

volatile uint32_t msTicks; /* counts 1ms timeTicks */
bool tsd305_coeff_read;

void Delay(uint32_t dlyTicks)
{
   uint32_t start = millis();

   while (start + dlyTicks > millis()); // Wait until the delay is complete, blocking

}


/**
 * \brief Perform initial configuration. Has to be called once.
 */
void TSD305I2CBegin(void) {
   I2CSPM_Init(&TSD305I2cInit);
   tsd305_coeff_read = false;
   Delay(1000);
}


/**
 * \brief Reads the tsd305 EEPROM coefficient stored at address provided.
 *
 * \param[in] uint8_t : Address of coefficient in EEPROM
 * \param[out] uint16_t* : Value read in EEPROM
 *
 * \return tsd305_status : status of TSD305
 *       - tsd305_status_ok : I2C transfer completed successfully
 *       - tsd305_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsd305_status_busy : Sensor is busy
 *       - tsd305_status_memory_error : Sensor EEPROM memory error
 */
tsd305_status read_eeprom_coeff(uint8_t address, uint16_t *coeff) {
  uint8_t status_byte = 0;
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[3];
  uint8_t                    i2c_write_data[1];

  seq.addr  = (uint8_t)TSD305_ADDR;
  seq.flags = I2C_FLAG_WRITE;
  /* Select command to issue */
  i2c_write_data[0] = address;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CSPM_Transfer(TSD305I2cInit.port, &seq);

  if (ret != i2cTransferDone) {
    return(tsd305_status_i2c_transfer_error);
  }

  Delay (10);

  seq.addr  = (uint8_t)TSD305_ADDR;
  seq.flags = I2C_FLAG_READ;
  /* Select command to issue */
  i2c_write_data[0] = address;
  seq.buf[0].data   = i2c_read_data;
  seq.buf[0].len    = 3;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 3;

  ret = I2CSPM_Transfer(TSD305I2cInit.port, &seq);

  if (ret != i2cTransferDone) {
    return(tsd305_status_i2c_transfer_error);
  }

  status_byte = i2c_read_data[0];
  printf ("Write %02x\nRead: %02x %02x %02x\n", address, i2c_read_data[0], i2c_read_data[1], i2c_read_data[2]);

  *coeff = (i2c_read_data[1] << 8) | i2c_read_data[2];

  return tsd305_status_ok;

  if (status_byte & TSD305_STATUS_BUSY_MASK)
    return tsd305_status_busy;
  if (status_byte & TSD305_STATUS_MEMOTY_ERROR_MASK)
    return tsd305_status_memory_error;



  //Delay (1);

  return tsd305_status_ok;
}

/**
 * \brief Reads the tsd305 EEPROM coefficient stored at address provided.
 *
 * \param[in] uint8_t : Address of coefficient in EEPROM
 * \param[out] float* : IEEE-745 Value read in EEPROM
 *
 * \return tsd305_status : status of TSD305
 *       - tsd305_status_ok : I2C transfer completed successfully
 *       - tsd305_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsd305_status_busy : Sensor is busy
 *       - tsd305_status_memory_error : Sensor EEPROM memory error
 */
tsd305_status read_eeprom_float(uint8_t address, float *value) {
  uint16_t h_word = 0;
  uint16_t l_word = 0;
  tsd305_status status = tsd305_status_ok;
  ieee754Union myUnion;

  //printf ("Read %d and %d\n", address, address+1);

  status = read_eeprom_coeff(address, &h_word);
  if (status != tsd305_status_ok) return status;

  Delay(20);

  status = read_eeprom_coeff(address + 1, &l_word);
  if (status != tsd305_status_ok) return status;

  uint32_t finalNum = h_word << 16 | l_word;
  myUnion.raw = finalNum;

  *value = myUnion.floatVal;
  //printf ("hex is 0x%8x: %f\n", myUnion.raw,myUnion.floatVal );
  //printf ("ref is 0x%8x: %f\n", myUnion2.raw,myUnion2.floatVal );
  //printf ("Float is %f\n", *value);

  Delay(20);

  return tsd305_status_ok;
}

/**
 * \brief Reads the tsd305 EEPROM coefficients to store them for computation.
 *
 * \return tsd305_status : status of TSD305
 *       - tsd305_status_ok : I2C transfer completed successfully
 *       - tsd305_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsd305_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsd305_status_busy : Sensor is busy
 *       - tsd305_status_memory_error : Sensor EEPROM memory error
 */
tsd305_status read_eeprom(void) {
  tsd305_status status = tsd305_status_ok;

  status = read_eeprom_float(0x1E, &coeff.tempCoeff);
  printf ("tempCoeff %f status %d\n", coeff.tempCoeff, status);
  if (status != tsd305_status_ok) return status;

  status = read_eeprom_float(0x20, &coeff.refTemp);
  printf ("refTemp %f status %d\n", coeff.refTemp, status);
  if (status != tsd305_status_ok) return status;

  status = read_eeprom_float(0x22, &coeff.compK4);
  printf ("compK4 %f status %d\n", coeff.compK4, status);
  if (status != tsd305_status_ok) return status;

  status = read_eeprom_float(0x24, &coeff.compK3);
  printf ("compK3 %f status %d\n", coeff.compK3, status);
  if (status != tsd305_status_ok) return status;

  status = read_eeprom_float(0x26, &coeff.compK2);
  printf ("compK2 %f status %d\n", coeff.compK2, status);
  if (status != tsd305_status_ok) return status;

  status = read_eeprom_float(0x28, &coeff.compK1);
  printf ("compK1 %f status %d\n", coeff.compK1, status);
  if (status != tsd305_status_ok) return status;

  status = read_eeprom_float(0x2A, &coeff.compK0);
  printf ("compK0 %f status %d\n", coeff.compK0, status);
  if (status != tsd305_status_ok) return status;

  status = read_eeprom_float(0x2E, &coeff.adcK4);
  printf ("adcK4 %f status %d\n", coeff.adcK4, status);
  if (status != tsd305_status_ok) return status;

  status = read_eeprom_float(0x30, &coeff.adcK3);
  printf ("adcK3 %f status %d\n", coeff.adcK3, status);
  if (status != tsd305_status_ok) return status;

  status = read_eeprom_float(0x32, &coeff.adcK2);
  printf ("adcK2 %f status %d\n", coeff.adcK2, status);
  if (status != tsd305_status_ok) return status;

  status = read_eeprom_float(0x34, &coeff.adcK1);
  printf ("adcK1 %f status %d\n", coeff.adcK1, status);
  if (status != tsd305_status_ok) return status;

  status = read_eeprom_float(0x36, &coeff.adcK0);
  printf ("adcK0 %f status %d\n", coeff.adcK0, status);
  if (status != tsd305_status_ok) return status;


  tsd305_coeff_read = true;

  return status;
}



/**
 * \brief Triggers conversion and read ADC value
 *
 * \param[in] uint8_t : Command used for conversion (will determine Temperature
 * vs Pressure and osr)
 * \param[out] uint32_t* : ADC value.
 *
 * \return tsd305_status : status of TSD305
 *       - tsd305_status_ok : I2C transfer completed successfully
 *       - tsd305_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsd305_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsd305_status_busy : Sensor is busy
 *       - tsd305_status_memory_error : Sensor EEPROM memory error
 */
tsd305_status conversion_and_read_adcs(uint32_t *adc_object,
                                                    uint32_t *adc_ambient) {
  uint8_t status_byte = 0;
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[7];
  uint8_t                    i2c_write_data[1];

  seq.addr  = (uint8_t)TSD305_ADDR;
  seq.flags = I2C_FLAG_WRITE;
  /* Select command to issue */
  i2c_write_data[0] = (uint8_t)TSD305_CONVERT_ADCS_COMMAND;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  // Send the conversion command
  ret = I2CSPM_Transfer(TSD305I2cInit.port, &seq);

  if (ret != i2cTransferDone) {
    return(tsd305_status_i2c_transfer_error);
  }

  Delay (200);

  seq.addr  = (uint8_t)TSD305_ADDR;
  seq.flags = I2C_FLAG_READ;
  /* Select command to issue */
  i2c_write_data[0] = (uint8_t)TSD305_CONVERT_ADCS_COMMAND;
  seq.buf[0].data   = i2c_read_data;
  seq.buf[0].len    = 7;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 7;

  ret = I2CSPM_Transfer(TSD305I2cInit.port, &seq);

  if (ret != i2cTransferDone) {
    return(tsd305_status_i2c_transfer_error);
  }

  status_byte = i2c_read_data[0];
  if (status_byte & TSD305_STATUS_BUSY_MASK)
    return tsd305_status_busy;
  if (status_byte & TSD305_STATUS_MEMOTY_ERROR_MASK)
    return tsd305_status_memory_error;

  *adc_object = ((uint32_t)i2c_read_data[1] << 16) | ((uint32_t)i2c_read_data[2] << 8) |
                (uint32_t)i2c_read_data[3];
  *adc_ambient = ((uint32_t)i2c_read_data[4] << 16) | ((uint32_t)i2c_read_data[5] << 8) |
                 (uint32_t)i2c_read_data[6];

  return tsd305_status_ok;
}

/**
 * \brief Reads the temperature and pressure ADC value and compute the
 * compensated values.
 *
 * \param[out] float* : Celsius Degree temperature value
 * \param[out] float* : mbar pressure value
 *
 * \return tsd305_status : status of TSD305
 *       - tsd305_status_ok : I2C transfer completed successfully
 *       - tsd305_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsd305_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsd305_status_busy : Sensor is busy
 *       - tsd305_status_memory_error : Sensor EEPROM memory error
 *       - tsd305_status_out_of_range : Sensor is out of range
 */
tsd305_status
read_temperature_and_object_temperature(float *temperature,
                                                float *object_temperature) {
  tsd305_status status = tsd305_status_ok;

  int32_t adc_object, adc_ambient;

  if (tsd305_coeff_read == false)
  {
    status = read_eeprom();
    if (status != tsd305_status_ok) return status;
  }

  status = conversion_and_read_adcs((uint32_t *)&adc_object,
                                         (uint32_t *)&adc_ambient);
  if (status != tsd305_status_ok)
  {
     printf ("CONVERT FAIL\n");
     return status;
  }
  else
  {
     //printf ("adc_object %ld adc_ambient %ld\n\n", adc_object, adc_ambient);
  }

  float ambFloat = (float) adc_ambient;
  float objFloat = (float) adc_object;
  //printf ("ambFloat %f\n", ambFloat);
  float t_sens = ((ambFloat)/16777216 * 105) - 20; // sensor temp



  float TCF = 1 + ((t_sens - coeff.refTemp) * coeff.tempCoeff); //Temperature Correction Factor

  float fOffset = 0;
  fOffset = fOffset + coeff.compK4* t_sens* t_sens* t_sens* t_sens;
  fOffset = fOffset + coeff.compK3* t_sens* t_sens* t_sens;
  fOffset = fOffset + coeff.compK2* t_sens* t_sens;
  fOffset = fOffset + coeff.compK1* t_sens;
  fOffset = fOffset + coeff.compK0;

  fOffset = fOffset * TCF;

  float c_adc = fOffset + ((objFloat - 8388608))/TCF;

  float t_obj = 0;
  t_obj = t_obj + coeff.adcK4 *c_adc*c_adc*c_adc*c_adc;
  t_obj = t_obj + coeff.adcK3 *c_adc*c_adc*c_adc;
  t_obj = t_obj + coeff.adcK2 *c_adc*c_adc;
  t_obj = t_obj + coeff.adcK1 *c_adc;
  t_obj = t_obj + coeff.adcK0;

  //printf ("foffset cadc tcf  %f %f %f\n", fOffset, c_adc, TCF);
  //printf ("sensor temp = %f\n", t_sens);

  *temperature = t_sens;
  *object_temperature = t_obj;

  return status;
}
