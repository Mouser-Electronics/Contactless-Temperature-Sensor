/**************************************************************************//**
MIT License

Copyright (c) 2016 TE Connectivity
Modified work Copyright 2020 Connected Development, Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 *
 *******************************************************************************
 * @file
 * @brief Implements TE TSD305 Thermopile Drivers.
 ******************************************************************************/

#ifndef DRIVERS_TE_CONNECTIVITY_TSD305_TSD305_H_
#define DRIVERS_TE_CONNECTIVITY_TSD305_TSD305_H_

#include "app_rtcc.h"

typedef enum {
  tsd305_status_ok,
  tsd305_status_no_i2c_acknowledge,
  tsd305_status_i2c_transfer_error,
  tsd305_status_busy,
  tsd305_status_memory_error,
  tsd305_status_out_of_range
} tsd305_status;

struct {
  float tempCoeff;
  float refTemp;
  float compK4;
  float compK3;
  float compK2;
  float compK1;
  float compK0;
  float adcK4;
  float adcK3;
  float adcK2;
  float adcK1;
  float adcK0;
} coeff;


int32_t get_lut_at(uint8_t x, uint8_t y);


  void TSD305I2CBegin();
  tsd305_status read_temperature_and_object_temperature(float *temperature,
                                          float *object_temperature);
  tsd305_status read_eeprom_coeff(uint8_t address, uint16_t *coeff);
  tsd305_status read_eeprom_float(uint8_t address, float *value);
  tsd305_status conversion_and_read_adcs(uint32_t *adc_object,
                                              uint32_t *adc_ambient);
  tsd305_status read_eeprom(void);
  void Delay (uint32_t dlyTicks);

#endif /* DRIVERS_TE_CONNECTIVITY_TSD305_TSD305_H_ */
