/*
 * TSD305.h
 *
 *  Created on: May 13, 2020
 *      Author: djhoo
 */

#ifndef DRIVERS_TE_CONNECTIVITY_TSD305_TSD305_H_
#define DRIVERS_TE_CONNECTIVITY_TSD305_TSD305_H_


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

#endif /* DRIVERS_TE_CONNECTIVITY_TSD305_TSD305_H_ */
