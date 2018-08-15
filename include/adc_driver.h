#ifndef _ADC_DRIVER_H
#define _ADC_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <stdbool.h>

// External triggers
typedef enum ADC_INTERFACE_ID
{
  ADC_THERMISTOR = 0,
  ADC_TACTILE = 1,
  ADC_TORQUE = 2,

  ADC_NUMBER_OF_ADCS
} ADC_INTERFACE_ID;

#define ADC_INTERFACE_INVALID (((ADC_INTERFACE_ID) -1))

#define ADC_MAX_INDEX 15

// Initialize the trigger driver
int adc_initialize();
// Deinitialize the trigger driver
void adc_deinitialize();

// Read the value from the ADC
uint32_t adc_read_value(ADC_INTERFACE_ID adc_id, uint8_t adc_num);

#ifdef __cplusplus
}
#endif

#endif // _ADC_DRIVER_H
