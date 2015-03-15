/*
* Led Logger v3
*
* Created: 15.03.2015
*  Author: Bogdan Raducanu
* copyright (c) Bogdan Raducanu, 2015
* www.electrobob.com
* bogdan@electrobob.com
* Released under GPLv3.
* Please refer to LICENSE file for licensing information.
*/

#ifdef __cplusplus
extern "C" {
	#endif
	
#ifndef ANALOG_H_
#define ANALOG_H_


#define MY_ADC    ADCA
#define V5V_READ_ADC ADCCH_POS_PIN3
#define V1_READ_ADC ADCCH_POS_PIN4
#define V2_READ_ADC ADCCH_POS_PIN5
#define V3_READ_ADC ADCCH_POS_PIN9


//#define ADC_OFFSET -5

#define ADC_VREF_V 2.5
#define ADC_COUNT_TO 4096

#define ADC_V5V_GAIN 15.78
#define ADC_V1_GAIN 15.71
#define ADC_V2_GAIN 15.80
#define ADC_V3_GAIN 15.84
#define ADC_I_GAIN 3.633 // 1/(r_shunt*amp_gain)


#define ADC_AVG_NO 256

//#define RATE_OF_CONVERSION       500
//#define OUTPUT_DAC               DACB

// #define V_SRC_REF DAC_CH0
// #define I_SRC_REF DAC_CH1

#endif /* ANALOG_H_ */

void adc_init_src(void);
void dac_init_src(void);
void adc_read_voltages(double *ll_vdc_supply);
double adc_read_current(void);
uint16_t adc_get_offset(void);

#ifdef __cplusplus
}
	#endif