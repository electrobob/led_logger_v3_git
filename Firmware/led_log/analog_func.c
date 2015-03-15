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
	
#include <asf.h>
#include "analog_func.h"
#include <util/delay.h>
//#include "hw_init.h"


uint16_t adc_offset_m;
//Initialize the ADC
//the ADC uses 4 channels, automatic trigger. data wil be placed in the result registers constantly.
//it will be up to the main thread of the supply to average the results
// clock chosen to be 2MHz, should be 2 MSps???
void adc_init_src(void)
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;

	adc_read_configuration(&MY_ADC, &adc_conf); //why exactly do I need this line
	adcch_read_configuration(&MY_ADC, ADC_CH0, &adcch_conf); //and this one ?

	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12,	ADC_REF_AREFA); //data is 2's complement in signed, be careful!
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_FREERUN_SWEEP, 3, 0); //dont forget the _SWEEP if more than 1 ch
	adc_set_clock_rate(&adc_conf, 10000);
	adc_write_configuration(&MY_ADC, &adc_conf);

	adcch_set_input(&adcch_conf, V5V_READ_ADC, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&MY_ADC, ADC_CH0, &adcch_conf);

	adcch_set_input(&adcch_conf, ADCCH_POS_PIN2, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&MY_ADC, ADC_CH1, &adcch_conf);
// 	
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN1, ADCCH_NEG_NONE, 1); //use this for offset readout
	adcch_write_configuration(&MY_ADC, ADC_CH2, &adcch_conf);
// 	
// 	adcch_set_input(&adcch_conf, V3_READ_ADC, ADCCH_NEG_NONE, 1);
// 	adcch_write_configuration(&MY_ADC, ADC_CH3, &adcch_conf);


}


void adc_read_voltages(double *ll_vdc_supply){
			uint32_t temp;
			struct adc_config adc_conf;
			struct adc_channel_config adcch_conf;

			adc_read_configuration(&MY_ADC, &adc_conf); 
			adcch_read_configuration(&MY_ADC, ADC_CH0, &adcch_conf); 
		
		//select 5V channel for measurement
		adcch_set_input(&adcch_conf, V5V_READ_ADC, ADCCH_NEG_NONE, 1); //ADCCH_NEG_NONE
		adcch_write_configuration(&MY_ADC, ADC_CH0, &adcch_conf);
		_delay_us(312);	
		temp=0;
		for(uint16_t i=0;i<ADC_AVG_NO;i++){
		temp+=adc_get_result(&MY_ADC, ADC_CH0);
		_delay_us(312);//avg 50Hz noise over 64 samples
		}
		ll_vdc_supply[0]=(temp/ADC_AVG_NO-adc_offset_m)*ADC_VREF_V/ADC_COUNT_TO*ADC_V5V_GAIN;
		if(ll_vdc_supply[0]>(ADC_VREF_V*ADC_V5V_GAIN)) ll_vdc_supply[0]=0; //if value too large actually went to neg values
	
		//select V1 channel for measurement
		adcch_set_input(&adcch_conf, V1_READ_ADC, ADCCH_NEG_NONE, 1); //ADCCH_NEG_NONE
		adcch_write_configuration(&MY_ADC, ADC_CH0, &adcch_conf);
		_delay_us(312);
		temp=0;
		for(uint16_t i=0;i<ADC_AVG_NO;i++){
			temp+=adc_get_result(&MY_ADC, ADC_CH0);
			_delay_us(312);//avg 50Hz noise over 64 samples
		}
		ll_vdc_supply[1]=(temp/ADC_AVG_NO-adc_offset_m)*ADC_VREF_V/ADC_COUNT_TO*ADC_V1_GAIN;
		if(ll_vdc_supply[1]>(ADC_VREF_V*ADC_V1_GAIN)) ll_vdc_supply[1]=0; //if value too large actually went to neg values	
		
		//select V2 channel for measurement
		adcch_set_input(&adcch_conf, V2_READ_ADC, ADCCH_NEG_NONE, 1); //ADCCH_NEG_NONE
		adcch_write_configuration(&MY_ADC, ADC_CH0, &adcch_conf);
		_delay_us(312);
		temp=0;
		for(uint16_t i=0;i<ADC_AVG_NO;i++){
			temp+=adc_get_result(&MY_ADC, ADC_CH0);
			_delay_us(312);//avg 50Hz noise over 64 samples
		}
		ll_vdc_supply[2]=(temp/ADC_AVG_NO-adc_offset_m)*ADC_VREF_V/ADC_COUNT_TO*ADC_V2_GAIN;
		if(ll_vdc_supply[2]>(ADC_VREF_V*ADC_V2_GAIN)) ll_vdc_supply[2]=0; //if value too large actually went to neg values	
		
		//select V3 channel for measurement
		adcch_set_input(&adcch_conf, V3_READ_ADC, ADCCH_NEG_NONE, 1); //ADCCH_NEG_NONE
		adcch_write_configuration(&MY_ADC, ADC_CH0, &adcch_conf);
		_delay_us(312);
		temp=0;
		for(uint16_t i=0;i<ADC_AVG_NO;i++){
			
			_delay_us(312);//avg 50Hz noise over 64 samples
			temp+=adc_get_result(&MY_ADC, ADC_CH0);
		}
		ll_vdc_supply[3]=(temp/ADC_AVG_NO-adc_offset_m)*ADC_VREF_V/ADC_COUNT_TO*ADC_V3_GAIN;	
		if(ll_vdc_supply[3]>(ADC_VREF_V*ADC_V3_GAIN)) ll_vdc_supply[3]=0; //if value too large actually went to neg values		
	
}


double adc_read_current(void){
	
double current=0;
		uint16_t temp=0;
			for(uint16_t i=0;i<ADC_AVG_NO;i++){
				_delay_us(312);//avg 50Hz noise over 64 samples
				temp+=adc_get_result(&MY_ADC, ADC_CH1);
			}
			current=(temp/ADC_AVG_NO-adc_offset_m)*ADC_VREF_V/ADC_COUNT_TO*ADC_I_GAIN;
			if(current>(ADC_VREF_V*ADC_I_GAIN)) current=0; //if value too large actually went to neg values
			return current;
}

uint16_t adc_get_offset(void){
	
	uint16_t offset=0;
	
	for(uint16_t i=0;i<ADC_AVG_NO;i++){
		offset += adc_get_result(&MY_ADC, ADC_CH2);
		_delay_us(312);//avg 50Hz noise over 64 samples
	}
	offset=offset/ADC_AVG_NO;
	return offset;
}
	#ifdef __cplusplus
}
#endif