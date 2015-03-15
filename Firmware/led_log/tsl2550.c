/*
 * tsl2550.c
 *
 * Created: 9/23/2012 00:02:10
 *  Author: Bogdan Raducanu
 *  Website: www.electrobob.com
 *  Free for non commercial use.
 */

#ifdef __cplusplus
extern "C" {
	#endif


#include <inttypes.h>
#include <math.h>
#include "tsl2550.h"

const uint16_t TSL2550_chord[] = {0, 16, 49, 115, 247, 511, 1039, 2095};
const uint16_t TSL2550_step[]={1,2,4,8,16,32,64,128};
	
	



void TSL2550_init(void)
{
	uint8_t data = TSL2550_POWER_UP;
		TWI_MasterWrite(&twiMaster, TSL2550_ADDR, &data, 1);
		while (twiMaster.status != TWIM_STATUS_READY) {
			/* Wait until transaction is complete. */
		}

}

uint8_t TSL2550_get_ADC0(void)
{

	uint8_t data = TSL2550_READ_ADC0;
	TWI_MasterWriteRead(&twiMaster, TSL2550_ADDR, &data, 1,1);
	while (twiMaster.status != TWIM_STATUS_READY) {
		/* Wait until transaction is complete. */
	}
	return twiMaster.readData[0];
	
}

uint8_t TSL2550_get_ADC1(void)
{
	uint8_t data = TSL2550_READ_ADC1;
	TWI_MasterWriteRead(&twiMaster, TSL2550_ADDR, &data, 1,1);
	while (twiMaster.status != TWIM_STATUS_READY) {
		/* Wait until transaction is complete. */
	}
	return twiMaster.readData[0];

}

uint16_t TSL2550_ADC_CALC(uint8_t ADC_raw_data)
{
	ADC_raw_data = ADC_raw_data & 0x7F;
	return TSL2550_chord[ADC_raw_data>>4] + TSL2550_step[ADC_raw_data>>4]*(ADC_raw_data & 0x0F);
	
}

float TLS2550_lux_calc(uint8_t TSL2550_ADC0, uint8_t TSL2550_ADC1)
{
	float lux, R;
	lux = TSL2550_ADC_CALC(TSL2550_ADC0);
	R = TSL2550_ADC_CALC(TSL2550_ADC1);
	R = R / lux;
	R = (-3.13)* R;
	R = exp (R);
	R = R * 0.46;
	lux = lux * R;
	return lux;
}


uint16_t TLS2550_lux_calc_dec(void)
{
	float lux, R;
	uint16_t ll_lux_dec;
	uint8_t TSL2550_ADC0, TSL2550_ADC1;
					TSL2550_ADC0 = TSL2550_get_ADC0() & 0x7F;
					//TSL2550_ADC0 = 66;
					TSL2550_ADC1 = TSL2550_get_ADC1() & 0x7F;
	lux = TSL2550_ADC_CALC(TSL2550_ADC0);
	R = TSL2550_ADC_CALC(TSL2550_ADC1);
	R = R / lux;
	R = (-3.13)* R;
	R = exp (R);
	R = R * 0.46;
	lux = lux * R;
	ll_lux_dec = lux;
	return ll_lux_dec;
}

#ifdef __cplusplus
}
#endif