/*
 * tsl2550.h
 *
 * Created: 9/23/2012 00:01:53
 *  Author: Bogdan Raducanu
 *  Website: www.electrobob.com
 *  Free for non commercial use.
 */


#ifdef __cplusplus
extern "C" {
	#endif
#include "twi_master_driver.h"
//#include <inttypes.h>
//#define F_CPU 8000000

//address will be shifted 1 bit
#define TSL2550_ADDR 0x39
#define TSL2550_POWER_DOWN 0x00
#define TSL2550_POWER_UP 0x03
#define TSL2550_EXTENDED 0x1d
#define TSL2550_STANDARD 0x18
#define TSL2550_READ_ADC0 0x43
#define TSL2550_READ_ADC1 0x83



//void TSL2550_pretty_printer(uint8_t ADC0_value, uint8_t ADC1_value, char *string);
extern void TSL2550_init(void);
extern uint8_t TSL2550_get_ADC0(void);
extern uint8_t TSL2550_get_ADC1(void);
extern unsigned int TSL2550_ADC_CALC(uint8_t ADC_raw_data);
extern uint16_t TLS2550_lux_calc_dec(void);
extern TWI_Master_t twiMaster;    /*!< TWI master module. */




#ifdef __cplusplus
}
#endif