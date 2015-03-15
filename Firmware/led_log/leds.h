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

	#define NUM_LEDS 16
	#define CONTROL_LED 15
	//#include <inttypes.h>
	//#include "global.h"

	void led_init(void);

	void LED1_ON(void);
	void LED1_OFF(void);
	void LED2_ON(void);
	void LED2_OFF(void);
	void LED3_ON(void);
	void LED3_OFF(void);
	void LED4_ON(void);
	void LED4_OFF(void);
	void LED5_ON(void);
	void LED5_OFF(void);
	void LED6_ON(void);
	void LED6_OFF(void);

	void LED7_ON(void);
	void LED7_OFF(void);
	void LED8_ON(void);
	void LED8_OFF(void);
	void LED9_ON(void);
	void LED9_OFF(void);
	void LED10_ON(void);
	void LED10_OFF(void);
	void LED11_ON(void);
	void LED11_OFF(void);
	void LED12_ON(void);
	void LED12_OFF(void);

	void LED13_ON(void);
	void LED13_OFF(void);

	void LED14_ON(void);
	void LED14_OFF(void);
	void LED15_ON(void);
	void LED15_OFF(void);
	
	void LED16_ON(void);
	void LED16_OFF(void);
	
	void leds_scan(void);
	
	void LED_ON(uint8_t led);
		void LED_OFF(uint8_t led);

	void status_led(uint8_t Red, uint8_t Green, uint8_t Blue);
	
	
		void leds_ON(void);
			void leds_OFF(void);
	
	/*


	extern void leds_full_measurement(uint16_t ll_lux_dec[]);
	*/
	#ifdef __cplusplus
}
#endif