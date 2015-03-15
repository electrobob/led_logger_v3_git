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
	


	
	
	#include <inttypes.h>
	#include "leds.h"
	#include "tsl2550.h"
	#include <avr/io.h>
	#include "gammacorrection.h"
	#include "ws2812.h"
	

	//#include "global.h"
	//#define F_CPU 8000000
	#include <util/delay.h>

	#define NUMLEDS     5     // number of leds on strip

	static uint8_t MyData[NUMLEDS*3];
	static const uint8_t PROGMEM GammaTable[] = GCN_TABLE7TO8;


	void led_init(void){
		
		WS_init();
		
		//Make ports output
		PORTA.DIRSET = 0b11000000;
		//PORTA.OUT = 0b11000000;
		PORTA.OUT=0;
		
		PORTB.DIRSET = 0b0000001;
		//PORTB.OUT = 0b0000001;
		PORTB.OUT=0;
		
		//PORTC.DIRSET = 0b11111101;
		PORTC.DIRSET = 0b11111111;
		//PORTC.OUT = 0b11111100;
		PORTC.OUT=0;
		
		PORTD.DIRSET = 0b00111111;
		//PORTD.OUT = 0b00111111;
		PORTD.OUT=0;
		
		WS_init();
	}

	void LED_ON(uint8_t led){
		switch(led){
			case 1: PORTC.OUTSET = PIN3_bm;
			break;
			case 2:  PORTA.OUTSET = PIN7_bm;  //problem on this port, used channel 14 instead, something related to the WS leds
			break;
			case 3: PORTC.OUTSET = PIN5_bm;
			break;
			case 4: PORTC.OUTSET = PIN4_bm;
			break;
			case 5:  PORTC.OUTSET = PIN7_bm;
			break;
			case 6: PORTC.OUTSET = PIN6_bm;
			break;
			case 7: PORTD.OUTSET = PIN1_bm;
			break;
			case 8: PORTD.OUTSET = PIN0_bm;
			break;
			case 9: PORTD.OUTSET = PIN3_bm;
			break;
			case 10: PORTD.OUTSET = PIN2_bm;
			break;
			case 11: PORTD.OUTSET = PIN5_bm;
			break;
			case 12:  PORTD.OUTSET = PIN4_bm;
			break;
			case 13:  PORTB.OUTSET = PIN0_bm;
// 			_delay_ms(100);
// 			MyData[3]=127;
// 			MyData[4]=127;
// 			MyData[5]=127;
// 			MyData[6]=127;
// 			MyData[7]=127;
// 			MyData[8]=127;
// 			MyData[9]=127;
// 			MyData[10]=127;
// 			MyData[11]=127;
// 			MyData[12]=127;
// 			MyData[13]=127;
// 			MyData[14]=127;
// 				WS_init();
// 			WS_out(MyData, sizeof(MyData), GammaTable);
			break;
			case 14:  PORTC.OUTSET = PIN2_bm;
			break;
			case 15:  PORTA.OUTSET = PIN6_bm;
			break;
			case 16:
						MyData[3]=127;
			 			MyData[4]=127;
			 			MyData[5]=127;
			 			MyData[6]=127;
			 			MyData[7]=127;
			 			MyData[8]=127;
			 			MyData[9]=127;
			 			MyData[10]=127;
			 			MyData[11]=127;
			 			MyData[12]=127;
			 			MyData[13]=127;
			 			MyData[14]=127;
			
			 			WS_out(MyData, sizeof(MyData), GammaTable);
			break;
		}
	}

	void LED_OFF(uint8_t led){
		switch(led){
			case 1: PORTC.OUTCLR = PIN3_bm;
			break;
			case 2:  PORTA.OUTCLR = PIN7_bm; //problem on this port, used channel 14 instead, something related to the WS leds
			break;
			case 3: PORTC.OUTCLR = PIN5_bm;
			break;
			case 4: PORTC.OUTCLR = PIN4_bm;
			break;
			case 5:  PORTC.OUTCLR = PIN7_bm;
			break;
			case 6: PORTC.OUTCLR = PIN6_bm;
			break;
			case 7: PORTD.OUTCLR = PIN1_bm;
			break;
			case 8: PORTD.OUTCLR = PIN0_bm;
			break;
			case 9: PORTD.OUTCLR = PIN3_bm;
			break;
			case 10: PORTD.OUTCLR = PIN2_bm;
			break;
			case 11: PORTD.OUTCLR = PIN5_bm;
			break;
			case 12:  PORTD.OUTCLR = PIN4_bm;
			break;
			case 13:  PORTB.OUTCLR = PIN0_bm;
// 			_delay_ms(100);
// 			MyData[3]=127;
// 			MyData[3]=0;
// 			MyData[4]=0;
// 			MyData[5]=0;
// 			MyData[6]=0;
// 			MyData[7]=0;
// 			MyData[8]=0;
// 			MyData[9]=0;
// 			MyData[10]=0;
// 			MyData[11]=0;
// 			MyData[12]=0;
// 			MyData[13]=0;
// 			MyData[14]=0;
// 				WS_init();
// 			WS_out(MyData, sizeof(MyData), GammaTable);
			break;
			case 14:  PORTC.OUTCLR = PIN2_bm;
			break;
			case 15:  PORTA.OUTCLR = PIN6_bm;
			break;
			case 16:
			 			MyData[3]=127;
			 			MyData[3]=0;
			 			MyData[4]=0;
			 			MyData[5]=0;
			 			MyData[6]=0;
			 			MyData[7]=0;
			 			MyData[8]=0;
			 			MyData[9]=0;
			 			MyData[10]=0;
			 			MyData[11]=0;
			 			MyData[12]=0;
			 			MyData[13]=0;
			 			MyData[14]=0;
			
			 			WS_out(MyData, sizeof(MyData), GammaTable);
			break;
		}
	}

	// 	void LED1_ON(void){ PORTC.OUTSET = PIN3_bm; }
	// 	void LED1_OFF(void){ PORTC.OUTCLR = PIN3_bm; }
	// 	// 	void LED2_ON(void){ PORTC.OUTSET = PIN2_bm; }
	// 	// 	void LED2_OFF(void){ PORTC.OUTCLR = PIN2_bm; } //problem on this port, used channel 14 instead, something related to the WS leds
	// 	void LED2_ON(void){ PORTA.OUTSET = PIN7_bm; }
	// 	void LED2_OFF(void){ PORTA.OUTCLR = PIN7_bm; }
	//
	// 	void LED3_ON(void){ PORTC.OUTSET = PIN5_bm; }
	// 	void LED3_OFF(void){ PORTC.OUTCLR = PIN5_bm; }
	// 	void LED4_ON(void){ PORTC.OUTSET = PIN4_bm; }
	// 	void LED4_OFF(void){ PORTC.OUTCLR = PIN4_bm; }
	// 	void LED5_ON(void){ PORTC.OUTSET = PIN7_bm; }
	// 	void LED5_OFF(void){ PORTC.OUTCLR = PIN7_bm; }
	// 	void LED6_ON(void){ PORTC.OUTSET = PIN6_bm; }
	// 	void LED6_OFF(void){ PORTC.OUTCLR = PIN6_bm; }
	//
	// 	void LED7_ON(void){ PORTD.OUTSET = PIN1_bm; }
	// 	void LED7_OFF(void){ PORTD.OUTCLR = PIN1_bm; }
	// 	void LED8_ON(void){ PORTD.OUTSET = PIN0_bm; }
	// 	void LED8_OFF(void){ PORTD.OUTCLR = PIN0_bm; }
	// 	void LED9_ON(void){ PORTD.OUTSET = PIN3_bm; }
	// 	void LED9_OFF(void){ PORTD.OUTCLR = PIN3_bm; }
	// 	void LED10_ON(void){ PORTD.OUTSET = PIN2_bm; }
	// 	void LED10_OFF(void){ PORTD.OUTCLR = PIN2_bm; }
	// 	void LED11_ON(void){ PORTD.OUTSET = PIN5_bm; }
	// 	void LED11_OFF(void){ PORTD.OUTCLR = PIN5_bm; }
	// 	void LED12_ON(void){ PORTD.OUTSET = PIN4_bm; }
	// 	void LED12_OFF(void){ PORTD.OUTCLR = PIN4_bm; }
	//
	// 	void LED13_ON(void){ PORTB.OUTSET = PIN0_bm; }
	// 	void LED13_OFF(void){ PORTB.OUTCLR = PIN0_bm; }
	//
	// 	// 	void LED14_ON(void){ PORTA.OUTSET = PIN7_bm; }
	// 	// 	void LED14_OFF(void){ PORTA.OUTCLR = PIN7_bm; }//problem with LED2, LED14 is used instead of LED2
	// 	void LED14_ON(void){ PORTC.OUTSET = PIN2_bm; }
	// 	void LED14_OFF(void){ PORTC.OUTCLR = PIN2_bm; } //problem on this port, used channel 14 instead
	//
	// 	void LED15_ON(void){ PORTA.OUTSET = PIN6_bm; }
	// 	void LED15_OFF(void){ PORTA.OUTCLR = PIN6_bm; }
	//
	// 	void LED16_ON(void)	{
	// 		MyData[3]=127;
	// 		MyData[4]=127;
	// 		MyData[5]=127;
	// 		MyData[6]=127;
	// 		MyData[7]=127;
	// 		MyData[8]=127;
	// 		MyData[9]=127;
	// 		MyData[10]=127;
	// 		MyData[11]=127;
	// 		MyData[12]=127;
	// 		MyData[13]=127;
	// 		MyData[14]=127;
	//
	// 		WS_out(MyData, sizeof(MyData), GammaTable);
	// 	}
	
	// 	void LED16_OFF(void)	{
	// 		MyData[3]=0;
	// 		MyData[4]=0;
	// 		MyData[5]=0;
	// 		MyData[6]=0;
	// 		MyData[7]=0;
	// 		MyData[8]=0;
	// 		MyData[9]=0;
	// 		MyData[10]=0;
	// 		MyData[11]=0;
	// 		MyData[12]=0;
	// 		MyData[13]=0;
	// 		MyData[14]=0;
	//
	// 		WS_out(MyData, sizeof(MyData), GammaTable);
	// 	}


	void status_led(uint8_t Red, uint8_t Green, uint8_t Blue){
		MyData[0]=Green;
		MyData[1]=Red;
		MyData[2]=Blue;
		WS_out(MyData, sizeof(MyData), GammaTable);
	}

	
	void leds_ON(void){
		// 		LED1_ON();
		// 		LED2_ON();
		// 		LED3_ON();
		// 		LED4_ON();
		// 		LED5_ON();
		// 		LED6_ON();
		// 		LED7_ON();
		// 		LED8_ON();
		// 		LED9_ON();
		// 		LED10_ON();
		// 		LED11_ON();
		// 		LED12_ON();
		// 		LED13_ON();
		// 		LED14_ON();
		// 		LED15_ON();
		// 		LED16_ON();
		for(uint8_t i=1;i<=NUM_LEDS;i++){
			LED_ON(i);
		}
		
	}
	
	void leds_OFF(void){
		// 		LED1_OFF();
		// 		LED2_OFF();
		// 		LED3_OFF();
		// 		LED4_OFF();
		// 		LED5_OFF();
		// 		LED6_OFF();
		// 		LED7_OFF();
		// 		LED8_OFF();
		// 		LED9_OFF();
		// 		LED10_OFF();
		// 		LED11_OFF();
		// 		LED12_OFF();
		// 		LED13_OFF();
		// 		LED14_OFF();
		// 		LED15_OFF();
		// 		LED16_OFF();
		for(uint8_t i=1;i<=NUM_LEDS;i++){
			LED_OFF(i);
		}
		
	}
	
	void leds_scan(void){
		LED1_ON();
		_delay_ms(200);
		LED1_OFF();
		
		LED2_ON();
		_delay_ms(200);
		LED2_OFF();

		LED3_ON();
		_delay_ms(200);
		LED3_OFF();
		
		LED4_ON();
		_delay_ms(200);
		LED4_OFF();
		
		LED5_ON();
		_delay_ms(200);
		LED5_OFF();
		
		LED6_ON();
		_delay_ms(200);
		LED6_OFF();
		
		LED7_ON();
		_delay_ms(200);
		LED7_OFF();
		
		LED8_ON();
		_delay_ms(200);
		LED8_OFF();
		
		LED9_ON();
		_delay_ms(200);
		LED9_OFF();
		
		LED10_ON();
		_delay_ms(200);
		LED10_OFF();
		
		LED11_ON();
		_delay_ms(200);
		LED11_OFF();
		
		LED12_ON();
		_delay_ms(200);
		LED12_OFF();
		
		LED13_ON();
		_delay_ms(200);
		LED13_OFF();
		
		LED14_ON();
		_delay_ms(200);
		LED14_OFF();
		
		LED15_ON();
		_delay_ms(200);
		LED15_OFF();
		
		LED16_ON();
		_delay_ms(200);
		LED16_OFF();

	}
	
	// Read the AD conversion result
	// uint16_t read_adc(unsigned char adc_input)
	// {
	// 	ADMUX=adc_input | (0x40 & 0xdf);
	// 	// Delay needed for the stabilization of the ADC input voltage
	// 	_delay_us(10);
	// 	// Start the AD conversion
	// 	ADCSRA|=0x40;
	// 	// Wait for the AD conversion to complete
	// 	while ((ADCSRA & 0x10)==0);
	// 	ADCSRA|=0x10;
	// 	return ADCW;
	// }
	/*
	void leds_full_measurement(uint16_t *ll_lux_dec)
	{
	
	//first read the adc
	LED5_OFF;
	DDRC &= ~(1<<PC3);
	_delay_ms(800);
	ll_lux_dec[8] =  read_adc(3);
	DDRC |= (1<<PC3);
	
	//perform all channel measurement
	//all off
	LED0_OFF;
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
	LED4_OFF;
	LED5_OFF;
	LED6_OFF;
	LED7_OFF;
	

	
	LED0_ON;
	_delay_ms(1500);
	ll_lux_dec[0] = TLS2550_lux_calc_dec();
	LED0_OFF;
	
	LED1_ON;
	_delay_ms(1500);
	ll_lux_dec[1] = TLS2550_lux_calc_dec();
	LED1_OFF;
	
	LED2_ON;
	_delay_ms(1500);
	ll_lux_dec[2] = TLS2550_lux_calc_dec();
	LED2_OFF;
	
	LED3_ON;
	_delay_ms(1500);
	ll_lux_dec[3] = TLS2550_lux_calc_dec();
	LED3_OFF;
	
	LED4_ON;
	_delay_ms(1500);
	ll_lux_dec[4] = TLS2550_lux_calc_dec();
	LED4_OFF;
	
	LED5_ON;
	_delay_ms(1500);
	ll_lux_dec[5] = TLS2550_lux_calc_dec();
	LED5_OFF;
	
	LED6_ON;
	_delay_ms(1500);
	ll_lux_dec[6] = TLS2550_lux_calc_dec();
	LED6_OFF;
	
	LED7_ON;
	_delay_ms(1500);
	ll_lux_dec[7] = TLS2550_lux_calc_dec();
	LED7_OFF;
	
	//all except control on
	LED1_ON;
	LED2_ON;
	LED3_ON;
	LED4_ON;
	LED5_ON;
	LED6_ON;
	LED7_ON;
	}*/
	#ifdef __cplusplus
}
#endif