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

//Led logger V2 operated about 6 months, 15.12.2012 till 16.06.2013 = 4400H = 15.840.000 seconds
//leds mapping: box >> channel, time used on previous designs, initial intensity:
// 1 >>> 1, 1400+4400=20.088.000sec ,536lux 
// 2 >>> 2, 15.840.000, 538lux
// 3 >>> 3, 15.840.000, 429lux
// 4 >>> 4, 15.840.000, 460 lux
// 5 >>> 5, 0, 182
// 6 >>> 6, 0, 801
// Control >>> 15, 15.840.000 sec, 653 lux
// WS2812 >>> 16, 0, 601

#define MEASURE_INTERVAL 3600 //measurement interval in seconds
#define LIGHT_AVG_NO 4 //how many averages to make

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <asf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "secret.h"
#include "serial.h"
#include "tsl2550.h"
#include "gammacorrection.h"
#include "avr_compiler.h"
#include "twi_master_driver.h"
#include "twi_slave_driver.h"
#include "ws2812.h"
#include "leds.h"
#include "analog_func.h"
#include "onewire.h"
#include "ds18x20.h"
#include "eeprom_driver.h"


//#define OW_ONE_BUS 1
#define HS_MODE_RUNNING 1
#define HS_MODE_STOP 0
#define MAXSENSORS 5
#define NEWLINESTR "\r\n"

/*! Defining an example slave address. */
#define SLAVE_ADDRESS    0x22

/*! Defining number of bytes in buffer. */
#define NUM_BYTES        8

/*! CPU speed 32MHz, BAUDRATE 100kHz and Baudrate Register Settings */
#define CPU_SPEED       32000000
#define BAUDRATE	100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)


/* Global variables */
TWI_Master_t twiMaster;    /*!< TWI master module. */
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)
ISR(TWIE_TWIM_vect){ TWI_MasterInterruptHandler(&twiMaster); }

/*! Buffer with test data to send.*/
uint8_t sendBuffer[NUM_BYTES] = {0x00, 0x00, 0xF0, 0x0F, 0xB0, 0x0B, 0xDE, 0xAD};

uart debug(&USARTD1, 115200);
ISR (USARTD1_RXC_vect){ debug.rxInterrupt(); }
ISR (USARTD1_DRE_vect){ debug.txInterrupt(); }

uart esp_serial(&USARTE0, 115200);
ISR (USARTE0_RXC_vect){ esp_serial.rxInterrupt(); }
ISR (USARTE0_DRE_vect){ esp_serial.txInterrupt(); }

uint16_t ll_meas_no=0;
uint16_t ll_lux_dec[NUM_LEDS];
double ll_idc_leds[NUM_LEDS];
uint32_t ll_time_leds[NUM_LEDS];
double ll_vdc_supply[4];
double ll_temperature;


char csv_data_output[512];
//char csv_data_output[128];

uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
uint8_t sens_id;
char hs_mode = HS_MODE_STOP;
uint8_t nSensors;
int16_t decicelsius; //temp in deg C * 10

extern uint16_t adc_offset_m;
uint8_t auto_transmit=0;
uint32_t pack_cntr = 0;
uint32_t pack_cntr_rx = 0;
uint32_t pack_cntr_rx_old=0;
uint32_t pack_cntr_rx_missed=0;
char out_str[16];
uint16_t var0 = 50;
uint16_t var1=0;
uint16_t var2=200;
uint32_t time, time_slow,time_now, time_last_measure, time_last_loop;
uint16_t vin=0;
uint16_t current = 74;
uint16_t dac=43;
uint16_t power_set=37;
uint16_t power_read = 23;
uint16_t temp_set = 75;
uint16_t temp_read = 42;
uint16_t rth = 33;
uint8_t stat_led_color;



char cmd[1024];
char counter_hits[9];
char ip_string[16];
char esp_rx_buf[256];
uint16_t esp_rx_buf_ptr=0;

char wifi_connected=0;
uint16_t global_wifi_reconn=0;
uint16_t global_wifi_attempts=0;
uint16_t counter_succes=0;

double adcres;
double LED_current;

//read old data hours from eeprom. Warning, assumes all data fits in one eeprom page, 128 bytes
void ll_read_eeprom_times(void)
{
	debug.sendString("\n\r read to eprom\n\r");
	for(uint8_t i =0; i<NUM_LEDS; i++)
	{
		ll_time_leds[i]=EEPROM_ReadByte(i*4/EEPROM_PAGESIZE, i*4+0);//itoa(EEPROM_ReadByte(i*4/EEPROM_PAGESIZE, i*4+0), out_str, 10);debug.sendString(out_str);debug.sendString(".");
 		ll_time_leds[i]= ( ll_time_leds[i]<<8 ) + EEPROM_ReadByte(i*4/EEPROM_PAGESIZE, i*4+1);//itoa(EEPROM_ReadByte(i*4/EEPROM_PAGESIZE, i*4+1), out_str, 10);debug.sendString(out_str);debug.sendString(".");
 		ll_time_leds[i]= ( ll_time_leds[i]<<8 ) + EEPROM_ReadByte(i*4/EEPROM_PAGESIZE, i*4+2);//itoa(EEPROM_ReadByte(i*4/EEPROM_PAGESIZE, i*4+2), out_str, 10);debug.sendString(out_str);debug.sendString(".");
 		ll_time_leds[i]= ( ll_time_leds[i]<<8 ) + EEPROM_ReadByte(i*4/EEPROM_PAGESIZE, i*4+3);//itoa(EEPROM_ReadByte(i*4/EEPROM_PAGESIZE, i*4+3), out_str, 10);debug.sendString(out_str);
		debug.sendString("\n\reeread time ");
		itoa(i, out_str,10); debug.sendString(out_str);
		debug.sendString("=");
		ltoa(ll_time_leds[i], out_str,10); debug.sendString(out_str);
	}
}

//read old data hours from eeprom. Warning, assumes all data fits in one eeprom page, 128 bytes
void ll_write_eeprom_times(void)
{
	debug.sendString("\n\r write to eprom\n\r");
	uint8_t temp;
	for(uint8_t i =0; i<NUM_LEDS; i++)
	{
		temp = (ll_time_leds[i]); EEPROM_WriteByte(i*4/EEPROM_PAGESIZE, i*4+3, temp);// itoa(temp, out_str, 10);debug.sendString(out_str);debug.sendString(".");
		temp = (ll_time_leds[i] >>8); EEPROM_WriteByte(i*4/EEPROM_PAGESIZE, i*4+2, temp);//itoa(temp, out_str, 10);debug.sendString(out_str);debug.sendString(".");
		temp = (ll_time_leds[i] >> 16); EEPROM_WriteByte(i*4/EEPROM_PAGESIZE, i*4+1, temp);//itoa(temp, out_str, 10);debug.sendString(out_str);debug.sendString(".");
		temp = (ll_time_leds[i] >> 24); EEPROM_WriteByte(i*4/EEPROM_PAGESIZE, i*4+0, temp);//itoa(temp, out_str, 10);debug.sendString(out_str);
		//debug.sendString("\n\r");
	}
}

//dumps the data in the esp buffer to the debug serial
void esp_dump(void){
	debug.sendString("\r ESP RX:");
	while(esp_serial.dataAvailable()){
		debug.sendChar(esp_serial.getChar());
	}
	debug.sendString("\r");
}

void esp_clear_usart_buff(void){ while(esp_serial.dataAvailable()) esp_serial.getChar(); }

//copy data from esp to the working buffer, make sure do to this after waiting for a while
void esp_cpy(void){
	uint16_t esp_rx_buf_ptr=0;
	while(esp_serial.dataAvailable()){
		esp_rx_buf[esp_rx_buf_ptr]=esp_serial.getChar();
		esp_rx_buf_ptr++;
	}
	esp_rx_buf[esp_rx_buf_ptr]='\0'; //make sure to null terminate string
	//debug.sendString("\rESP RX buff: ");
	//debug.sendString(esp_rx_buf);
}

uint8_t get_IP(void){//should implement a zero time delay, wait for something like 4 x . in received, then check for ip
	//dump the data
	esp_serial.flush();
	esp_clear_usart_buff();
	debug.sendString("\n\rget ip\n\r");
	//check if IP is acquired
	esp_serial.sendString("AT+CIFSR\r"); //so have to check IP assigned
	_delay_ms(2000); //wait for the rest of the data to come in.
	esp_cpy();
	char *find_ptr= strstr(esp_rx_buf, IP_BASE_EXP);
	if(find_ptr!=NULL){ //simple check, look for 192.168 in ip
		debug.sendStringPgm(PSTR(" OK\r\n"));
		//strcpy()
		for(uint8_t i=0; i<15; i++){
			ip_string[i]=*find_ptr;
			find_ptr++;
		}
		ip_string[15]='\0';
		return 1;
	}
	else return 0;
}

//connect to wifi with maximum attempts number,  infinite for 0.
//returns 1 for succesful conection, 0 for failed
//resets device before attempting wifi
uint8_t connectWiFi(uint16_t wifi_conn_attempts)
{
	uint8_t attempts=1;
	while((attempts <= wifi_conn_attempts) || (wifi_conn_attempts==0)){
		
		debug.sendStringPgm(PSTR("\n\rConnect to wifi attempt ")); debug.sendInt(attempts);
		attempts++;
		global_wifi_attempts++;
		esp_clear_usart_buff();
		esp_serial.flush();
		esp_serial.sendStringPgm(PSTR("AT+RST\r\n")); //should add HW reset, SW does not work if device is stuck
		//esp_wait_for_data("ready", 1000);
		while(esp_serial.dataAvailable()<5);
		_delay_ms(1000);
		
		esp_clear_usart_buff();
		esp_serial.flush();
		esp_serial.sendString("AT+CWMODE=1\r");
		//esp_wait_for_data("no change", 1000);
		while(esp_serial.dataAvailable()<5);
		_delay_ms(1000);
		
		esp_clear_usart_buff();
		esp_serial.flush();
		esp_serial.sendString("AT+CWJAP=\"");
		esp_serial.sendString(SSID);
		esp_serial.sendString("\",\"");
		esp_serial.sendString(PASS_SSID);
		esp_serial.sendString("\"\r");
		//SOB returns OK even if wifi does not exist, cannot use that for monitoring
		//esp_wait_for_data("OK", 1000);
		while(esp_serial.dataAvailable()<5);
		_delay_ms(1000);
		_delay_ms(5000);
		//check if IP is aquired
		if(get_IP())
		{
			debug.sendStringPgm(PSTR(" OK\r\n"));
			debug.sendString("\r\nIP: ");
			debug.sendString(ip_string);
			return 1;
		}
		else
		{
			debug.sendStringPgm(PSTR(" Failed! Buffer dump: "));
			debug.sendStringPgm(esp_rx_buf);
			_delay_ms(5000); //wait 5 seconds before retry
		}
	}
	return 0;
}

//upload the data to emoncms
void upload_electrobob_data(void){
	esp_serial.flush();
	debug.sendString("\rAttempt to upload led_logger_data\r");
	cmd[0]='\0';
	strcat(cmd, "AT+CIPSTART=\"TCP\",\"");
	strcat(cmd, ELECTROBOB_IP);
	strcat(cmd, "\",80\r");
	esp_serial.sendString(cmd);
	//wait for connection
	while(esp_serial.dataAvailable()<3);
	_delay_ms(1000);
	
	//construct command
	cmd[0]='\0';
	strcat(cmd, "GET ");
	strcat(cmd, API_LINK);
	strcat(cmd, csv_data_output);
	strcat(cmd, counter_hits);
	strcat(cmd, "&apikey=");
	strcat(cmd, API_KEY);
	strcat(cmd, " HTTP/1.0\r\nHost: electrobob.com\r\n\r\n\r");
	
	debug.sendString("\n\r command string: ");
	debug.sendString(cmd);
	//send command and size
	esp_serial.sendStringPgm(PSTR("AT+CIPSEND="));
	esp_serial.sendInt(strlen(cmd));
	esp_serial.sendString("\r");
	
	//wait for prompt
	while(esp_serial.dataAvailable()<2);
	_delay_ms(100);
	esp_serial.sendString(cmd);
	
	//wait for data
	while(esp_serial.dataAvailable()<3);
	_delay_ms(1000);
}


static uint8_t search_sensors(void)
{
	uint16_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	ow_reset();
	nSensors = 0;
	diff = OW_SEARCH_FIRST;
	while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ) {
		DS18X20_find_sensor( &diff, &id[0] );
		if( diff == OW_PRESENCE_ERR ) {
			debug.sendString( "No Sensor found"  );
			break;
		}
		if( diff == OW_DATA_ERR ) {
			debug.sendString( "Bus Error"  );
			break;
		}
		for ( i=0; i < OW_ROMCODE_SIZE; i++ )
		gSensorIDs[nSensors][i] = id[i];
		nSensors++;
	}
	return nSensors;
}

//should be called only once, it writes the intial data in eeprom
void ll_eeprom_write_orig_data(void){
for(uint8_t i=0; i<NUM_LEDS;i++) ll_time_leds[i]=0; //set all to zero intially
ll_time_leds[0]=20088000;	//led 1
ll_time_leds[1]=15840000;	//led 2
ll_time_leds[2]=15840000;	//led 3
ll_time_leds[3]=15840000;	//led 4
ll_time_leds[14]=15840000;	//control
ll_write_eeprom_times();
}

int main(void)
{
	sysclk_init(); //this SOB turns off clock to all peripherals. Disable that "FEATURE".
	rtc_init();
	time_slow=0;

	/* Initialize TWI master. */
	TWI_MasterInit(&twiMaster,
	&TWIE,
	TWI_MASTER_INTLVL_LO_gc,
	TWI_BAUDSETTING);

	//Enable all interrupts
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	/* Enable LO interrupt level. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	sei();

	_delay_ms(10);
	debug.sendString("\n\n\n\n\n\n\n\n\nWelcome to led Logger V3\n");
	debug.sendString("ADC start init\n");
	adc_init_src();
	
	adc_enable(&MY_ADC);
	debug.sendString("ADC initialized\n");

	nSensors = search_sensors();
	if ( nSensors == 1 ) {
		debug.sendString("Temp sensor initialized");
		sens_id = gSensorIDs[0][0]; // family-code for conversion-routine
		DS18X20_start_meas( DS18X20_POWER_PARASITE, NULL );
		_delay_ms( DS18B20_TCONV_12BIT );
		debug.sendString(out_str);
		debug.sendString( "\n" );
	}
	else
	{
		debug.sendString("Oopsie no temp sens found");
	}

	debug.sendString("The time is ");
	time = rtc_get_time();
	ltoa(time, out_str, 10);
	debug.sendString(out_str);
	debug.sendString("\n");

	TSL2550_init();
	debug.sendString("TSL initialized\n");
	led_init();
	debug.sendString("LEDS initialized\n");
	debug.sendString("status led test\n");
	
	status_led(127,0,0);
	_delay_ms(200);
	status_led(0,127,0);
	_delay_ms(200);
	status_led(0,0,127);
	_delay_ms(200);
	
		debug.sendString("\n\rLoading eeprom data");
		//ll_eeprom_write_orig_data(); //to be called once only ~!!!!!
		
		ll_read_eeprom_times();
	
	//Connect to wifi
	_delay_ms(2000);//wait from start
	
	while(!(connectWiFi(10))) {
		//wifi connect failed after 10 attempts, let's wait for a while
		debug.sendString("\r\nWifi connect failed, waiting and retrying\r\n");
		_delay_ms(60000);
	}
	global_wifi_reconn++;

	
	debug.sendString("LEDs scan\n");
	//leds_scan();
	debug.sendString("LEDs scan finish\n");
	//measure the ADC offset
	adc_offset_m = adc_get_offset();
	debug.sendString("ADC got offset\n");
	itoa(adc_offset_m, out_str,10);
	debug.sendString("ADC_OFF:");
	debug.sendString(out_str);
	debug.sendString("\r\n");
	
	status_led(0,50,0);
	
	leds_ON(); //let's brighten the day
	LED_OFF(CONTROL_LED); //make sure control stays off
	
	while(1)
	{
		//take proper care of time in the longer 32b counter, normal counter counts till 65536 even though it is 32bits
		time = rtc_get_time();
		time_now = time_slow+time;
		if(time>32768) {
			time_slow=time_now;
			rtc_set_time(0);
		}
		
		if(time_last_loop!=time_now){
			time_last_loop=time_now;
			//change the status led color
			stat_led_color++;
			if(stat_led_color>5) stat_led_color=0;
			switch(stat_led_color) {//127 was way too bright
				case 0: status_led(70,0,0);
				break;
				case 1: status_led(70,70,0);
				break;
				case 2: status_led(0,70,0);
				break;
				case 3: status_led(0,70,70);
				break;
				case 4: status_led(0,0,70);
				break;
				case 5: status_led(70,0,70);
				break;

			}
		}

		//output format: time_since_start=uptime, measurement_interval, temperature, 4*voltages, 16*(light intens, current, run_time)
		if(((time_now-time_last_measure)>=MEASURE_INTERVAL) || ll_meas_no==0){ //might want to disable first measurement, otherwise there will be an error in the total running time.
			time_last_measure=time_now;
			ll_meas_no++;
			//let's do a measurement
			status_led(0,0,0); //black light! dont want to influence the measurement, the ws led is bright!
			
			//update the ADC offset
			adc_offset_m = adc_get_offset();
			
			//get the time
			strcpy(csv_data_output, "ut:");
			ltoa(time_now, out_str, 10);
			strcat(csv_data_output, out_str);
			strcat(csv_data_output, ",");
			
			//print the measurement interval to add to light on time
			strcat(csv_data_output, "mi:");
			ltoa(MEASURE_INTERVAL, out_str, 10);
			strcat(csv_data_output, out_str);
			strcat(csv_data_output, ",");
			
			//measure the voltages and print them
			adc_read_voltages(ll_vdc_supply);
			dtostrf(ll_vdc_supply[0], 4,2,out_str);
			strcat(csv_data_output, "V1:");
			strcat(csv_data_output, out_str);
			strcat(csv_data_output, ",");
			
			dtostrf(ll_vdc_supply[1], 4,2,out_str);
			strcat(csv_data_output, "V2:");
			strcat(csv_data_output, out_str);
			strcat(csv_data_output, ",");
			
			dtostrf(ll_vdc_supply[2], 4,2,out_str);
			strcat(csv_data_output, "V3:");
			strcat(csv_data_output, out_str);
			strcat(csv_data_output, ",");
			
			dtostrf(ll_vdc_supply[3], 4,2,out_str);
			strcat(csv_data_output, "V4:");
			strcat(csv_data_output, out_str);
			strcat(csv_data_output, ",");
			
			//measure the ll_temperature and print
			DS18X20_start_meas( DS18X20_POWER_PARASITE, NULL );
			_delay_ms(800); //quite a lot of wait for this one.
			DS18X20_read_decicelsius_single( sens_id, &decicelsius );
			ll_temperature = decicelsius/10;
			strcat(csv_data_output, "T:");
			dtostrf(ll_temperature, 4,1,out_str);
			strcat(csv_data_output, out_str);

			//for each led, measure brightness and current
			leds_OFF();
			for(uint8_t i=1;i<=NUM_LEDS;i++){
				//for(uint8_t i=1;i<=2;i++){
				LED_ON(i);
				
				uint16_t lux=0;
				for(uint8_t j=0;j<LIGHT_AVG_NO;j++){
					_delay_ms(1000);
					lux += TLS2550_lux_calc_dec();
				}
				ll_lux_dec[i-1] = lux / LIGHT_AVG_NO;
				if((ll_lux_dec[i-1]>50) && (ll_meas_no>1)) ll_time_leds[i-1]+= MEASURE_INTERVAL; //write extra time only if there is some light read
				strcat(csv_data_output, ",");
				//L1
				strcat(csv_data_output, "L");
				itoa(i, out_str, 10);
				strcat(csv_data_output, out_str);
				strcat(csv_data_output, ":");
				itoa(ll_lux_dec[i-1], out_str, 10);
				strcat(csv_data_output, out_str);
				strcat(csv_data_output, ",");
				ll_idc_leds[i-1] = adc_read_current();
				//I1
				strcat(csv_data_output, "I");
				itoa(i, out_str, 10);
				strcat(csv_data_output, out_str);
				strcat(csv_data_output, ":");
				dtostrf(ll_idc_leds[i-1], 4,3,out_str);
				strcat(csv_data_output, out_str);
				
				strcat(csv_data_output, ",");
 				strcat(csv_data_output, "t");
 				itoa(i, out_str, 10);
 				strcat(csv_data_output, out_str);
 				strcat(csv_data_output, ":");
 				ltoa(ll_time_leds[i-1], out_str, 10);
 				strcat(csv_data_output, out_str);
				
				LED_OFF(i);
			}
			upload_electrobob_data();//send the data to the could
			
			strcat(csv_data_output, "\n");
			//for easier readout during devel
			strcat(csv_data_output, "\n");
			strcat(csv_data_output, "\n");
			
			leds_ON();
			LED_OFF(CONTROL_LED); //make sure control stays off
			
			//print the results
			debug.sendString(csv_data_output);
			//clear the buffer contents
			csv_data_output[0]='\0';
			csv_data_output[0]='\0';
			
			//turn on all leds
			leds_ON();
			
			//clear the buffer contents
			csv_data_output[0]='\0';
			ll_write_eeprom_times(); //save new data to eeprom
		}
	}
	
	return 0;
}



