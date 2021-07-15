/**
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "nrf.h"
#include "urf_timer.h"
#include "urf_radio.h"
#include "urf_ble_peripheral.h"
#include "ble_const.h"

//openocd -f interface/stlink-v2.cfg -f target/nrf52.cfg
//flash write_image erase build/uplant.hex

float moisture_value = 13000;
int temperature = 100; //25*4
uint32_t lighting = 0; //not measured yet

float moisture_hist[6*50]; //1 point per 10 mins
int temp_hist[6*50]; //1 point per 10 mins
uint32_t light_hist[6*50]; //1 point per 10 mins
const int hist_len = 6*48;
int hist_vals_inited = 0;
uint32_t time_seconds = 0; //approximate time based on sleep interval
uint32_t prev_update_time = 0;

void push_hist_vals()
{
	if(!hist_vals_inited)
	{
		hist_vals_inited = 1;
		for(int x = 0; x < hist_len; x++)
		{
			moisture_hist[x] = 0;
			temp_hist[x] = 0;
			light_hist[x] = 0;
		}
	}
	if(time_seconds - prev_update_time > 600)
	{
		prev_update_time = time_seconds;
		for(int x = hist_len-1; x > 0; x--)
		{
			moisture_hist[x] = moisture_hist[x-1];
			temp_hist[x] = temp_hist[x-1];
			light_hist[x] = light_hist[x-1];
		}
	}
	moisture_hist[0] = moisture_value;
	temp_hist[0] = temperature;
	light_hist[0] = lighting;
}

volatile uint8_t in_sleep = 0;

void fast_clock_start()
{
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}
}
void slow_clock_start()
{
	NRF_CLOCK->LFCLKSRC = 0;
	NRF_CLOCK->TASKS_LFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {}
}
void fast_clock_stop()
{
	NRF_CLOCK->TASKS_HFCLKSTOP = 1;
}
void slow_clock_stop()
{
	NRF_CLOCK->TASKS_LFCLKSTOP = 1;
}
void mode_idle()
{
	NRF_POWER->TASKS_LOWPWR = 1;
	time_stop();
	schedule_event_stop();
	fast_clock_stop();
}
void mode_resume_idle()
{
	fast_clock_start();
	NRF_POWER->TASKS_CONSTLAT = 1;
	time_start();
}

void stop_rtc()
{
	NRF_RTC1->TASKS_STOP = 1;
}
void start_rtc()
{
	stop_rtc();
	NRF_RTC1->TASKS_CLEAR = 1;
	int sleep_time_s = 4;
	time_seconds += sleep_time_s;
	NRF_RTC1->CC[0] = 10*sleep_time_s; //10 cycles per 1 second
	NRF_RTC1->CC[1] = 0xFFFF;
	NRF_RTC1->CC[2] = 0xFFFF;
	NRF_RTC1->CC[3] = 0xFFFF;
	NRF_RTC1->PRESCALER = 3276;
	NRF_RTC1->INTENSET = (1<<16); //CC0 event
	NVIC_EnableIRQ(RTC1_IRQn);
	NRF_RTC1->TASKS_START = 1;
}
void RTC1_IRQHandler(void)
{
	/* Update compare counter */
	if (NRF_RTC1->EVENTS_COMPARE[0])
	{
		NRF_RTC1->EVENTS_COMPARE[0] = 0;
		NRF_RTC1->TASKS_CLEAR = 1;  // Clear Counter		
//		if(U32_delay_ms) U32_delay_ms--; // used in V_hw_delay_ms()
		__SEV();
//		stop_rtc();
		mode_resume_idle();
		in_sleep = 0;
	}
}

int sens_out_pin = 6;

void start_sens_out()
{
	NRF_TIMER1->MODE = 0;
	NRF_TIMER1->BITMODE = 1;
	NRF_TIMER1->PRESCALER = 0b0;
	NRF_TIMER1->CC[0] = 0b01;
	NRF_TIMER1->CC[1] = 0b01;
//	NRF_TIMER1->CC[0] = 4;
//	NRF_TIMER1->CC[1] = 4;
	NRF_TIMER1->SHORTS = 0b10; //clear on compare 1

	NRF_GPIO->DIRSET = 1<<sens_out_pin;
	NRF_GPIOTE->CONFIG[1] = (1<<20) | (0b11 << 16) | (sens_out_pin << 8) | 0b11; //toggle PIN_NUM<<8 (0) TASK

	NRF_PPI->CHENSET = 1<<5;
	NRF_PPI->CHG[0] |= 1<<5;
	NRF_PPI->CH[5].EEP = (uint32_t)&NRF_TIMER1->EVENTS_COMPARE[0];
	NRF_PPI->CH[5].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[1];

	NRF_TIMER1->TASKS_START = 1;
}

void stop_sens_out()
{
	NRF_TIMER1->TASKS_SHUTDOWN = 1;
	NRF_GPIO->OUTCLR = 1<<sens_out_pin;
}

typedef struct {

    union {
        uint32_t value;

        struct {
            unsigned RESP : 2;
            unsigned : 2;
            unsigned RESN : 2;
            unsigned : 2;
            unsigned GAIN : 3;
            unsigned : 1;
            unsigned REFSEL : 1;
            unsigned : 3;
            unsigned TACQ : 3;
			unsigned : 1;
			unsigned MODE : 1;
			unsigned : 3;
			unsigned BURST : 1;
        };
    };
} SAADC_CONFIG0_REG;


volatile uint32_t saadc_result = 0;

void measure_moisture_start()
{
	SAADC_CONFIG0_REG conf;
	conf.value = 0;
	conf.RESP = 0;
	conf.RESN = 0;
	conf.GAIN = 4; //1/2
	conf.REFSEL = 1; //use Vdd because we are sending reference signal at Vdd amplitude
	conf.TACQ = 4; //20 uS
	conf.MODE = 0; //single-ended
	conf.BURST = 1; //oversampling
	
	NRF_SAADC->CH[0].CONFIG = conf.value;

	// Configure the SAADC channel with VDD as positive input, no negative input(single ended).
	NRF_SAADC->CH[0].PSELP = 3; //AIN2
	NRF_SAADC->CH[0].PSELN = 0; //NC

	// Configure the SAADC resolution.
	NRF_SAADC->RESOLUTION = 3; //14 bit

	// Configure result to be put in RAM at the location of "result" variable.
	NRF_SAADC->RESULT.MAXCNT = 1;
	NRF_SAADC->RESULT.PTR = (uint32_t)&saadc_result;

	// No automatic sampling, will trigger with TASKS_SAMPLE.
	NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;

	// Enable SAADC (would capture analog pins if they were used in CH[0].PSELP)
	NRF_SAADC->ENABLE = 1;
	static int had_calibration = 0;
	if(!had_calibration)
	{
		had_calibration = 1;
		NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
		while(!NRF_SAADC->EVENTS_CALIBRATEDONE) ;
		NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
	}
// Convert the result to voltage
// Result = [V(p) - V(n)] * GAIN/REFERENCE * 2^(RESOLUTION)
// Result = (VDD - 0) * ((1/6) / 0.6) * 2^14
// VDD = Result / 4551.1
//  precise_result = (float)result / 4551.1f;
//  precise_result; // to get rid of set but not used warning

	// Stop the SAADC, since it's not used anymore.
//	NRF_SAADC->TASKS_STOP = 1;
//	while (NRF_SAADC->EVENTS_STOPPED == 0);
//	NRF_SAADC->EVENTS_STOPPED = 0;
//	NRF_SAADC->ENABLE = 0;
	return;
}

void measure_moisture_continue()
{
	NRF_SAADC->RESULT.MAXCNT = 1;
	NRF_SAADC->RESULT.PTR = (uint32_t)&saadc_result;
	NRF_SAADC->OVERSAMPLE = 8; //256x oversample
	NRF_SAADC->TASKS_START = 1;
	while (NRF_SAADC->EVENTS_STARTED == 0);
	NRF_SAADC->EVENTS_STARTED = 0;
	
	NRF_SAADC->TASKS_SAMPLE = 1;
	while (NRF_SAADC->EVENTS_RESULTDONE == 0);
	NRF_SAADC->EVENTS_RESULTDONE = 0;

	if(saadc_result < 16383-64)
	{
		moisture_value *= 0.9;
		moisture_value += 0.1*(float)saadc_result;
//		moisture_value = saadc_result;
	}

	
	push_hist_vals();
}

void measure_moisture_end()
{
	NRF_SAADC->TASKS_STOP = 1;
	while (NRF_SAADC->EVENTS_STOPPED == 0);
	NRF_SAADC->EVENTS_STOPPED = 0;
	NRF_SAADC->ENABLE = 0;	
}


void measure_light_prepare()
{
	//send power to power pin
	NRF_GPIO->DIRSET = 1<<14;
	NRF_GPIO->OUTSET = 1<<14;
}
void measure_light_disable()
{
	//send power to power pin
	NRF_GPIO->DIRSET = 1<<14;
	NRF_GPIO->OUTCLR = 1<<14;
}

typedef struct {

    union {
        uint8_t value;

        struct {
            unsigned shutdown : 1;
            unsigned manual_mode : 1;
            unsigned TRIG : 1;
            unsigned : 1;
            unsigned interval : 3;
            unsigned : 1;
        };
    };
} VEML_CONFIG_REG;

uint8_t veml_addr = 0x10;
volatile uint8_t vtx_buf[16];
volatile uint8_t vrx_buf[16];

VEML_CONFIG_REG veml_conf;

int measure_light_turnon()
{
	veml_conf.value = 0;
	veml_conf.shutdown = 0;
	veml_conf.manual_mode = 0;
	veml_conf.TRIG = 0;
	veml_conf.interval = 0; //0 - 40, 1 - 80, 2 - 160, 3 - 320, 4 - 640, 5 - 1280
	//for interval 0 maximum detected lux is 16k, less for longer - so no point in using them
	if(NRF_TWIM0->ENABLE != 6)
	{
		NRF_TWIM0->ADDRESS = veml_addr;
//		NRF_TWIM0->FREQUENCY = 0x01980000; //100k
		NRF_TWIM0->FREQUENCY = 0x04000000; //250k
		NRF_TWIM0->PSEL.SDA = 12;
		NRF_TWIM0->PSEL.SCL = 13;

		NRF_TWIM0->ENABLE = 6;
		
	}
	NRF_TWIM0->RXD.PTR = (uint32_t)vrx_buf;
	NRF_TWIM0->RXD.MAXCNT = 1;
	NRF_TWIM0->RXD.LIST = 0;
	vtx_buf[0] = 0; //config addr
	vtx_buf[1] = veml_conf.value;
	vtx_buf[2] = 0;
	NRF_TWIM0->TXD.PTR = (uint32_t)vtx_buf;
	NRF_TWIM0->TXD.LIST = 0;
	NRF_TWIM0->TXD.MAXCNT = 3;
	NRF_TWIM0->TASKS_STARTTX = 1;
	NRF_TWIM0->SHORTS = 0;
//	NRF_TWIM0->SHORTS = 1<<9; //lasttx->stop
	while(!(NRF_TWIM0->EVENTS_LASTTX || NRF_TWIM0->EVENTS_ERROR)) ;
	if(NRF_TWIM0->EVENTS_ERROR)
	{
		NRF_TWIM0->TASKS_STOP = 1;
		if(NRF_TWIM0->ERRORSRC & 0b010)
		{
			while(!NRF_TWIM0->EVENTS_STOPPED) ;
			delay_mcs(400);
//			NRF_TWIM0->ENABLE = 0;
			return 0;
		}
		if(NRF_TWIM0->ERRORSRC & 0b100)
		{
			return 0;
		}
	}
	NRF_TWIM0->TASKS_STOP = 1;
	while(!NRF_TWIM0->EVENTS_STOPPED) ;
	return 1;
}

int measure_light_start()
{
	if(NRF_TWIM0->ENABLE != 6)
		NRF_TWIM0->ENABLE = 6;
	veml_conf.value = 0;
	veml_conf.shutdown = 0;
	veml_conf.manual_mode = 0;
	veml_conf.TRIG = 1;
	veml_conf.interval = 0; //0 - 40, 1 - 80, 2 - 160, 3 - 320, 4 - 640, 5 - 1280
	//for interval 0 maximum detected lux is 16k, less for longer - so no point in using them
	NRF_TWIM0->RXD.PTR = (uint32_t)vrx_buf;
	NRF_TWIM0->RXD.MAXCNT = 1;
	NRF_TWIM0->RXD.LIST = 0;
	vtx_buf[0] = 0; //config addr
	vtx_buf[1] = veml_conf.value;
	vtx_buf[2] = 0;
	NRF_TWIM0->TXD.PTR = (uint32_t)vtx_buf;
	NRF_TWIM0->TXD.LIST = 0;
	NRF_TWIM0->TXD.MAXCNT = 3;
	NRF_TWIM0->TASKS_STARTTX = 1;
	NRF_TWIM0->SHORTS = 0;
	NRF_TWIM0->SHORTS = 1<<9; //lasttx->stop
	while(!(NRF_TWIM0->EVENTS_STOPPED || NRF_TWIM0->EVENTS_ERROR)) ;
	if(NRF_TWIM0->EVENTS_ERROR)
	{
		NRF_TWIM0->TASKS_STOP = 1;
	}
	return 0;
}

uint8_t encode_light(int val)
{ //0.25168
	if(val < 256) return val>>3;//0-32, 2.013 lux/unit, 0-62.4 lux
	if(val < 256+1024) return 32 + ((val-256)>>5); //32-64, 8.054 lux/unit, 66.4-316.06 lux
	if(val < 256+1024+4096) return 64 + ((val-256-1024)>>6); //64-128, 16.11 lux/unit, 324.1 - 1338.87 lux
	return 128 + ((val-256-1024-4096)>>9); //128-245, 128.86 lux/unit, 1403.3 - 16415.5 lux
}

void measure_light_read_end()
{
	int r_val, g_val, b_val;
//	NRF_TWIM0->ENABLE = 6;
//	delay_ms(1);
	for(int N = 0; N < 3; N++)
	{
		int err_v = 0;
		vtx_buf[0] = 0x08 + N; //R, G, B
		vtx_buf[1] = 0;
		NRF_TWIM0->TXD.PTR = (uint32_t)vtx_buf;
		NRF_TWIM0->TXD.LIST = 0;
		NRF_TWIM0->TXD.MAXCNT = 1;
		NRF_TWIM0->RXD.PTR = (uint32_t)vrx_buf;
		NRF_TWIM0->RXD.LIST = 0;
		NRF_TWIM0->RXD.MAXCNT = 2;
		NRF_TWIM0->EVENTS_STOPPED = 0;
		NRF_TWIM0->TASKS_STARTTX = 1;
		NRF_TWIM0->SHORTS = (1<<7) | (1<<12); //lasttx->startrx lastrx->stop
		while(!(NRF_TWIM0->EVENTS_STOPPED || NRF_TWIM0->EVENTS_ERROR));// || NRF_TWIM0->EVENTS_ERROR)) ;
		if(NRF_TWIM0->EVENTS_ERROR)
		{
			NRF_TWIM0->TASKS_STOP = 1;
			err_v = (N+1)*100;
		}
		if(err_v != 0)
		{
			if(N == 0) r_val = err_v;
			if(N == 1) g_val = err_v;
			if(N == 2) b_val = err_v;
		}
		else
		{
			if(N == 0) r_val = (vrx_buf[1]<<8) | vrx_buf[0];
			if(N == 1) g_val = (vrx_buf[1]<<8) | vrx_buf[0];
			if(N == 2) b_val = (vrx_buf[1]<<8) | vrx_buf[0];
		}
		delay_mcs(200);
	}
	uint8_t er = encode_light(r_val);
	uint8_t eg = encode_light(g_val);
	uint8_t eb = encode_light(b_val);
	lighting = er | (eg<<8) | (eb<<16);


/*	veml_conf.shutdown = 1;
	vtx_buf[0] = 0; //config addr
	vtx_buf[1] = veml_conf.value;
	NRF_TWIM0->TXD.PTR = vtx_buf;
	NRF_TWIM0->TXD.LIST = 0;
	NRF_TWIM0->TXD.MAXCNT = 2;
	NRF_TWIM0->TASKS_STARTTX = 1;
	NRF_TWIM0->SHORTS = 1<<9; //lasttx->stop
	while(!(NRF_TWIM0->EVENTS_STOPPED || NRF_TWIM0->EVENTS_ERROR)) ;*/
	NRF_TWIM0->ENABLE = 0;
}

void send_ble_adv_data()
{
	uint8_t pdu[40];
	uint8_t payload[40];
	payload[0] = 0x7A;
	payload[1] = 0xB1;
	payload[2] = NRF_FICR->DEVICEID[1];
	payload[3] = NRF_FICR->DEVICEID[1]>>8;
	payload[4] = NRF_FICR->DEVICEID[1]>>16;
	payload[5] = NRF_FICR->DEVICEID[1]>>24;
	payload[5] |= 0b11000000;

	int pp = 6;
	payload[pp++] = 0x02;
	payload[pp++] = 0x01; //flags
	payload[pp++] = 0b0110; //general discovery, BR/EDR not supported

	payload[pp] = 0;
	payload[pp+1] = 0x08;
	int len;
	static int cnt = 0;
	cnt++;
	int temp = NRF_TEMP->TEMP;
	temperature = temp;
	int moist = moisture_value;
	len = sprintf(payload + pp + 2, "uPlant %02X%02X", payload[2], payload[3]);// moist, (temp>>2), (temp%4)*25);//moisture_value);
	payload[pp] = len + 1;
	pp += len + 2;
	if(1)
	{
	payload[pp] = 0; //length determined later
	payload[pp+1] = 0xFF; //manufacturer's data
	int scnt = 2;
	static int hour_to_send = 2;
	static int color_to_send = 0;
	for(int h = 0; h < 3; h++)
	{
		int hpos = 0;
		if(h > 0)
		{
			hpos = 6*hour_to_send;
			hour_to_send ++;
			if(hour_to_send > 47) hour_to_send = 1;
		}
		int moist_enc = (moisture_hist[hpos] - 10500.0) / 12.0;
//		int moist_enc = (moisture_hist[hpos]) / 64.0;
		if(moist_enc < 0) moist_enc = 0;
		if(moist_enc > 255) moist_enc = 255;
		payload[pp+scnt] = moist_enc; scnt++;
		payload[pp+scnt] = temp_hist[hpos]; scnt++;
		if(color_to_send == 0) payload[pp+scnt] = light_hist[hpos]; 
		if(color_to_send == 1) payload[pp+scnt] = light_hist[hpos]>>8; 
		if(color_to_send == 2) payload[pp+scnt] = light_hist[hpos]>>16; 
		scnt++;
	}
	uint8_t col_mask = 0;
	if(color_to_send == 1) col_mask = 1<<6;
	if(color_to_send == 2) col_mask = 1<<7;
	payload[pp+scnt] = col_mask | hour_to_send; scnt++;
	payload[pp] = scnt-1;
	color_to_send++;
	if(color_to_send > 2) color_to_send = 0;
	}
	static int adv_ch = 37;
			
	len = ble_prepare_adv_pdu(pdu, 35, payload, BLE_ADV_NONCONN_IND_TYPE, 1, 1);
	for(int x = 0; x < 1; x++)
	{
		ble_LL_send_PDU(0x8E89BED6, len, pdu, adv_ch);
//		adv_ch++;
		if(adv_ch > 39) adv_ch = 37;
		if(x < -2) delay_ms(15);
/*		{
			uint32_t start_ms = millis();
			int err_cnt = 0;
			while(millis() - start_ms < 15 && err_cnt++ < 10000) //don't want to handle overflow, instead rely on err_cnt
				measure_moisture(); //takes ~50 uS
		}*/
		else delay_mcs(2000);
		if(ble_get_conn_state() != 0) break;
	}
	NRF_TEMP->TASKS_STOP = 1;
	
}

void set_ram_retention()
{
	NRF_POWER->RAM[0].POWER = 0b11;
	NRF_POWER->RAM[1].POWER = 0b11;
	NRF_POWER->RAM[2].POWER = 0b11;
	NRF_POWER->RAM[3].POWER = 0b11;
	NRF_POWER->RAM[4].POWER = 0b11;
	NRF_POWER->RAM[5].POWER = 0b11;
	NRF_POWER->RAM[6].POWER = 0b11;
	NRF_POWER->RAM[7].POWER = 0b11;
}

int main(void)
{
	while(0)
	{
		mode_idle();
		start_rtc();
		__SEV();
		__WFE();
		__WFE();
	} 
	
	fast_clock_start();
	slow_clock_start();
	time_start();
	NRF_POWER->DCDCEN = 1;
	NRF_UART0->ENABLE = 0; //from bootloader
	NRF_SPI0->ENABLE = 0;
	NRF_GPIO->PIN_CNF[12] = (6<<8) | 0b1100;
	NRF_GPIO->PIN_CNF[13] = (6<<8) | 0b1100;
	ble_init_radio();
	NRF_GPIO->DIRSET = 1<<16;
	set_ram_retention();
	measure_moisture_start();
	measure_light_prepare();
	NRF_GPIO->OUTSET = 1<<16;
	int has_light_sensor = 0;
	while(1)
	{
		if(measure_light_turnon())
		{
			has_light_sensor = 1;
			break;
		}
		has_light_sensor--;
		delay_ms(1);
		if(has_light_sensor < -20)
		{
			has_light_sensor = 0;
			break;
		}
//		veml_addr++;
		if(0)if(veml_addr == 255)
		{
			for(int x = 0; x < 10; x++)
			{
				NRF_GPIO->OUTSET = 1<<16;
				delay_ms(100);
				NRF_GPIO->OUTCLR = 1<<16;
				delay_ms(100);
				
			}
			break;
		}
	}
	NRF_GPIO->OUTCLR = 1<<16;
	while(1)
	{
		if(has_light_sensor) measure_light_prepare();
		measure_moisture_start();
		start_sens_out();
		NRF_TEMP->TASKS_START = 1;
		ble_init_radio();
		NRF_GPIO->OUTSET = 1<<16;
//		for(int n = 0; n < 40; n++)
		measure_moisture_continue(); //takes ~2.5 ms
//		delay_ms(2);
		if(has_light_sensor) measure_light_turnon();
		measure_moisture_continue();
//		delay_ms(1);
		if(has_light_sensor) measure_light_start();
//		while(!measure_light_start());
//		delay_ms(40);
		uint32_t light_start_ms = millis();
		send_ble_adv_data();
//		for(int n = 0; n < 40; n++)
		measure_moisture_continue();
//		delay_ms(2);
		ble_LL_radio_off();
		rf_disable();
		measure_moisture_end();
		stop_sens_out();
		NRF_RADIO->POWER = 0;
		NRF_RNG->TASKS_STOP = 1;
		NRF_GPIO->OUTCLR = 1<<16;

//		if(has_light_sensor) while(millis() - light_start_ms < 48) ; //wait for veml to complete
		if(has_light_sensor) measure_light_read_end();
		if(has_light_sensor) delay_ms(1);
		if(has_light_sensor) measure_light_start();
//		measure_light_disable();

		//sleep cycle
		mode_idle();
		start_rtc();
		__SEV();
		__WFE();
		__WFE();
	}

}

