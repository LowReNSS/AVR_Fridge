/*
 * ATmega_Fridge.c
 *
 * Created: 11.12.2022 9:29:51
 * Author : Schumacher
 */ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#define		DOOR_SW_PIN		PIND3	//	Door Open/Close interrupt pin ISR(INT1_Vect)
#define		COMP_S1_PIN		PIND5	//	Compressor Relay On/Off Switch pin
#define		COMP_S2_PIN		PIND6	//	Compressor Relay On/Off Switch pin
#define		LAMP_LD_PIN		PIND7	//	Led Lamp On/Off Switch pin

#define		lamp_on()		PORTD |=  (1 << LAMP_LD_PIN)
#define		lamp_off()		PORTD &= ~(1 << LAMP_LD_PIN)






void SSD1306_init(void);
void SSD1306_endl(void);
void SSD1306_clrl(void);
void SSD1306_home(void);
void SSD1306_print_char(uint8_t ch);
void SSD1306_print_int(uint16_t num);
void SSD1306_print_float(float num, uint8_t n);

uint8_t DS18B20_search_sensors(uint64_t* addr, uint8_t size);
uint8_t DS18B20_start_measurement(uint8_t* address);
uint8_t DS18B20_get_measurement(uint8_t* address, float* temp);


uint64_t sens_addr_nofrost;	// ROM address of DS18B20 sensor on nofrost section
uint64_t sens_addr_freezer;	// ROM address of DS18B20 sensor on freezer section

float sens_temp_nofrost = -274.0;
float sens_temp_freezer = -274.0;

volatile uint16_t time_work = 0;
volatile uint16_t time_idle = 0;

volatile uint16_t sec_cnt = 0;

ISR(TIMER1_COMPA_vect)	//	sysTick
{
	sec_cnt++;
}

ISR(INT1_vect)
{
	if(PIND & (1 << DOOR_SW_PIN)) lamp_on();
	else lamp_off();
}


void compressor_start(void)
{
	PORTD |= (1 << COMP_S1_PIN);
	PORTD |= (1 << COMP_S2_PIN);
	_delay_ms(1);
	PORTD &= ~(1 << COMP_S1_PIN);
}

void compressor_stop(void)
{
	PORTD &= ~(1 << COMP_S1_PIN);
	PORTD &= ~(1 << COMP_S2_PIN);
}



void DS18B20_init(void)
{
	// Initialize temperature sensors
	sens_addr_nofrost = 0xFFFFFF;
	sens_addr_freezer = 0xFFFFFF;
	
	compressor_start();
	uint16_t time_stamp = sec_cnt;
	while(sec_cnt < (time_stamp + 1200)) // start compressor to time <= 20 minute
	{
		uint64_t addresses[2];
		float measurements[2] = { 0 };
		
		uint8_t n_found = DS18B20_search_sensors(addresses, 2);
		if(n_found != 2) continue;
		

		DS18B20_start_measurement((uint8_t*)&addresses[0]);
		_delay_ms(750);
		DS18B20_get_measurement((uint8_t*)&addresses[0], &measurements[0]);
		
		
		DS18B20_start_measurement((uint8_t*)&addresses[1]);
		_delay_ms(750);
		DS18B20_get_measurement((uint8_t*)&addresses[1], &measurements[1]);
		
		if((measurements[0] - measurements[1]) > 5)
		{
			sens_addr_nofrost = addresses[0];
			sens_addr_freezer = addresses[1];
			break;
		}
		if((measurements[1] - measurements[0]) > 5)
		{
			sens_addr_nofrost = addresses[1];
			sens_addr_freezer = addresses[0];
			break;
		}
	}
}


void DS18B20_loop(void)
{
	static uint8_t	state = 0;
	
	if(state == 0)
	{
		DS18B20_start_measurement((uint8_t*)&sens_addr_nofrost);
		state = 1;
		return;
	}
	
	if(state == 1)
	{
		sens_temp_nofrost = -274;
		DS18B20_get_measurement((uint8_t*)&sens_addr_nofrost, &sens_temp_nofrost);
		state = 2;
		return;
	}
	
	if(state == 2)
	{
		DS18B20_start_measurement((uint8_t*)&sens_addr_freezer);
		state = 3;
		return;
	}
	
	if(state == 3)
	{
		sens_temp_freezer = -274;
		DS18B20_get_measurement((uint8_t*)&sens_addr_freezer, &sens_temp_freezer);
		state = 0;
		return;
	}
	
	state = 0;
}


void SSD1306_loop(void)
{
	SSD1306_home();
	SSD1306_print_float(sens_temp_nofrost, 3);	SSD1306_endl();
	SSD1306_print_float(sens_temp_freezer, 3);	SSD1306_endl();
	SSD1306_print_int(time_work);	SSD1306_endl();
	SSD1306_print_int(time_idle);	SSD1306_endl();
}


void compressor_loop(void)
{
	static uint16_t time_stamp = 0;
	static uint8_t state = 0;
	
	if(state == 0)
	{
		if(sec_cnt >= 43200) sec_cnt = 0;
		// if sensor broken or temp hi - always start compressor;
		if(sens_temp_freezer > -16.0)		state = 1;
		if(sens_temp_freezer < -273.0)		state = 1;
		return;
	}else
	if(state == 1)
	{
		time_idle = sec_cnt - time_stamp;
		time_stamp = sec_cnt;
		compressor_start();
		state = 2;
		return;
	} else
	if(state == 2)
	{
		// if compressor work > 20 minute or temp < -22.0*C;
		if(sec_cnt >= (time_stamp + 1200))	state = 3;
		if(sens_temp_freezer < -273.0)		return;
		if(sens_temp_freezer < -22.0)		state = 3;
		return;
	} else
	if(state == 3)
	{
		time_work = sec_cnt - time_stamp;
		time_stamp = sec_cnt;
		compressor_stop();
		state = 4;
		return;
	}
	if(state == 4)
	{
		// hold compressor idle on time  10 minute
		if(sec_cnt >= (time_stamp + 600)) state = 0;
		return;
	}
	
	state = 0;
}

void damper_loop(void)
{
	static uint16_t damper_pos_tgt = 0; 
	static uint16_t damper_pos_now = 0; 
	
	if(sens_temp_nofrost > 4.0)		damper_pos_tgt = 420;
	if(sens_temp_nofrost < 2.0)		damper_pos_tgt = 0;
	if(sens_temp_nofrost < -273.0)	damper_pos_tgt = 420;

	if(damper_pos_now > damper_pos_tgt)	// if damper open and need close it
	{
		PORTC |= (1 << PINC3);
		_delay_ms(3);
		PORTC &= ~((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3));
		PORTC |= (1 << PINC1);
		_delay_ms(3);
		PORTC &= ~((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3));
		PORTC |= (1 << PINC2);
		_delay_ms(3);
		PORTC &= ~((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3));
		PORTC |= (1 << PINC0);
		_delay_ms(3);
		PORTC &= ~((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3));
		if(damper_pos_now > 0) damper_pos_now--;
	}
	
	if(damper_pos_now < damper_pos_tgt) // if damper close and need open it
	{
		PORTC |= (1 << PINC0);
		_delay_ms(3);
		PORTC &= ~((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3));
		PORTC |= (1 << PINC2);
		_delay_ms(3);
		PORTC &= ~((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3));
		PORTC |= (1 << PINC1);
		_delay_ms(3);
		PORTC &= ~((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3));
		PORTC |= (1 << PINC3);
		_delay_ms(3);
		PORTC &= ~((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3));
		if(damper_pos_now < 420) damper_pos_now++;
	}	
	
}



int main(void)
{
	// Initialize Damper Pins
	DDRC  |=  ((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3));
	PORTC &= ~((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3));
	
	// Initialize Compressor On/Off Switch Pins
	DDRD  |= (1 << COMP_S1_PIN);
	DDRD  |= (1 << COMP_S2_PIN);
	
	// Initialize Door Open/Close Detector
	DDRD  &= ~(1 << DOOR_SW_PIN);	// INT1 (Pin external interrupt)
	PORTD |=  (1 << DOOR_SW_PIN);
	GICR  |= (1 << INT1);	// External Interrupt Request 1 Enable
	MCUCR |= (1 << ISC10);	// Any logical change on INT1 generates an interrupt request
	
	
	// Initialize Lamp On/Off Switch Pins
	DDRD  |= (1 << LAMP_LD_PIN);
	if(PIND & (1 << DOOR_SW_PIN)) lamp_on();
	else lamp_off();

	
	// Initialize sys tick timer
	TCCR1A = 0;
	TCCR1B = (1 << WGM12); // CTC
	OCR1A = 31250 - 1;
	TCNT1 = 0;
	TIMSK = (1 << OCIE1A);
	TCCR1B |= (1 << CS12); // F_TIM = 31250; F_ISR_TIMER1_COMPA = 1;
	
	sei();
	uint16_t time_stamp = sec_cnt;
	_delay_ms(1100);
	if(sec_cnt == time_stamp)
	{
		while(1);
	}
	
	DS18B20_init();
	SSD1306_init();

    while(1) 
    {
		DS18B20_loop();
		SSD1306_loop();
		compressor_loop();
		damper_loop();
	}

}

