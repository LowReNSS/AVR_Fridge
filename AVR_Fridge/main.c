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

void SSD1306_init(void);
void SSD1306_endl(void);
void SSD1306_clrl(void);
void SSD1306_home(void);
void SSD1306_print_char(uint8_t ch);
void SSD1306_print_int(uint16_t num);
void SSD1306_print_float(float num, uint8_t n);

uint8_t DS18B20_search(uint64_t* addr, uint8_t size);
uint8_t DS18B20_get_temp(uint64_t addr, float* temp);

#define		DOOR_SW_PIN		PIND3

#define		COMP_S1_PIN		PIND5
#define		COMP_S2_PIN		PIND6
#define		LAMP_LD_PIN		PIND7	//	INT1


uint64_t sens_addr_nofrost;
uint64_t sens_addr_freezer;



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


void lamp_on(void)
{
	PORTD |=  (1 << LAMP_LD_PIN);
}

void lamp_off(void)
{
	PORTD &= ~(1 << LAMP_LD_PIN);
}




void damper_open(void)
{ 
	DDRC  |=  ((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3));
	PORTC &= ~((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3));
	
	for(uint16_t i = 0; i < 420; i++)
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
	}
}


void damper_close(void)
{
	DDRC  |=  ((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3));
	PORTC &= ~((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3));
		
	for(uint16_t i = 0; i < 420; i++)
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
	}
}



volatile uint16_t sec_cnt = 0;

ISR(TIMER1_COMPA_vect)
{
	sec_cnt++;
}



ISR(INT1_vect)
{
	if(PIND & (1 << DOOR_SW_PIN))
	{
		lamp_on();
	}
	else
	{
		lamp_off();
	}
}



uint64_t sens_addr_nofrost = 0;
uint64_t sens_addr_freezer = 0;




uint16_t get_time_stamp(uint16_t sec_delay)
{
	if((sec_cnt +  sec_delay) > 32000) sec_cnt = 0;
	_delay_ms(1);
	if((sec_cnt +  sec_delay) > 32000) sec_cnt = 0;
	
	return (sec_cnt + sec_delay);
}




int main(void)
{
	// Initialize Damper Pins
	DDRC  |=  ((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3));
	
	// Initialize Compressor Switch Pins
	DDRD  |= (1 << COMP_S1_PIN);
	DDRD  |= (1 << COMP_S2_PIN);
	
	// Initialize Door Open/Close Detector
	DDRD  &= ~(1 << DOOR_SW_PIN);	// INT1 (Pin external interrupt)
	PORTD |=  (1 << DOOR_SW_PIN);
	GICR  |= (1 << INT1);	// External Interrupt Request 1 Enable
	MCUCR |= (1 << ISC10);	// Any logical change on INT1 generates an interrupt request
	
	
	// Initialize Lamp Switch Pins
	DDRD  |= (1 << LAMP_LD_PIN);
	if(PIND & (1 << DOOR_SW_PIN)) lamp_on();
	else lamp_off();
	
	_delay_ms(1000);
	
	SSD1306_init();

	SSD1306_home();
	SSD1306_print_int(sec_cnt); SSD1306_endl();
	
	// Initialize sys tick timer
	TCCR1A = 0;
	TCCR1B = (1 << WGM12); // CTC
	OCR1A = 31250 - 1;
	TCNT1 = 0;
	TIMSK = (1 << OCIE1A);
	TCCR1B |= (1 << CS12); // F_TIM = 31250; F_ISR_TIMER1_COMPA = 1;
	
	sei();
	uint16_t time_stamp = get_time_stamp(1);
	_delay_ms(1200);
	if(sec_cnt < time_stamp)
	{
		while(1);
	}
	

	time_stamp = get_time_stamp(60);	// 1 minute idle to protect the compressor from a short restart
	while(sec_cnt < time_stamp)
	{
		SSD1306_home();
		SSD1306_print_int(sec_cnt);
		SSD1306_endl();
		_delay_ms(100);
	}


	// Initialize temperature sensors
	sens_addr_nofrost = 33;
	sens_addr_freezer = 44;
	
	compressor_start();
	time_stamp = get_time_stamp(2400);	// start compressor to time <= 40 minute
	while(sec_cnt < time_stamp)
	{
		SSD1306_home();
		SSD1306_print_int(sec_cnt); 
		SSD1306_endl();
		
		uint64_t addresses[2];
		uint8_t n_found = DS18B20_search(addresses, 2);
		if(n_found != 2) continue;
		
		float temp_a;
		float temp_b;
		
		if(DS18B20_get_temp(addresses[0], &temp_a) == 0) continue;
		if(DS18B20_get_temp(addresses[1], &temp_b) == 0) continue;

		if((temp_a - temp_b) > 5)
		{
			sens_addr_nofrost = addresses[0];
			sens_addr_freezer = addresses[1];
			break;
		}
		if((temp_b - temp_a) > 5)
		{
			sens_addr_nofrost = addresses[1];
			sens_addr_freezer = addresses[0];
			break;
		}
	}
	


	volatile uint16_t time_work = 0;
	volatile uint8_t state = 1;
	
    while(1) 
    {			
		float temp_nofrost = -274.0;
		float temp_freezer = -274.0;
		DS18B20_get_temp(sens_addr_nofrost, &temp_nofrost);
		
		SSD1306_home();
		SSD1306_print_float(temp_nofrost, 3);	SSD1306_endl();
		
		
		DS18B20_get_temp(sens_addr_freezer, &temp_freezer);
		
		SSD1306_print_float(temp_freezer, 3);	SSD1306_endl();
		SSD1306_print_int(sec_cnt);				SSD1306_endl();
		SSD1306_print_int(time_work);			SSD1306_endl();
		
		

		if(state == 0) // check temp freezer camera
		{
			if((temp_freezer < -273.0) || (temp_freezer > -15.0)) // if sensor broken or temp hi - always start compressor;
			{
				compressor_start();
				time_stamp = get_time_stamp(1800);	// start compressor to time <= 30 minute;
				state = 1;
			}
			continue;
		}
		
		if(state == 1) // freeze the freezer camera to temp: -22.0*C;
		{
			if((sec_cnt > (time_stamp)) || ((temp_freezer > -273.0) && (temp_freezer < -22.0))) // if compressor work > 30 minute or temp < -22.0*C;
			{
				compressor_stop();
				time_work = time_stamp - (time_stamp - sec_cnt);
				
				time_stamp = get_time_stamp(600); // hold compressor to idle on time  10 minute
				state = 2;
			}
			continue;
		}
		
		if(state == 2) // hold compressor to idle on time  10 minute
		{
			if(sec_cnt > (time_stamp))
			{
				state = 0;
			}
			continue;
		}
		
		state = 0;

    }
}

