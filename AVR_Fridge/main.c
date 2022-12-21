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

uint8_t DS18B20_search(uint64_t* addr, uint8_t size);
uint8_t DS18B20_get_temp(uint64_t addr, float* temp);


uint64_t sens_addr_nofrost;	// ROM address of DS18B20 sensor on nofrost section
uint64_t sens_addr_freezer;	// ROM address of DS18B20 sensor on freezer section


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

ISR(TIMER1_COMPA_vect)	//	sysTick
{
	sec_cnt++;
}

void reset_sysTick(void)
{
	sec_cnt = 0;
	_delay_us(10);
	sec_cnt = 0;
}


ISR(INT1_vect)
{
	if(PIND & (1 << DOOR_SW_PIN)) lamp_on();
	else lamp_off();
}



int main(void)
{
	// Initialize Damper Pins
	DDRC  |=  ((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3));
	
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
	
	SSD1306_init();


	time_stamp = sec_cnt;
	while(sec_cnt < (time_stamp + 30)) // 0.5 minute idle to protect the compressor from a short restart
	{
		SSD1306_home();
		SSD1306_print_int(sec_cnt);
		SSD1306_endl();
		_delay_ms(200);
	}


	// Initialize temperature sensors
	sens_addr_nofrost = 0xFFFFFF;
	sens_addr_freezer = 0xFFFFFF;
	
	compressor_start();
	time_stamp = sec_cnt;
	while(sec_cnt < (time_stamp + 1800)) // start compressor to time <= 30 minute
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
	

	damper_open();
	bool damper_is_open = true;


	uint8_t state = 0;
	
	uint16_t time_work = 0;
	uint16_t time_idle = 0;
	
		
    while(1) 
    {			
		float temp_nofrost = -274.0;
		float temp_freezer = -274.0;
		DS18B20_get_temp(sens_addr_nofrost, &temp_nofrost);
		
		SSD1306_home();
		SSD1306_print_float(temp_nofrost, 3);	SSD1306_endl();
		
		DS18B20_get_temp(sens_addr_freezer, &temp_freezer);
		
		SSD1306_print_float(temp_freezer, 3);	SSD1306_endl();

		SSD1306_print_int(time_work);			SSD1306_endl();
		SSD1306_print_int(time_idle);			SSD1306_endl();
		
		
		if(temp_nofrost < -273.0) // sensor error
		{
			if(damper_is_open == false)
			{
				damper_open();	// if sensor broken  always open damper;
				damper_is_open = true;
			}
		}
		else if((temp_nofrost < 2.0) && (damper_is_open == true))
		{
			damper_close();
			damper_is_open = false;
		}
		else if((temp_nofrost > 6.0) && (damper_is_open == false))
		{
			damper_open();
			damper_is_open = true;
		}
			
		if(state == 0) // freeze the freezer camera to temp: -22.0*C;
		{
			if((sec_cnt >= (time_stamp + 1800)) || ((temp_freezer < -22.0) && (temp_freezer > -273.0))) // if compressor work > 30 minute or temp < -22.0*C;
			{
				compressor_stop();
				time_work = sec_cnt - time_stamp;
				
				time_stamp = sec_cnt;
				state = 1;
			}
			continue;
		}
		
		if(state == 1)
		{
			if(sec_cnt >= (time_stamp + 600))  // hold compressor idle on time  10 minute
			{
				state = 2;
			}
			continue;
		}
		
		if(state == 2) // check temp freezer camera
		{
			if((temp_freezer < -273.0) || (temp_freezer > -15.0)) // if sensor broken or temp hi - always start compressor;
			{
				time_idle = sec_cnt - time_stamp;
				sec_cnt = 0;
				compressor_start();
				time_stamp = sec_cnt;
				state = 0;
			}
			continue;
		}
		
		state = 2;
    }
}

