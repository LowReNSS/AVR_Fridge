/*
 * DS18B20.c
 *
 * Created: 05.12.2022 18:48:39
 *  Author: Schumacher
 */ 

#define F_CPU 8000000UL

#include	<avr/io.h>
#include	<util/delay.h>

#define OW_DQ_PIN	PIND2
#define OW_DDR		DDRD
#define OW_PIN		PIND
#define OW_PORT		PORTD

#define DQ_SET_HI()			OW_DDR  &= ~(1 << OW_DQ_PIN); OW_PORT |= (1 << OW_DQ_PIN)
#define DQ_SET_LO()			OW_PORT &= ~(1 << OW_DQ_PIN); OW_DDR  |= (1 << OW_DQ_PIN)
	
#define DQ_SPU_ENABLE()		OW_DDR |= (1 << OW_DQ_PIN)	// Enable Strong Pull Up
	
#define DQ_SPU_DISABLE()	DQ_SET_HI()																// Disable Strong Pull Up
	
#define DQ_READ()			((OW_PIN & (1 << OW_DQ_PIN)) == (1 << OW_DQ_PIN))



uint8_t OW_reset(void)
{
	DQ_SET_HI();
	_delay_ms(100);
	
	/* Start reset sequence */
	DQ_SET_LO();
	_delay_us(480 + 10);
	DQ_SET_HI();
	_delay_us(50);
	uint8_t status = 0;
	if(DQ_READ() == 0)
	{
		status = 1;
		while(DQ_READ() == 0);
	}
	_delay_us(600);
	return status;
}


static inline void OW_send_byte(uint8_t byte)
{
	for(uint8_t bitMask = 1; bitMask; bitMask <<= 1)
	{
		_delay_us(10);
		if(byte & bitMask)
		{
			DQ_SET_LO();
			_delay_us(5);
			DQ_SET_HI();
			_delay_us(55);
		}
		else
		{
			DQ_SET_LO();
			_delay_us(60);
			DQ_SET_HI();
		}
	}
	/*if(byte == 0x44) DQ_SET_SPU();*/
}

static inline  uint8_t OW_read_byte(void)
{
	uint8_t byte = 0;
	
	for(uint8_t bitMask = 1; bitMask; bitMask <<= 1)
	{
		_delay_us(10);
		
		uint8_t bit = 0;
		DQ_SET_LO();
		_delay_us(5);
		DQ_SET_HI();
		_delay_us(10);
		if(DQ_READ() != 0) bit = 1;
		_delay_us(55);
		if(bit) byte |= bitMask;
	}
	return byte;
}

static inline uint8_t OW_CRC8_calc(uint8_t *data, uint8_t size)
{
	if(data == 0) return 255;
	uint8_t crc = 0;
	for(uint8_t i = 0; i < size; i++)
	{
		uint8_t value = data[i];
		uint8_t tmp = 0;
		for(uint8_t j = 0; j < 8; j++)
		{
			tmp = (crc ^ value) & 0x01;
			crc >>= 1;
			value >>= 1;
			if(tmp == 1) crc ^= 0x8C;
		}
	}
	
	return crc;
}


uint8_t DS18B20_search(uint64_t* addr, uint8_t size)
{
	if(size > 8) size = 8;
	
	for(uint8_t i = 0; i < size; i++) addr[i] = 0;
	
	
	uint8_t headCnt, tailCnt = 1;
	for(headCnt = 0; headCnt < tailCnt; headCnt++)
	{
		if(headCnt >= size) break;
		if(!OW_reset()) break; // No active devices found

		OW_send_byte(0xF0); // SEARCH ROM [F0h]

		uint64_t branch_a = 0;
		uint64_t branch_b = 0;
				
		for(uint64_t bitMask = 0x01; bitMask; bitMask <<= 1)
		{
			_delay_us(10);
			/*Read Bit A*/
			uint8_t bit_a = 0;
			DQ_SET_LO();
			_delay_us(5);
			DQ_SET_HI();
			_delay_us(10);
			if(DQ_READ() != 0) bit_a = 1;
			_delay_us(55);
					
			_delay_us(10);	
			/*Read Bit B*/
			uint8_t bit_b = 0;
			DQ_SET_LO();
			_delay_us(5);
			DQ_SET_HI();
			_delay_us(10);
			if(DQ_READ() != 0) bit_b = 1;
			_delay_us(55);
					
			if(bit_a && bit_b) break;

			uint8_t ans = bit_a;
			if((bit_a || bit_b) == 0)
			{
				if (addr[headCnt] & bitMask) ans = 1;
				else branch_b = branch_a | bitMask;
			}
					
			_delay_us(10);
			/*Send Ans*/
			if(ans)
			{
				DQ_SET_LO();
				_delay_us(5);
				DQ_SET_HI();
				_delay_us(55);
			}
			else
			{
				DQ_SET_LO();
				_delay_us(60);
				DQ_SET_HI();
			}
						
			if (ans == 1) branch_a |= bitMask;
		}

		uint8_t crc = OW_CRC8_calc((uint8_t*)&branch_a, 8);

		if (crc == 0) addr[headCnt] = branch_a;
		else break;

		if(branch_b != 0)
		{
			addr[tailCnt] = branch_b;
			tailCnt++;
		}
		_delay_ms(10);
	}
	return headCnt;
}

uint8_t DS18B20_get_raw_temp(uint64_t address, int16_t* temp)
{
	if(temp == 0) return 0;
	if(!OW_reset()) return 0;
	
	if(address == 0)
	{
		OW_send_byte(0xCC);	// SKIP ROM [CCh]
	}
	else
	{
		OW_send_byte(0x55);	// MATCH ROM [55h]
		uint8_t* ptr = (uint8_t*)&address;
		for(uint8_t i = 0; i < sizeof(address); i++)
		{
			OW_send_byte(ptr[i]);
		}
	
	}
	OW_send_byte(0x44);	// CONVERT T [44h]
	
	DQ_SPU_ENABLE();
	_delay_ms(750);
	DQ_SPU_DISABLE();

	
	
	if(!OW_reset()) return 0;

	if(address == 0)
	{
		OW_send_byte(0xCC);	// SKIP ROM [CCh]
	}
	else
	{
		OW_send_byte(0x55);	// MATCH ROM [55h]
		uint8_t* ptr = (uint8_t*)&address;
		for(uint8_t i = 0; i < sizeof(address); i++)
		{
			OW_send_byte(ptr[i]);
		}
	
	}
	OW_send_byte(0xBE);	// READ SCRATCHPAD [BEh]

	uint8_t data[9] = { 0 };

	uint8_t check = 0xFF;
	for(uint8_t i = 0; i < sizeof(data); i++)
	{
		data[i] = OW_read_byte();
		check &= data[i];
	}

	if(check == 0xFF) return 0;

	uint8_t crc = OW_CRC8_calc(data, sizeof(data));
	if(crc != 0) return 0;

	*temp = *(int16_t*)&data;
	
	return 1;
}

uint8_t DS18B20_get_temp(uint64_t address, float* temp)
{
	if(temp == 0) return 0;
	
	uint8_t status = 0;
	for(uint8_t t = 0; t < 3; t++) // Try receive data x3
	{
		int16_t rawTemp;
		if(DS18B20_get_raw_temp(address, &rawTemp) == 0) continue;
		
		if((rawTemp & 0xFFF) == 85)
		{
			if(DS18B20_get_raw_temp(address, &rawTemp) == 0) continue;
		}

		*temp = (rawTemp >> 4) + (float)(rawTemp & 0x0F) * 0.0625;
		status = 1;
		break;
	}
	
	return status;
}