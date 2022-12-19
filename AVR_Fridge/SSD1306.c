/*
 * SSD1306.c
 *
 * Created: 05.12.2022 18:26:00
 *  Author: dytec
 */ 

#include	<avr/io.h>

#define		I2C_DDR			DDRC
#define		I2C_PIN			PINC
#define		I2C_PORT		PORTC

#define		I2C_SCL_PIN		PINC5
#define		I2C_SDA_PIN		PINC4


#define		I2C_SCL_set_hi()		I2C_DDR&=~(1<<I2C_SCL_PIN)
#define		I2C_SDA_set_hi()		I2C_DDR&=~(1<<I2C_SDA_PIN)

#define		I2C_SCL_set_lo()		I2C_DDR|=(1<<I2C_SCL_PIN)
#define		I2C_SDA_set_lo()		I2C_DDR|=(1<<I2C_SDA_PIN)


#define		SSD1306_I2C_ADDR		0b01111000
#define		SSD1306_SEND_BUF		0b01000000
#define		SSD1306_SEND_BYTE		0b11000000
#define		SSD1306_SEND_CMD		0b10000000


void SSD1306_print_char(uint8_t ch);


const uint8_t digits[10][8] =
{
	{ 0x00, 0xFF, 0xFF, 0xC3,  0xC3, 0xFF, 0xFF, 0x00 },	// 0
	{ 0x00, 0x00, 0x06, 0xFF,  0xFF, 0xFF, 0x00, 0x00 },	// 1
	{ 0x00, 0xFB, 0xFB, 0xDB,  0xDB, 0xDF, 0xDF, 0x00 },	// 2
	{ 0x00, 0xC3, 0xDB, 0xDB,  0xDB, 0xFF, 0xFF, 0x00 },	// 3
	{ 0x00, 0x1F, 0x1F, 0x18,  0x18, 0xFF, 0xFF, 0x00 },	// 4
	{ 0x00, 0xDF, 0xDF, 0xDB,  0xDB, 0xFB, 0xFB, 0x00 },	// 5
	{ 0x00, 0xFF, 0xFF, 0xDB,  0xDB, 0xFB, 0xFB, 0x00 },	// 6
	{ 0x00, 0x03, 0x03, 0x03,  0x03, 0xFF, 0xFF, 0x00 },	// 7
	{ 0x00, 0xFF, 0xFF, 0xDB,  0xDB, 0xFF, 0xFF, 0x00 },	// 8
	{ 0x00, 0xDF, 0xDF, 0xDB,  0xDB, 0xFF, 0xFF, 0x00 },	// 9
};

const uint8_t symbols[ ][8] =
{
	{ 0x00, 0xC0, 0xC0, 0x00,  0x00, 0x00, 0x00, 0x00 },	//  '.'
	{ 0x00, 0x18, 0x18, 0x18,  0x18, 0x18, 0x18, 0x00 },	//  '-'
	{ 0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00 },	//  ' '
	{ 0x00, 0x66, 0x66, 0x00,  0x00, 0x66, 0x66, 0x00 },	//  ':'

};

volatile uint8_t nrow = 0;
volatile uint8_t ncol = 0;



void I2C_start(void)
{
	I2C_PORT &= ~(1 << I2C_SCL_PIN);
	I2C_PORT &= ~(1 << I2C_SDA_PIN);
	
	I2C_SDA_set_hi();
	I2C_SCL_set_hi();
	I2C_SDA_set_lo();
	I2C_SCL_set_lo();
}


void I2C_stop(void)
{
	I2C_SDA_set_lo();
	I2C_SCL_set_hi();
	I2C_SDA_set_hi();
}


void I2C_send(uint8_t data)
{
	for(uint8_t k = 0x80; k; k >>= 1)
	{
		if(data & k)
		{
			I2C_SCL_set_lo();
			I2C_SDA_set_hi();
			I2C_SCL_set_hi();
			I2C_SCL_set_lo();
		}
		else
		{
			I2C_SCL_set_lo();
			I2C_SDA_set_lo();
			I2C_SCL_set_hi();
			I2C_SCL_set_lo();
		}
	}

	/* ACK */
	I2C_SDA_set_hi();
	I2C_SCL_set_hi();
	if(!(I2C_PIN & (1 << I2C_SDA_PIN)))
	I2C_SCL_set_lo();
}

void SSD1306_clear(void)
{
	for(uint8_t i = 0; i < 8; i++)
	{
		I2C_start();
		I2C_send(SSD1306_I2C_ADDR);

		I2C_send(SSD1306_SEND_CMD);
		I2C_send(0x21);		// Set Column Address
		I2C_send(SSD1306_SEND_CMD);
		I2C_send(0);
		I2C_send(SSD1306_SEND_CMD);
		I2C_send(127);

		I2C_send(SSD1306_SEND_CMD);
		I2C_send(0x22);		// Set Page Address
		I2C_send(SSD1306_SEND_CMD);
		I2C_send(i);
		I2C_send(SSD1306_SEND_CMD);
		I2C_send(i + 1);

		I2C_send(SSD1306_SEND_BUF);
		for(uint8_t j = 0; j < 128; j++)
		{
			I2C_send(0);
		}
		I2C_stop();
	}
}

void SSD1306_init(void)
{
	I2C_PORT &= ~(1 << I2C_SCL_PIN);
	I2C_PORT &= ~(1 << I2C_SDA_PIN);
	
	I2C_DDR	&= ~(1 << I2C_SCL_PIN);
	I2C_DDR	&= ~(1 << I2C_SDA_PIN);

	I2C_start();
	I2C_send(SSD1306_I2C_ADDR);

	const uint8_t init[18] = { 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0x1F, 0xA4, 0xA6, 0xD5, 0x80, 0x8D, 0x14, 0xAF };

	for(uint8_t k=0; k < 18; k++)
	{
		I2C_send(SSD1306_SEND_CMD);
		I2C_send(init[k]);
	}

	I2C_stop();
	SSD1306_clear();
}

void SSD1306_disp_char(uint8_t ch, uint8_t row, uint8_t col)
{

	I2C_start();
	I2C_send(SSD1306_I2C_ADDR);
	
	I2C_send(SSD1306_SEND_CMD);
	I2C_send(0x21);
	I2C_send(SSD1306_SEND_CMD);
	I2C_send(col * 8);
	I2C_send(SSD1306_SEND_CMD);
	I2C_send(col * 8 + 8);

	I2C_send(SSD1306_SEND_CMD);
	I2C_send(0x22);
	I2C_send(SSD1306_SEND_CMD);
	I2C_send(row);
	I2C_send(SSD1306_SEND_CMD);
	I2C_send(row + 1);

	I2C_send(SSD1306_SEND_BUF);

	if((ch >= '0') && (ch <= '9'))
	{
		uint8_t digit = ch - '0';
		for(uint8_t i = 0; i < 8; i++)
		{
			I2C_send(digits[digit][i]);
		}
	}
	else if(ch == '.')
	{
		for(uint8_t i = 0; i < 8; i++)
		{
			I2C_send(symbols[0][i]);
		}
	}
	else if(ch == '-')
	{
		for(uint8_t i = 0; i < 8; i++)
		{
			I2C_send(symbols[1][i]);
		}
	}
	else if(ch == ' ')
	{
		for(uint8_t i = 0; i < 8; i++)
		{
			I2C_send(symbols[2][i]);
		}
	}
	else if(ch == ':')
	{
		for(uint8_t i = 0; i < 8; i++)
		{
			I2C_send(symbols[3][i]);
		}
	}
	

	I2C_stop();
}

void SSD1306_endl(void)
{
	while(ncol < 14)
	{
		SSD1306_print_char(' ');
	}
	ncol = 0;
	nrow += 2;
}

void SSD1306_home(void)
{
	nrow = 0;
	ncol = 0;
}


// void SSD1306_clrl(void)
// {
// 	while(ncol < 14)
// 	{
// 		SSD1306_print_char(' ');
// 		ncol++;
// 	}
// 	ncol = 0;
// }


void SSD1306_print_char(uint8_t ch)
{
	if(ncol > 14)
	{
		SSD1306_endl();
	}
	
	if(nrow > 6)
	{
		nrow = ncol = 0;
		SSD1306_clear();
	}
	SSD1306_disp_char(ch, nrow, ncol);
	ncol++;
}

void SSD1306_print_int(int16_t num)
{
	if(num < 0)
	{
		SSD1306_print_char('-');
		num = -num;
	}

	if(num >= 10000) SSD1306_print_char('0' + (num / 10000) % 10);
	if(num >= 1000)  SSD1306_print_char('0' + (num / 1000) % 10);
	if(num >= 100)   SSD1306_print_char('0' + (num / 100) % 10);
	if(num >= 10)    SSD1306_print_char('0' + (num / 10) % 10);

	SSD1306_print_char('0' + (num / 1) % 10);
}


void SSD1306_print_float(float num, uint8_t n)
{
	SSD1306_print_int(num);
	
	SSD1306_print_char('.');

	if(num < 0) num = -num;
	
	num = num - ((uint16_t)num);
	if(n > 3) n = 3;
	while(n--)
	{
		num *= 10;
		if((uint16_t)num == 0) SSD1306_print_char('0');
	}
	
	if(num > 0)SSD1306_print_int(num+0.4);
	
}


