// ATTiny13a AD9850 dds project
// Dmitry Ivanov

// -----------------------------------------------------------------------------
// Project config

// Freq config
#define F_CPU 1200000UL // 1.2 MHz
#define DDS_EX_CLK 125.0e6 // 125 MHz

// Pin config
#define LCD_I2C_CLK 0
#define LCD_I2C_DAT 1
#define DDS_CLK 4
#define DDS_UPD 3
#define DDS_DAT 1
// adc is on PB2
// for adc pin config look for adc_setup

// PCF8574 config
#define PCF8574_I2C_ADDRESS 0x27
#define LCD_DATA0_PIN		4
#define LCD_DATA1_PIN		5
#define LCD_DATA2_PIN		6
#define LCD_DATA3_PIN		7
#define LCD_RS_PIN			0
#define LCD_RW_PIN			1
#define LCD_E_PIN			2
#define LCD_LED_PIN			3

// -----------------------------------------------------------------------------
// Helpers

#include <util/delay.h>
#include <stdlib.h>

void __attribute__ ((noinline)) delay_16_ms()
{
	_delay_ms(16);
}

// -----------------------------------------------------------------------------
// Simple bit banging I2C for AVR, based on :
// https://github.com/t413/tinycopter/blob/master/i2c_bitbang.c
// http://codinglab.blogspot.com/2008/10/i2c-on-avr-using-bit-banging.html

#include <avr/io.h>

#define I2C_DATA_HI() (DDRB &= ~(1 << LCD_I2C_DAT))
#define I2C_DATA_LO() (DDRB |= (1 << LCD_I2C_DAT))

#define I2C_CLOCK_HI() (DDRB &= ~(1 << LCD_I2C_CLK))
#define I2C_CLOCK_LO() (DDRB |= (1 << LCD_I2C_CLK))

inline void I2C_WriteBit(unsigned char c)
{
	if(c > 0)
		I2C_DATA_HI();
	else
		I2C_DATA_LO();

	I2C_CLOCK_HI();
	I2C_CLOCK_LO();
}

inline void I2C_Stop()
{
	I2C_CLOCK_HI();
	I2C_DATA_HI();
}

inline void I2C_Init()
{
	PORTB &= ~(( 1 << LCD_I2C_DAT) | (1 << LCD_I2C_CLK));
	I2C_Stop();
}

inline void I2C_Start()
{
	DDRB &= ~(( 1 << LCD_I2C_DAT) | (1 << LCD_I2C_CLK));
	I2C_DATA_LO();
	I2C_CLOCK_LO();
}

void __attribute__ ((noinline)) I2C_Write(unsigned char c)
{
	for(char i = 0;i < 8; ++i)
	{
		I2C_WriteBit(c & 128);
		c <<= 1;
	}

	I2C_CLOCK_HI(); // ACK =)
	I2C_CLOCK_LO();
}

// -----------------------------------------------------------------------------
// Simple HD44780 + PCF8574 lib, based on
// http://davidegironi.blogspot.se/2013/06/an-avr-atmega-library-for-hd44780-based.html

void __attribute__ ((noinline)) lcd_pcf8574_write(uint8_t data)
{
	I2C_Start();
	I2C_Write(PCF8574_I2C_ADDRESS << 1 | 0);
	I2C_Write(data);
	I2C_Stop();
}

#define LCD_CLR               0      /* DB0: clear display                  */
#define LCD_HOME              1      /* DB1: return to home position        */
#define LCD_ENTRY_MODE        2      /* DB2: set entry mode                 */
#define LCD_ENTRY_INC         1      /* DB1: 1=increment, 0=decrement     */

#define LCD_DISP_OFF             0x08   /* display off                            */
#define LCD_DISP_ON              0x0C   /* display on, cursor off                 */

#define LCD_MODE_DEFAULT		((1<<LCD_ENTRY_MODE) | (1<<LCD_ENTRY_INC) )
#define LCD_FUNCTION_DEFAULT	0x28 /* 4-bit interface, dual line,   5x7 dots */

volatile uint8_t dataport = _BV(LCD_LED_PIN);

void __attribute__ ((noinline)) lcd_e_toggle()
{
	dataport |= _BV(LCD_E_PIN);
	lcd_pcf8574_write(dataport);
	dataport &= ~_BV(LCD_E_PIN);
	lcd_pcf8574_write(dataport);
}

void __attribute__ ((noinline)) lcd_write(uint8_t data,uint8_t rs)
{
	if (rs)
		dataport |= _BV(LCD_RS_PIN);
	else
		dataport &= ~_BV(LCD_RS_PIN);
	dataport &= ~_BV(LCD_RW_PIN);
	lcd_pcf8574_write(dataport);

	dataport &= ~(_BV(LCD_DATA3_PIN) | _BV(LCD_DATA2_PIN) | _BV(LCD_DATA1_PIN) | _BV(LCD_DATA0_PIN));
	if(data & 0x80) dataport |= _BV(LCD_DATA3_PIN);
	if(data & 0x40) dataport |= _BV(LCD_DATA2_PIN);
	if(data & 0x20) dataport |= _BV(LCD_DATA1_PIN);
	if(data & 0x10) dataport |= _BV(LCD_DATA0_PIN);
	lcd_pcf8574_write(dataport);
	lcd_e_toggle();

	dataport &= ~(_BV(LCD_DATA3_PIN) | _BV(LCD_DATA2_PIN) | _BV(LCD_DATA1_PIN) | _BV(LCD_DATA0_PIN));
	if(data & 0x08) dataport |= _BV(LCD_DATA3_PIN);
	if(data & 0x04) dataport |= _BV(LCD_DATA2_PIN);
	if(data & 0x02) dataport |= _BV(LCD_DATA1_PIN);
	if(data & 0x01) dataport |= _BV(LCD_DATA0_PIN);
	lcd_pcf8574_write(dataport);
	lcd_e_toggle();
}

void __attribute__ ((noinline)) lcd_command(uint8_t cmd)
{
	delay_16_ms();
	lcd_write(cmd, 0);
}

inline void lcd_clrscr(void)
{
	lcd_command(1 << LCD_CLR);
}

inline void lcd_home(void)
{
	lcd_command(1 << LCD_HOME);
}

void __attribute__ ((noinline)) lcd_putc(char c)
{
	lcd_write(c, 1);
}

inline void lcd_writeui16(uint16_t val)
{
	uint32_t i = 10000;
	for(; i ; i /= 10)
		lcd_putc('0' + (val / i) % 10);
}

void __attribute__ ((noinline)) lcd_e_toggle2()
{
	lcd_e_toggle();
	delay_16_ms();
}

inline void lcd_init()
{
	lcd_pcf8574_write(dataport);
	delay_16_ms();
	dataport |= _BV(LCD_DATA1_PIN) | _BV(LCD_DATA0_PIN);
	lcd_pcf8574_write(dataport);
	lcd_e_toggle2();
	lcd_e_toggle2();
	lcd_e_toggle2();
	dataport &= ~_BV(LCD_DATA0_PIN);
	lcd_pcf8574_write(dataport);
	lcd_e_toggle2();
	lcd_command(LCD_FUNCTION_DEFAULT);      /* function set: display lines  */
	lcd_command(LCD_MODE_DEFAULT);          /* set entry mode               */
	lcd_command(LCD_DISP_ON);               /* display/cursor control       */
}

// -----------------------------------------------------------------------------
// AD9850 DDS control

inline void dds_setfreq(uint32_t f)
{
	for (int i = 0; i < 32; i++, f >>= 1)
	{
		if(f & (uint32_t)1)
			DDRB &= ~(1 << DDS_DAT);
		else
			DDRB |= (1 << DDS_DAT);
		DDRB &= ~(1 << DDS_CLK);
		DDRB |= (1 << DDS_CLK);
	}
	DDRB |= (1 << DDS_DAT);
	for (int i = 0; i < 8; i++) {
		DDRB &= ~(1 << DDS_CLK);
		DDRB |= (1 << DDS_CLK);
	}
	DDRB &= ~(1 << DDS_UPD);
	DDRB |= (1 << DDS_UPD);
}

// -----------------------------------------------------------------------------
// ADC control

inline void adc_setup(void)
{
	ADMUX |= (1 << MUX0);
	ADMUX |= (1 << ADLAR); // ADC1, Pin 2
	ADCSRA |= (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN);
}

inline int16_t adc_read(void)
{
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADCH;
}

// -----------------------------------------------------------------------------
// Main function

int main(void)
{
	int16_t was = -2, read = 0;
	uint16_t mode = 0;

	delay_16_ms();
	adc_setup();
	delay_16_ms();

	I2C_Init();
	delay_16_ms();
	lcd_init();

	while(1)
	{
		while(abs(read - was) < 2) // standard abs is adding 2 bytes, so it's ok
		{
			read = adc_read();
			delay_16_ms();
		}
		was = read;

		if(read == 0)
			mode = 1 - mode;

		uint16_t freq = read;
		if(mode)
			freq *= 32;
		else
			freq *= 4;

		const double ref_k = 4294967296.0 / DDS_EX_CLK;
		const uint64_t ref_kd_hz = ref_k * (1ULL << 32) * 1;
		const uint64_t ref_kd_khz = ref_k * (1ULL << 32) * 1000;
		const uint64_t ref_kd_mhz = ref_k * (1ULL << 32) * 10000;
		uint64_t control_world_big = (uint64_t)freq;
		if(mode)
			control_world_big *= ref_kd_khz;
		else
			control_world_big *= ref_kd_hz;
		uint32_t control_world_small = ((uint32_t*)&control_world_big)[1];

		dds_setfreq(control_world_small);

		lcd_clrscr();
		lcd_home();
		lcd_writeui16(freq);
		lcd_putc(' ');
		if(mode)
			lcd_putc('K');
		lcd_putc('H');
		lcd_putc('z');
	}

	return 0;
}