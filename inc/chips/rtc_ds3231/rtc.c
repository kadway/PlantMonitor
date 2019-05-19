/*
 * DS RTC Library: DS1307 and DS3231 driver library
 * (C) 2011 Akafugu Corporation
 * (C) 2019 Joao Goncalves
 *
 * This program is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 */

/*
 *
 * DS3231 register map
 *
 *  00h-06h: seconds, minutes, hours, day-of-week, date, month, year (all in BCD)
 *       bit 7 should be set to zero: The DS3231 clock is always running
 *  07h: A1M1  Alarm 1 seconds
 *  08h: A1M2  Alarm 1 minutes
 *  09h: A1M3  Alarm 1 hour (bit6 is am/pm flag in 12h mode)
 *  0ah: A1M4  Alarm 1 day/date (bit6: 1 for day, 0 for date)
 *  0bh: A2M2  Alarm 2 minutes
 *  0ch: A2M3  Alarm 2 hour (bit6 is am/pm flag in 12h mode)
 *  0dh: A2M4  Alarm 2 day/data (bit6: 1 for day, 0 for date)
 *       <see data sheet page12 for Alarm register mask bit tables:
 *        for alarm when hours, minutes and seconds match set 1000 for alarm 1>
 *  0eh: control
 *      bit7: !EOSC
 *      bit6: BBSQW
 *      bit5: CONV
 *      bit4: RS2
 *      bit3: RS1
 *      bit2: INTCN
 *      bit1: A2IE
 *      bit0: A1IE
 *  0fh: control/status
 *      bit7: OSF
 *      bit6: 0
 *      bit5: 0
 *      bit4: 0
 *      bit3: EN32kHz
 *      bit2: BSY
 *      bit1: A2F alarm 2 flag
 *      bit0: A1F alarm 1 flag
 * 10h: aging offset (signed)
 * 11h: MSB of temp (signed)
 * 12h: LSB of temp in bits 7 and 6 (0.25 degrees for each 00, 01, 10, 11)
 *
 */


#define TRUE 1
#define FALSE 0

#include "rtc.h"

#define RTC_ADDR 0x68 // I2C address
#define CH_BIT 7 // clock halt bit

// statically allocated structure for time value
struct tm time;

uint8_t dec2bcd(uint8_t d)
{
  return ((d/10 * 16) + (d % 10));
}

uint8_t bcd2dec(uint8_t b)
{
  return ((b/16 * 10) + (b % 16));
}

uint8_t rtc_read_byte(uint8_t offset)
{
	uint8_t received_data;
	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, offset);
	I2C_stop(I2C1);

	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	received_data = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission
	return received_data;
}

void rtc_write_byte(uint8_t b, uint8_t offset)
{
	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, offset);
	I2C_write(I2C1, b);
	I2C_stop(I2C1);
}

static bool s_is_ds1307 = false;
static bool s_is_ds3231 = true;

void rtc_init(void)
{
	// Attempt autodetection:
	// 1) Read and save temperature register
	// 2) Write a value to temperature register
	// 3) Read back the value
	//   equal to the one written: DS1307, write back saved value and return
	//   different from written:   DS3231

	uint8_t temp1 = rtc_read_byte(0x11);
	uint8_t temp2 = rtc_read_byte(0x12);

	rtc_write_byte(0xee, 0x11);
	rtc_write_byte(0xdd, 0x12);

	if (rtc_read_byte(0x11) == 0xee && rtc_read_byte(0x12) == 0xdd) {
		s_is_ds1307 = true;
		// restore values
		rtc_write_byte(temp1, 0x11);
		rtc_write_byte(temp2, 0x12);
	}
	else {
		s_is_ds3231 = true;
	}
}

// Autodetection
bool rtc_is_ds1307(void) { return s_is_ds1307; }
bool rtc_is_ds3231(void) { return s_is_ds3231; }

// Autodetection override
void rtc_set_ds1307(void) { s_is_ds1307 = true;   s_is_ds3231 = false; }
void rtc_set_ds3231(void) { s_is_ds1307 = false;  s_is_ds3231 = true;  }

struct tm* rtc_get_time(void)
{
	uint8_t rtc[9];
	uint8_t century = 0;

	// read 7 bytes starting from register 0
	// sec, min, hour, day-of-week, date, month, year
	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, 0x0);
	I2C_stop(I2C1);

	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Receiver); // start a transmission in Master receiver mode

	for (uint8_t i = 0; i < 6; i++) {
		rtc[i] = I2C_read_ack(I2C1);// read one byte and request another byte
	}
	rtc[7] = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission


	// Clear clock halt bit from read data
	// This starts the clock for a DS1307, and has no effect for a DS3231
	//rtc[0] &= ~(_BV(CH_BIT)); // clear bit

	time.sec = bcd2dec(rtc[0]);
	time.min = bcd2dec(rtc[1]);
	time.hour = bcd2dec(rtc[2]);
	time.mday = bcd2dec(rtc[4]);
	time.mon = bcd2dec(rtc[5] & 0x1F); // returns 1-12
	century = (rtc[5] & 0x80) >> 7;
	time.year = century == 1 ? 2000 + bcd2dec(rtc[6]) : 1900 + bcd2dec(rtc[6]); // year 0-99
	time.wday = bcd2dec(rtc[3]); // returns 1-7

	if (time.hour == 0) {
		time.twelveHour = 0;
		time.am = 1;
	} else if (time.hour < 12) {
		time.twelveHour = time.hour;
		time.am = 1;
	} else {
		time.twelveHour = time.hour - 12;
		time.am = 0;
	}

	return &time;
}

void rtc_get_time_s(uint8_t* hour, uint8_t* min, uint8_t* sec)
{
	uint8_t rtc[9];

	// read 7 bytes starting from register 0
	// sec, min, hour, day-of-week, date, month, year
	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, 0x0);
	I2C_stop(I2C1);

	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Receiver); // start a transmission in Master receiver mode

	for (uint8_t i = 0; i < 6; i++) {
		rtc[i] = I2C_read_ack(I2C1);// read one byte and request another byte
	}
	rtc[7] = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission

	if (sec)  *sec =  bcd2dec(rtc[0]);
	if (min)  *min =  bcd2dec(rtc[1]);
	if (hour) *hour = bcd2dec(rtc[2]);
}

// fixme: support 12-hour mode for setting time
void rtc_set_time(struct tm* tm_)
{
	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, 0x0);

	uint8_t century;
	if (tm_->year > 2000) {
		century = 0x80;
		tm_->year = tm_->year - 2000;
	} else {
		century = 0;
		tm_->year = tm_->year - 1900;
	}

	// clock halt bit is 7th bit of seconds: this is always cleared to start the clock
	I2C_write(I2C1, dec2bcd(tm_->sec)); // seconds
	I2C_write(I2C1, dec2bcd(tm_->min)); // minutes
	I2C_write(I2C1, dec2bcd(tm_->hour)); // hours
	I2C_write(I2C1, dec2bcd(tm_->wday)); // day of week
	I2C_write(I2C1, dec2bcd(tm_->mday)); // day
	I2C_write(I2C1, dec2bcd(tm_->mon) + century); // month
	I2C_write(I2C1, dec2bcd(tm_->year)); // year

	I2C_stop(I2C1);
}

void rtc_set_time_s(uint8_t hour, uint8_t min, uint8_t sec)
{
	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, 0x0);

	// clock halt bit is 7th bit of seconds: this is always cleared to start the clock
	I2C_write(I2C1, dec2bcd(sec)); // seconds
	I2C_write(I2C1, dec2bcd(min)); // minutes
	I2C_write(I2C1, dec2bcd(hour)); // hours
	
	I2C_stop(I2C1);
}

void ds3231_get_temp_int(int8_t* i, uint8_t* f)
{
	uint8_t msb, lsb;
	
	*i = 0;
	*f = 0;
	
	if (s_is_ds1307) return; // only valid on DS3231

	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	// temp registers 0x11 and 0x12
	I2C_write(I2C1, 0x11);
	I2C_stop(I2C1);

	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Receiver); // start a transmission in Master receiver mode

	// integer part (in twos complement)
	msb = I2C_read_ack(I2C1);// read one byte and request another byte
	// fraction part
	lsb = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission

	// integer part in entire byte
	*i = msb;
	// fractional part in top two bits (increments of 0.25)
	*f = (lsb >> 6) * 25;

	// float value can be read like so:
	// float temp = ((((short)msb << 8) | (short)lsb) >> 6) / 4.0f;

}

void rtc_force_temp_conversion(uint8_t block)
{
	if (s_is_ds1307) return; // only valid on DS3231

	// read control register (0x0E)
	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, 0x0E);
	I2C_stop(I2C1);

	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	uint8_t ctrl = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission

	ctrl |= 0b00100000; // Set CONV bit

	// write new control register value
	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, 0x0E);
	I2C_write(I2C1, ctrl);
	I2C_stop(I2C1);

	if (!block) return;
	
	// Temp conversion is ready when control register becomes 0
	do {
		// Block until CONV is 0
		// write new control register value
		I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
		I2C_write(I2C1, 0x0E);
		I2C_stop(I2C1);

		I2C_start(I2C1, RTC_ADDR, I2C_Direction_Receiver); // start a transmission in Master receiver mode

	} while ((I2C_read_nack(I2C1) & 0b00100000) != 0);
}


//void rtc_SQW_enable(bool enable)
//{
//	if (s_is_ds1307) {
//		twi_begin_transmission(RTC_ADDR);
//		twi_send_byte(0x07);
//		twi_end_transmission();
//
//		// read control
//   		twi_request_from(RTC_ADDR, 1);
//		uint8_t control = twi_receive();
//
//		if (enable)
//			control |=  0b00010000; // set SQWE to 1
//		else
//			control &= ~0b00010000; // set SQWE to 0
//
//		// write control back
//		twi_begin_transmission(RTC_ADDR);
//		twi_send_byte(0x07);
//		twi_send_byte(control);
//		twi_end_transmission();
//
//	}
//	else { // DS3231
//		twi_begin_transmission(RTC_ADDR);
//		twi_send_byte(0x0E);
//		twi_end_transmission();
//
//		// read control
//   		twi_request_from(RTC_ADDR, 1);
//		uint8_t control = twi_receive();
//
//		if (enable) {
//			control |=  0b01000000; // set BBSQW to 1
//			control &= ~0b00000100; // set INTCN to 0
//		}
//		else {
//			control &= ~0b01000000; // set BBSQW to 0
//		}
//
//		// write control back
//		twi_begin_transmission(RTC_ADDR);
//		twi_send_byte(0x0E);
//		twi_send_byte(control);
//		twi_end_transmission();
//	}
//}
//
//void rtc_SQW_set_freq(enum RTC_SQW_FREQ freq)
//{
//	if (s_is_ds1307) {
//		twi_begin_transmission(RTC_ADDR);
//		twi_send_byte(0x07);
//		twi_end_transmission();
//
//		// read control (uses bits 0 and 1)
//   		twi_request_from(RTC_ADDR, 1);
//		uint8_t control = twi_receive();
//
//		control &= ~0b00000011; // Set to 0
//		control |= freq; // Set freq bitmask
//
//		// write control back
//		twi_begin_transmission(RTC_ADDR);
//		twi_send_byte(0x07);
//		twi_send_byte(control);
//		twi_end_transmission();
//
//	}
//	else { // DS3231
//		twi_begin_transmission(RTC_ADDR);
//		twi_send_byte(0x0E);
//		twi_end_transmission();
//
//		// read control (uses bits 3 and 4)
//   		twi_request_from(RTC_ADDR, 1);
//		uint8_t control = twi_receive();
//
//		control &= ~0b00011000; // Set to 0
//		control |= (freq << 4); // Set freq bitmask
//
//		// write control back
//		twi_begin_transmission(RTC_ADDR);
//		twi_send_byte(0x0E);
//		twi_send_byte(control);
//		twi_end_transmission();
//	}
//}
//
//void rtc_osc32kHz_enable(bool enable)
//{
//	if (!s_is_ds3231) return;
//
//	twi_begin_transmission(RTC_ADDR);
//	twi_send_byte(0x0F);
//	twi_end_transmission();
//
//	// read status
//	twi_request_from(RTC_ADDR, 1);
//	uint8_t status = twi_receive();
//
//	if (enable)
//		status |= 0b00001000; // set to 1
//	else
//		status &= ~0b00001000; // Set to 0
//
//	// write status back
//	twi_begin_transmission(RTC_ADDR);
//	twi_send_byte(0x0F);
//	twi_send_byte(status);
//	twi_end_transmission();
//}

// Alarm functionality

void rtc_reset_alarm(void)
{

	// writing 0 to bit 7 of all four alarm 1 registers disables alarm
	rtc_write_byte(0, 0x07); // second
	rtc_write_byte(0, 0x08); // minute
	rtc_write_byte(0, 0x09); // hour
	rtc_write_byte(0, 0x0a); // day

}

// fixme: add an option to set whether or not the INTCN and Interrupt Enable flag is set when setting the alarm
void rtc_set_alarm_s(uint8_t hour, uint8_t min, uint8_t sec)
{
	if (hour > 23) return;
	if (min > 59) return;
	if (sec > 59) return;

	/*
	 *  07h: A1M1:0  Alarm 1 seconds
	 *  08h: A1M2:0  Alarm 1 minutes
	 *  09h: A1M3:0  Alarm 1 hour (bit6 is am/pm flag in 12h mode)
	 *  0ah: A1M4:1  Alarm 1 day/date (bit6: 1 for day, 0 for date)
	 *  Sets alarm to fire when hour, minute and second matches
	 */
	rtc_write_byte(dec2bcd(sec),  0x07); // second
	rtc_write_byte(dec2bcd(min),  0x08); // minute
	rtc_write_byte(dec2bcd(hour), 0x09); // hour
	rtc_write_byte(0b10000001,         0x0a); // day (upper bit must be set)

	// clear alarm flag
	uint8_t val = rtc_read_byte(0x0f);
	rtc_write_byte(val & ~0b00000001, 0x0f);

}


void rtc_set_alarm(struct tm* tm_)
{
	if (!tm_) return;
	rtc_set_alarm_s(tm_->hour, tm_->min, tm_->sec);
}

void rtc_get_alarm_s(uint8_t* hour, uint8_t* min, uint8_t* sec)
{

	*sec  = bcd2dec(rtc_read_byte(0x07) & ~0b10000000);
	*min  = bcd2dec(rtc_read_byte(0x08) & ~0b10000000);
	*hour = bcd2dec(rtc_read_byte(0x09) & ~0b10000000);

}

struct tm* rtc_get_alarm(void)
{
	uint8_t hour, min, sec;

	rtc_get_alarm_s(&hour, &min, &sec);
	time.hour = hour;
	time.min = min;
	time.sec = sec;
	return &time;
}

bool rtc_check_alarm(void)
{

	// Alarm 1 flag (A1F) in bit 0
	uint8_t val = rtc_read_byte(0x0f);

	// clear flag when set
	if (val & 1)
		rtc_write_byte(val & ~0b00000001, 0x0f);

	return val & 1 ? 1 : 0;

}
