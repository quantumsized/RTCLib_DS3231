// Code by JeeLabs http://news.jeelabs.org/code/
// and modified by Quantumsized 
// Released to the public domain! Enjoy!

#include "DS3231.h"
#include <Wire.h>
#include "RTCLib.h"

// Include hardware-specific functions for the correct MCU
#if defined(__AVR__)
	#include "hardware/avr/HW_AVR.h"
#elif defined(__PIC32MX__)
  #include "hardware/pic32/HW_PIC32.h"
#elif defined(__arm__)
	#include "hardware/arm/HW_ARM.h"
#endif

#define DS3231_I2C_ADDRESS 0x68
#define REG_SEC		0x00
#define REG_MIN		0x01
#define REG_HOUR	0x02
#define REG_DOW		0x03
#define REG_DATE	0x04
#define REG_MON		0x05
#define REG_YEAR	0x06
#define DS3231_ALARM1SEC  0x07
#define DS3231_ALARM1MIN    0x08
#define DS3231_ALARM1HOU  0x09
#define DS3231_ALARM1DAY    0x0a
#define DS3231_ALARM2MIN    0x0b
#define DS3231_ALARM2HOU  0x0c
#define DS3231_ALARM2DAY    0x0d
#define REG_CON		0x0e
#define REG_STATUS	0x0f
#define REG_AGING	0x10
#define REG_TEMPM	0x11
#define REG_TEMPL	0x12
#define SECONDS_PER_DAY 86400L

#define SECONDS_FROM_1970_TO_2000 946684800

#if (ARDUINO >= 100)
 #include <Arduino.h> // capital A so it is error prone on case-sensitive filesystems
 // Macro to deal with the difference in I2C write functions from old and new Arduino versions.
 #define _I2C_WRITE write
 #define _I2C_READ  read
#else
 #include <WProgram.h>
 #define _I2C_WRITE send
 #define _I2C_READ  receive
#endif

////////////////////////////////////////////////////////////////////////////////
// utility code, some of this could be exposed in the DateTime API if needed

const uint8_t daysInMonth[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
    if (y >= 2000)
        y -= 2000;
    uint16_t days = d;
    for (uint8_t i = 1; i < m; ++i)
        days += pgm_read_byte(daysInMonth + i - 1);
    if (m > 2 && y % 4 == 0)
        ++days;
    return days + 365 * y + (y + 3) / 4 - 1;
}

static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
    return ((days * 24L + h) * 60 + m) * 60 + s;
}

////////////////////////////////////////////////////////////////////////////////
// DateTime implementation - ignores time zones and DST changes
// NOTE: also ignores leap seconds, see http://en.wikipedia.org/wiki/Leap_second

DateTime::DateTime (uint32_t t) {
  t -= SECONDS_FROM_1970_TO_2000;    // bring to 2000 timestamp from 1970

    ss = t % 60;
    t /= 60;
    mm = t % 60;
    t /= 60;
    hh = t % 24;
    uint16_t days = t / 24;
    uint8_t leap;
    for (yOff = 0; ; ++yOff) {
        leap = yOff % 4 == 0;
        if (days < 365 + leap)
            break;
        days -= 365 + leap;
    }
    for (m = 1; ; ++m) {
        uint8_t daysPerMonth = pgm_read_byte(daysInMonth + m - 1);
        if (leap && m == 2)
            ++daysPerMonth;
        if (days < daysPerMonth)
            break;
        days -= daysPerMonth;
    }
    d = days + 1;
}

DateTime::DateTime (uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
    if (year >= 2000)
        year -= 2000;
    yOff = year;
    m = month;
    d = day;
    hh = hour;
    mm = min;
    ss = sec;
}

DateTime::DateTime (const DateTime& copy):
  yOff(copy.yOff),
  m(copy.m),
  d(copy.d),
  hh(copy.hh),
  mm(copy.mm),
  ss(copy.ss)
{}

static uint8_t conv2d(const char* p) {
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
}

// A convenient constructor for using "the compiler's time":
//   DateTime now (__DATE__, __TIME__);
// NOTE: using F() would further reduce the RAM footprint, see below.
DateTime::DateTime (const char* date, const char* time) {
    // sample input: date = "Dec 26 2009", time = "12:34:56"
    yOff = conv2d(date + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec 
    switch (date[0]) {
        case 'J': m = date[1] == 'a' ? 1 : m = date[2] == 'n' ? 6 : 7; break;
        case 'F': m = 2; break;
        case 'A': m = date[2] == 'r' ? 4 : 8; break;
        case 'M': m = date[2] == 'r' ? 3 : 5; break;
        case 'S': m = 9; break;
        case 'O': m = 10; break;
        case 'N': m = 11; break;
        case 'D': m = 12; break;
    }
    d = conv2d(date + 4);
    hh = conv2d(time);
    mm = conv2d(time + 3);
    ss = conv2d(time + 6);
}

// A convenient constructor for using "the compiler's time":
// This version will save RAM by using PROGMEM to store it by using the F macro.
//   DateTime now (F(__DATE__), F(__TIME__));
DateTime::DateTime (const __FlashStringHelper* date, const __FlashStringHelper* time) {
    // sample input: date = "Dec 26 2009", time = "12:34:56"
    char buff[11];
    memcpy_P(buff, date, 11);
    yOff = conv2d(buff + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    switch (buff[0]) {
        case 'J': m = buff[1] == 'a' ? 1 : m = buff[2] == 'n' ? 6 : 7; break;
        case 'F': m = 2; break;
        case 'A': m = buff[2] == 'r' ? 4 : 8; break;
        case 'M': m = buff[2] == 'r' ? 3 : 5; break;
        case 'S': m = 9; break;
        case 'O': m = 10; break;
        case 'N': m = 11; break;
        case 'D': m = 12; break;
    }
    d = conv2d(buff + 4);
    memcpy_P(buff, time, 8);
    hh = conv2d(buff);
    mm = conv2d(buff + 3);
    ss = conv2d(buff + 6);
}

uint8_t DateTime::dayOfWeek() const {    
    uint16_t day = date2days(yOff, m, d);
    return (day + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
}

uint32_t DateTime::unixtime(void) const {
  uint32_t t;
  uint16_t days = date2days(yOff, m, d);
  t = time2long(days, hh, mm, ss);
  t += SECONDS_FROM_1970_TO_2000;  // seconds from 1970 to 2000

  return t;
}

long DateTime::secondstime(void) const {
  long t;
  uint16_t days = date2days(yOff, m, d);
  t = time2long(days, hh, mm, ss);
  return t;
}

DateTime DateTime::operator+(const TimeSpan& span) {
  return DateTime(unixtime()+span.totalseconds());
}

DateTime DateTime::operator-(const TimeSpan& span) {
  return DateTime(unixtime()-span.totalseconds());
}

TimeSpan DateTime::operator-(const DateTime& right) {
  return TimeSpan(unixtime()-right.unixtime());
}

////////////////////////////////////////////////////////////////////////////////
// TimeSpan implementation

TimeSpan::TimeSpan (int32_t seconds):
  _seconds(seconds)
{}

TimeSpan::TimeSpan (int16_t days, int8_t hours, int8_t minutes, int8_t seconds):
  _seconds(days*86400L + hours*3600 + minutes*60 + seconds)
{}

TimeSpan::TimeSpan (const TimeSpan& copy):
  _seconds(copy._seconds)
{}

TimeSpan TimeSpan::operator+(const TimeSpan& right) {
  return TimeSpan(_seconds+right._seconds);
}

TimeSpan TimeSpan::operator-(const TimeSpan& right) {
  return TimeSpan(_seconds-right._seconds);
}

////////////////////////////////////////////////////////////////////////////////
// RTC_DS3231 implementation changing to

static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }

/*uint8_t RTC_DS3231::begin(void) {
  return 1;
}*/
//May need this insted of the above
RTC_DS3231::DS3231(uint8_t data_pin, uint8_t sclk_pin) {
	_sda_pin = data_pin;
	_scl_pin = sclk_pin;
	return 1;
}

uint8_t RTC_DS3231::isrunning(void) {
  WIRE.beginTransmission(DS3231_I2C_ADDRESS);
  WIRE.WRITE(0);
  WIRE.endTransmission();

  WIRE.requestFrom(DS3231_I2C_ADDRESS, 1);
  uint8_t ss = WIRE.READ();
  return !(ss>>7);
}

void RTC_DS3231::adjust(const DateTime& dt) {
	Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0);
  Wire.write(decToBcd(dt.second()));
  Wire.write(decToBcd(dt.minute()));
  Wire.write(decToBcd(dt.hour()));
  Wire.write(decToBcd(dt.dayOfWeek()));
  Wire.write(decToBcd(dt.day));
  Wire.write(decToBcd(dt.month()));
  Wire.write(bin2bcd(dt.year() - 2000));
  WIRE.endTransmission();
	/* original
  WIRE.beginTransmission(DS1307_ADDRESS);
  WIRE._I2C_WRITE(0);
  WIRE._I2C_WRITE(bin2bcd(dt.second()));
  WIRE._I2C_WRITE(bin2bcd(dt.minute()));
  WIRE._I2C_WRITE(bin2bcd(dt.hour()));
  WIRE._I2C_WRITE(bin2bcd(dayOfWeek));
  WIRE._I2C_WRITE(bin2bcd(dt.day()));
  WIRE._I2C_WRITE(bin2bcd(dt.month()));
  WIRE._I2C_WRITE(bin2bcd(dt.year() - 2000));
  WIRE._I2C_WRITE(0);
  WIRE.endTransmission();*/
}

DateTime RTC_DS3231::now() {
	Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  uint8_t ss = bcd2bin(Wire.read() & 0x7f);
  uint8_t mm = bcd2bin(Wire.read());
  uint8_t hh = bcd2bin(Wire.read() & 0x3f);
  uint8_t dow = bcd2bin(Wire.read());
  uint8_t d = bcd2bin(Wire.read());
  uint8_t m = bcd2bin(Wire.read());
  uint16_t y = bcd2bin(Wire.read()) + 2000;
	
	/* original
  WIRE.beginTransmission(DS1307_ADDRESS);
  WIRE._I2C_WRITE(0);	
  WIRE.endTransmission();

  WIRE.requestFrom(DS1307_ADDRESS, 7);
  uint8_t ss = bcd2bin(WIRE._I2C_READ() & 0x7F);
  uint8_t mm = bcd2bin(WIRE._I2C_READ());
  uint8_t hh = bcd2bin(WIRE._I2C_READ());
  WIRE._I2C_READ();
  uint8_t d = bcd2bin(WIRE._I2C_READ());
  uint8_t m = bcd2bin(WIRE._I2C_READ());
  uint16_t y = bcd2bin(WIRE._I2C_READ()) + 2000;
  */
  return DateTime (y, m, d, dow, hh, mm, ss);
}

/*Ds3231SqwPinMode RTC_DS3231::readSqwPinMode() {
  int mode;

  WIRE.beginTransmission(DS3231_I2C_ADDRESS);
  WIRE.WRITE(DS3231_CONTROL);
  WIRE.endTransmission();
  
  WIRE.requestFrom((uint8_t)DS3231_I2C_ADDRESS, (uint8_t)1);
  mode = WIRE.READ();

  mode &= 0x93;
  return static_cast<Ds3231SqwPinMode>(mode);
}

void RTC_DS3231::writeSqwPinMode(Ds3231SqwPinMode mode) {
  WIRE.beginTransmission(DS3231_I2C_ADDRESS);
  WIRE.WRITE(DS3231_CONTROL);
  WIRE.WRITE(mode);
  WIRE.endTransmission();
}

void RTC_DS3231::readnvram(uint8_t* buf, uint8_t size, uint8_t address) {
  int addrByte = DS3231_NVRAM + address;
  WIRE.beginTransmission(DS3231_I2C_ADDRESS);
  WIRE.WRITE(addrByte);
  WIRE.endTransmission();
  
  WIRE.requestFrom((uint8_t) DS3231_I2C_ADDRESS, size);
  for (uint8_t pos = 0; pos < size; ++pos) {
    buf[pos] = WIRE.READ();
  }
}

void RTC_DS3231::writenvram(uint8_t address, uint8_t* buf, uint8_t size) {
  int addrByte = DS3231_NVRAM + address;
  WIRE.beginTransmission(DS3231_I2C_ADDRESS);
  WIRE.WRITE(addrByte);
  for (uint8_t pos = 0; pos < size; ++pos) {
    WIRE.WRITE(buf[pos]);
  }
  WIRE.endTransmission();
}

uint8_t RTC_DS3231::readnvram(uint8_t address) {
  uint8_t data;
  readnvram(&data, 1, address);
  return data;
}

void RTC_DS3231::writenvram(uint8_t address, uint8_t data) {
  writenvram(address, &data, 1);
}*/

////////////////////////////////////////////////////////////////////////////////
// RTC_3231 extended functions

char *RTC_DS3231::getDOWStr(uint8_t format)
{
	char *output = "xxxxxxxxxx";
	char *daysLong[]  = {"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
	char *daysShort[] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
	Time t;
	t=getTime();
	if (format == FORMAT_SHORT)
		output = daysShort[t.dow-1];
	else
		output = daysLong[t.dow-1];
	return output;
}

char *RTC_DS3231::getMonthStr(uint8_t format)
{
	char *output= "xxxxxxxxx";
	char *monthLong[]  = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
	char *monthShort[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
	Time t;
	t=getTime();
	if (format == FORMAT_SHORT)
		output = monthShort[t.mon-1];
	else
		output = monthLong[t.mon-1];
	return output;
}

long RTC_DS3231::getUnixTime(Time t)
{
	uint16_t	dc;

	dc = t.date;
	for (uint8_t i = 0; i<(t.mon-1); i++)
		dc += dim[i];
	if ((t.mon > 2) && (((t.year-2000) % 4) == 0))
		++dc;
	dc = dc + (365 * (t.year-2000)) + (((t.year-2000) + 3) / 4) - 1;

	return ((((((dc * 24L) + t.hour) * 60) + t.min) * 60) + t.sec) + SEC_1970_TO_2000;

}

void RTC_DS3231::enable32KHz(bool enable)
{
  uint8_t _reg = _readRegister(REG_STATUS);
  _reg &= ~(1 << 3);
  _reg |= (enable << 3);
  _writeRegister(REG_STATUS, _reg);
}

void RTC_DS3231::setOutput(byte enable)
{
  uint8_t _reg = _readRegister(REG_CON);
  _reg &= ~(1 << 2);
  _reg |= (enable << 2);
  _writeRegister(REG_CON, _reg);
}

void RTC_DS3231::setSQWRate(int rate)
{
  uint8_t _reg = _readRegister(REG_CON);
  _reg &= ~(3 << 3);
  _reg |= (rate << 3);
  _writeRegister(REG_CON, _reg);
}

float RTC_DS3231::getTemp()
{
	uint8_t _msb = _readRegister(REG_TEMPM);
	uint8_t _lsb = _readRegister(REG_TEMPL);
	return (float)_msb + ((_lsb >> 6) * 0.25f);
}


////////////////////////////////////////////////////////////////////////////////
// RTC_Millis implementation

long RTC_Millis::offset = 0;

void RTC_Millis::adjust(const DateTime& dt) {
    offset = dt.unixtime() - millis() / 1000;
}

DateTime RTC_Millis::now() {
  return (uint32_t)(offset + millis() / 1000);
}

////////////////////////////////////////////////////////////////////////////////
