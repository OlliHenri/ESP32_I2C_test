//#############################################################################
//
//  RTC DS3231 real time clock routines
//
//

#include <Arduino.h>

uint8_t _yearFrom2000;
uint8_t _month;
uint8_t _dayOfMonth;
uint8_t _hour;
uint8_t _minute;
uint8_t _second;

// convert String to set DS3231
uint8_t StringToUint8(const char *pString)
{
    uint8_t value = 0;

    // skip leading 0 and spaces
    while ('0' == *pString || *pString == ' ')
    {
        pString++;
    }

    // calculate number until we hit non-numeral char
    while ('0' <= *pString && *pString <= '9')
    {
        value *= 10;
        value += *pString - '0';
        pString++;
    }
    return value;
}

void RtcDateTime(const char *date, const char *time)
{
    // sample input: date = "Dec 06 2009", time = "12:34:56"
    _yearFrom2000 = StringToUint8(date + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    switch (date[0])
    {
    case 'J':
        if (date[1] == 'a')
            _month = 1;
        else if (date[2] == 'n')
            _month = 6;
        else
            _month = 7;
        break;
    case 'F':
        _month = 2;
        break;
    case 'A':
        _month = date[1] == 'p' ? 4 : 8;
        break;
    case 'M':
        _month = date[2] == 'r' ? 3 : 5;
        break;
    case 'S':
        _month = 9;
        break;
    case 'O':
        _month = 10;
        break;
    case 'N':
        _month = 11;
        break;
    case 'D':
        _month = 12;
        break;
    }
    _dayOfMonth = StringToUint8(date + 4);
    _hour = StringToUint8(time);
    _minute = StringToUint8(time + 3);
    _second = StringToUint8(time + 6);
}

// #### convert epoch time to set DS3231
enum DayOfWeek
{
    DayOfWeek_Sunday = 0,
    DayOfWeek_Monday,
    DayOfWeek_Tuesday,
    DayOfWeek_Wednesday,
    DayOfWeek_Thursday,
    DayOfWeek_Friday,
    DayOfWeek_Saturday,
};

const uint16_t c_OriginYear = 2000;
const uint32_t c_Epoch32OfOriginYear = 946684800; // Saturday, 1. January 2000 00:00:00

void InitWithEpoch32Time(uint32_t time)
{
    //_initWithSecondsFrom2000<uint32_t>(time - c_Epoch32OfOriginYear);     // this needs to be corrected here
    ;
}

template <typename T>
void _initWithSecondsFrom2000(T secondsFrom2000)
{
    _second = secondsFrom2000 % 60;
    T timeFrom2000 = secondsFrom2000 / 60;
    _minute = timeFrom2000 % 60;
    timeFrom2000 /= 60;
    _hour = timeFrom2000 % 24;
    T days = timeFrom2000 / 24;
    T leapDays;

    for (_yearFrom2000 = 0;; ++_yearFrom2000)
    {
        leapDays = (_yearFrom2000 % 4 == 0) ? 1 : 0;
        if (days < 365U + leapDays)
            break;
        days -= 365 + leapDays;
    }
    for (_month = 1;; ++_month)
    {
        uint8_t daysPerMonth = 0;     // pgm_read_byte(c_daysInMonth + _month - 1);   // need to declare  this
        if (leapDays && _month == 2)
            daysPerMonth++;
        if (days < daysPerMonth)
            break;
        days -= daysPerMonth;
    }
    _dayOfMonth = days + 1;
}