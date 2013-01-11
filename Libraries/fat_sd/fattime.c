
#include "integer.h"
#include "fattime.h"
//#include "rtc.h"

DWORD get_fattime (void)
{
	DWORD res;
/*
	RTC_t rtc;

	rtc_gettime( &rtc );
	res =  (((DWORD)rtc.year - 1980) << 25)
			| ((DWORD)rtc.month << 21)
			| ((DWORD)rtc.mday << 16)
			| (WORD)(rtc.hour << 11)
			| (WORD)(rtc.min << 5)
			| (WORD)(rtc.sec >> 1);
*/
	res = ((2011 - 1980) << 25)
		| (10 << 21)
		| (16 << 16)
		| (19 << 11)
		| (26 << 5)
		| (0 >> 1);

	return res;
}

