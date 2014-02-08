/* Martin Thomas 4/2009 */

#include "integer.h"
#include "fattime.h"
//#include "rtc.h"

DWORD get_fattime (void)
{
	DWORD res=0;
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
	return res;
}

