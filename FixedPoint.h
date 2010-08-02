#ifndef _FIXED_H
#define _FIXED_H

#include <stdlib.h>
#include <math.h>

 // Allow floating point input
#define FIXED_HAS_DOUBLE

 // Allow longs
#define FIXED_HAS_LONG

#define	RESOLUTION		1000000L
#define	RESOLUTION_FLOAT	1000000.0
#define	RESOLUTION_FLOATf	1000000.0f
#define	FLOAT_RESOLUTION	0.0000005
#define	FLOAT_RESOLUTIONf	0.0000005f
#define _XPI      3141592 // 3.1415926535897932384626433832795
#define XPI      fixed(true,_XPI)
#define _X2PI     6283185 // 6.283185307179586476925286766559
#define X2PI     fixed(true,_X2PI)
#define _XPIO2    1570796 // 1.5707963267948966192313216916398
#define XPIO2    fixed(true,_XPIO2)
#define _XPIO4    785398 // 0.78539816339744830961566084581988
#define XPIO4    fixed(true, _XPIO4)
#define _XLN_E    2718282 // 2.71828182845904523536
#define XLN_E    fixed(true,_XLN_E)
#define _XLN_10   2302585 // 2.30258509299404568402
#define XLN_10   fixed(true,_XLN_10)
#define sqrt_error   fixed(true, 1000) // 0.001

float floorf (float x)
{
    return (float) floor( (double)x );
}

class fixed
{
private:
	long	m_nVal;
public:
	fixed(void);
	fixed(const fixed& fixedVal);
	fixed(const fixed* fixedVal);
	fixed(bool bInternal, long nVal);
	fixed(long nVal);
	fixed(int nVal);
	fixed(short nVal);
#ifdef FIXED_HAS_DOUBLE
	fixed(double nVal);
#endif
	fixed(float nVal);
	~fixed(void);
	fixed operator++(void);
	fixed operator--(void);
	fixed& operator=(float floatVal);
#ifdef FIXED_HAS_DOUBLE
	fixed& operator=(double floatVal);
#endif
	fixed& operator=(fixed fixedVal);
	fixed& operator=(int intVal);
	fixed& operator=(unsigned int intVal);
#ifdef FIXED_HAS_LONG
	fixed& operator=(long longVal);
	fixed& operator=(unsigned long longVal);
#endif
	fixed& operator=(short shortVal);
	fixed& operator=(unsigned short shortVal);
	fixed& operator=(char charVal);
	fixed& operator=(unsigned char charVal);
	bool equals(fixed b);
	bool operator==(float floatVal);
#ifdef FIXED_HAS_DOUBLE
	bool operator==(double floatVal);
#endif
	bool operator==(fixed fixedVal);
	bool operator==(int intVal);
#ifdef FIXED_HAS_LONG
	bool operator==(long intVal);
#endif
	bool operator==(short intVal);
	bool operator!=(float floatVal);
#ifdef FIXED_HAS_DOUBLE
	bool operator!=(double floatVal);
#endif
	bool operator!=(fixed fixedVal);
	bool operator!=(int intVal);
#ifdef FIXED_HAS_LONG
	bool operator!=(long intVal);
#endif
	bool operator!=(short intVal);
	bool lessthan(fixed b);
	bool operator<(float floatVal);
#ifdef FIXED_HAS_DOUBLE
	bool operator<(double floatVal);
#endif
	bool operator<(fixed fixedVal);
	bool operator<(int intVal);
#ifdef FIXED_HAS_LONG
	bool operator<(long intVal);
#endif
	bool operator<(short intVal);
	bool lessthanequal(fixed b);
	bool operator<=(float floatVal);
#ifdef FIXED_HAS_DOUBLE
	bool operator<=(double floatVal);
#endif
	bool operator<=(fixed fixedVal);
	bool operator<=(int intVal);
#ifdef FIXED_HAS_LONG
	bool operator<=(long intVal);
#endif
	bool operator<=(short intVal);
	bool operator>(float floatVal);
#ifdef FIXED_HAS_DOUBLE
	bool operator>(double floatVal);
#endif
	bool operator>(fixed fixedVal);
	bool operator>(int intVal);
#ifdef FIXED_HAS_LONG
	bool operator>(long intVal);
#endif
	bool operator>(short intVal);
	bool operator>=(float floatVal);
#ifdef FIXED_HAS_DOUBLE
	bool operator>=(double floatVal);
#endif
	bool operator>=(fixed fixedVal);
	bool operator>=(int intVal);
#ifdef FIXED_HAS_LONG
	bool operator>=(long intVal);
#endif
	bool operator>=(short intVal);
	operator float(void);
#ifdef FIXED_HAS_DOUBLE
	operator double(void);
#endif
	operator unsigned short(void);
#ifdef FIXED_HAS_LONG
	long GetLong(void);
	operator long(void);
#endif
	operator short(void);
	operator int(void);
	fixed floor(void);
	fixed ceil(void);
	fixed add(fixed b);
	fixed subtract(fixed b);
	fixed multiply(fixed b);
	fixed divide(fixed b);
	fixed operator+(fixed b);
	fixed operator-(fixed b);
	fixed operator*(fixed b);
	fixed operator/(fixed b);
	fixed add(float b);
	fixed subtract(float b);
	fixed multiply(float b);
	fixed divide(float b);
	fixed operator+(float b);
	fixed operator-(float b);
	fixed operator*(float b);
	fixed operator/(float b);
#ifdef FIXED_HAS_DOUBLE
	fixed add(double b);
	fixed subtract(double b);
	fixed multiply(double b);
	fixed divide(double b);
	fixed operator+(double b);
	fixed operator-(double b);
	fixed operator*(double b);
	fixed operator/(double b);
#endif
	fixed add(int b);
	fixed subtract(int b);
	fixed multiply(int b);
	fixed divide(int b);
	fixed operator+(int b);
	fixed operator-(int b);
	fixed operator*(int b);
	fixed operator/(int b);
#ifdef FIXED_HAS_LONG
	fixed add(long b);
	fixed subtract(long b);
	fixed multiply(long b);
	fixed divide(long b);
	fixed operator+(long b);
	fixed operator-(long b);
	fixed operator*(long b);
	fixed operator/(long b);
#endif
	fixed add(short b);
	fixed subtract(short b);
	fixed multiply(short b);
	fixed divide(short b);
	fixed operator+(short b);
	fixed operator-(short b);
	fixed operator*(short b);
	fixed operator/(short b);
	fixed sqrt(void);
	fixed pow(fixed fixedPower);
	fixed log10(void);
	fixed log(void);
	fixed exp(void);
	fixed cos(void);
	fixed sin(void);
	fixed tan(void);
	fixed operator%(fixed fixedVal);
#ifdef FIXED_HAS_LONG
	fixed operator%(long longVal);
#endif
	fixed operator%(int intVal);
	fixed operator%(short shortVal);
	fixed operator*=(fixed val);
#ifdef FIXED_HAS_DOUBLE
	fixed operator*=(double val);
	fixed operator/=(double val);
	fixed operator-=(double val);
	fixed operator+=(double val);
#endif
#ifdef FIXED_HAS_LONG
	fixed operator*=(long val);
	fixed operator/=(long val);
	fixed operator-=(long val);
	fixed operator+=(long val);
#endif
	fixed operator*=(float val);
	fixed operator*=(int val);
	fixed operator*=(short val);
	fixed operator/=(fixed val);
	fixed operator/=(float val);
	fixed operator/=(int val);
	fixed operator/=(short val);
	fixed operator-=(fixed val);
	fixed operator-=(float val);
	fixed operator-=(int val);
	fixed operator-=(short val);
	fixed operator+=(fixed val);
	fixed operator+=(float val);
	fixed operator+=(int val);
	fixed operator+=(short val);
};

#ifdef FIXED_HAS_DOUBLE
fixed operator-(double a, fixed b);
#endif
fixed operator-(float a, fixed b);
#ifdef FIXED_HAS_LONG
fixed operator-(long a, fixed b);
#endif
fixed operator-(int a, fixed b);
fixed operator-(short a, fixed b);

#ifdef FIXED_HAS_DOUBLE
double operator+=(double& a, fixed b);
double operator-=(double& a, fixed b);
double operator*=(double& a, fixed b);
double operator/=(double& a, fixed b);
#endif
float operator+=(float& a, fixed b);
float operator-=(float& a, fixed b);
float operator*=(float& a, fixed b);
float operator/=(float& a, fixed b);

#ifdef FIXED_HAS_DOUBLE
bool operator<(double b, fixed a);
#endif
bool operator<(float b, fixed a);
#ifdef FIXED_HAS_LONG
bool operator<(long b, fixed a);
#endif
bool operator<(short b, fixed a);
bool operator<(int b, fixed a);

fixed operator-(fixed a);

fixed absx( fixed p_Base );
fixed floorx(fixed fixedVal);
fixed ceilx(fixed fixedVal);
fixed sqrtx(fixed fixedVal);
fixed powx(fixed fixedVal, fixed fixedPower);
fixed log10x(fixed fixedVal);
fixed logx(fixed fixedVal);
fixed expx(fixed fixedVal);
fixed sinx(fixed x);
fixed cosx(fixed x);
fixed tanx(fixed x);

fixed::fixed(void)
{
	m_nVal = 0;
}

fixed::fixed(const fixed& fixedVal)
{
	m_nVal = fixedVal.m_nVal;
}

fixed::fixed(const fixed* fixedVal)
{
	m_nVal = fixedVal->m_nVal;
}

fixed::fixed(bool bInternal, long nVal)
{
	m_nVal = nVal;
}

fixed::fixed(long nVal)
{
	m_nVal = nVal*RESOLUTION;
}

fixed::fixed(int nVal)
{
	m_nVal = nVal*RESOLUTION;
}

fixed::fixed(short nVal)
{
	m_nVal = nVal*RESOLUTION;
}

#ifdef FIXED_HAS_DOUBLE
fixed::fixed(double floatVal)
{
	floatVal += FLOAT_RESOLUTION;
	m_nVal = (long)::floor(floatVal*RESOLUTION);
}
#endif

fixed::fixed(float floatVal)
{
	floatVal += FLOAT_RESOLUTIONf;
	m_nVal = (long)::floorf(floatVal*RESOLUTION);
}

fixed::~fixed(void)
{
}

fixed fixed::operator++(void)
{
	m_nVal += RESOLUTION;
	return *this;
}

fixed fixed::operator--(void)
{
	m_nVal -= RESOLUTION;
	return *this;
}

fixed& fixed::operator=(fixed fixedVal)
{
	m_nVal = fixedVal.m_nVal;
	return *this;
}

fixed& fixed::operator=(float floatVal)
{
	floatVal += FLOAT_RESOLUTIONf;
	m_nVal = (long)::floorf(floatVal*RESOLUTION);
	return *this;
}

#ifdef FIXED_HAS_DOUBLE
fixed& fixed::operator=(double floatVal)
{
	floatVal+=FLOAT_RESOLUTION;
	m_nVal = (long)::floor(floatVal*RESOLUTION);
	return *this;
}
#endif

fixed& fixed::operator=(int intVal)
{
	m_nVal = intVal*RESOLUTION;
	return *this;
}

fixed& fixed::operator=(unsigned int intVal)
{
	m_nVal = intVal*RESOLUTION;
	return *this;
}

#ifdef FIXED_HAS_LONG
fixed& fixed::operator=(long longVal)
{
	m_nVal = longVal*RESOLUTION;
	return *this;
}

fixed& fixed::operator=(unsigned long longVal)
{
	m_nVal = longVal*RESOLUTION;
	return *this;
}
#endif

fixed& fixed::operator=(short shortVal)
{
	m_nVal = shortVal*RESOLUTION;
	return *this;
}

fixed& fixed::operator=(unsigned short shortVal)
{
	m_nVal = shortVal*RESOLUTION;
	return *this;
}

fixed& fixed::operator=(char charVal)
{
	m_nVal = charVal*RESOLUTION;
	return *this;
}

fixed& fixed::operator=(unsigned char charVal)
{
	m_nVal = charVal*RESOLUTION;
	return *this;
}

bool fixed::operator==(float floatVal)
{
	floatVal+=FLOAT_RESOLUTIONf;
	return (m_nVal == (long)::floorf(floatVal*RESOLUTION));
}

#ifdef FIXED_HAS_DOUBLE
bool fixed::operator==(double floatVal)
{
	floatVal+=FLOAT_RESOLUTION;
	return (m_nVal == (long)::floor(floatVal*RESOLUTION));
}
#endif

bool fixed::operator==(fixed fixedVal)
{
	return (m_nVal == fixedVal.m_nVal);
}

bool fixed::operator==(int intVal)
{
	return (m_nVal == intVal*RESOLUTION);
}

#ifdef FIXED_HAS_LONG
bool fixed::operator==(long intVal)
{
	return (m_nVal == intVal*RESOLUTION);
}
#endif

bool fixed::operator==(short intVal)
{
	return (m_nVal == intVal*RESOLUTION);
}

bool fixed::lessthan(fixed b)
{
	return (m_nVal < b.m_nVal);
}

bool fixed::lessthanequal(fixed b)
{
	return (m_nVal <= b.m_nVal);
}

bool fixed::operator<(float floatVal)
{
	floatVal+=FLOAT_RESOLUTIONf;
	return (m_nVal < (long)::floorf(floatVal*RESOLUTION));
}

#ifdef FIXED_HAS_DOUBLE
bool fixed::operator<(double floatVal)
{
	floatVal+=FLOAT_RESOLUTION;
	return (m_nVal < (long)::floor(floatVal*RESOLUTION));
}
#endif

bool fixed::operator<(fixed fixedVal)
{
	return (m_nVal < fixedVal.m_nVal);
}

bool fixed::operator<(int intVal)
{
	return (m_nVal < intVal*RESOLUTION);
}

#ifdef FIXED_HAS_LONG
bool fixed::operator<(long intVal)
{
	return (m_nVal < intVal*RESOLUTION);
}
#endif

bool fixed::operator<(short intVal)
{
	return (m_nVal < intVal*RESOLUTION);
}

bool fixed::operator<=(float floatVal)
{
	floatVal+=FLOAT_RESOLUTIONf;
	return (m_nVal <= (long)::floorf(floatVal*RESOLUTION));
}

#ifdef FIXED_HAS_DOUBLE
bool fixed::operator<=(double floatVal)
{
	floatVal+=FLOAT_RESOLUTION;
	return (m_nVal <= (long)::floor(floatVal*RESOLUTION));
}
#endif

bool fixed::operator<=(fixed fixedVal)
{
	return (m_nVal <= fixedVal.m_nVal);
}

bool fixed::operator<=(int intVal)
{
	return (m_nVal <= intVal*RESOLUTION);
}

#ifdef FIXED_HAS_LONG
bool fixed::operator<=(long intVal)
{
	return (m_nVal <= intVal*RESOLUTION);
}
#endif

bool fixed::operator<=(short intVal)
{
	return (m_nVal <= intVal*RESOLUTION);
}

bool fixed::operator>(float floatVal)
{
	floatVal+=FLOAT_RESOLUTIONf;
	return (m_nVal > (long)::floorf(floatVal*RESOLUTION));
}

#ifdef FIXED_HAS_DOUBLE
bool fixed::operator>(double floatVal)
{
	floatVal+=FLOAT_RESOLUTION;
	return (m_nVal > (long)::floor(floatVal*RESOLUTION));
}
#endif

bool fixed::operator>(fixed fixedVal)
{
	return (m_nVal > fixedVal.m_nVal);
}

bool fixed::operator>(int intVal)
{
	return (m_nVal > intVal*RESOLUTION);
}

#ifdef FIXED_HAS_LONG
bool fixed::operator>(long intVal)
{
	return (m_nVal > intVal*RESOLUTION);
}
#endif

bool fixed::operator>(short intVal)
{
	return (m_nVal > intVal*RESOLUTION);
}

bool fixed::operator>=(float floatVal)
{
	floatVal+=FLOAT_RESOLUTIONf;
	return (m_nVal >= (long)::floorf(floatVal*RESOLUTION));
}

#ifdef FIXED_HAS_DOUBLE
bool fixed::operator>=(double floatVal)
{
	floatVal+=FLOAT_RESOLUTION;
	return (m_nVal >= (long)::floor(floatVal*RESOLUTION));
}
#endif

bool fixed::operator>=(fixed fixedVal)
{
	return (m_nVal >= fixedVal.m_nVal);
}

bool fixed::operator>=(int intVal)
{
	return (m_nVal >= intVal*RESOLUTION);
}

#ifdef FIXED_HAS_LONG
bool fixed::operator>=(long intVal)
{
	return (m_nVal >= intVal*RESOLUTION);
}
#endif

bool fixed::operator>=(short intVal)
{
	return (m_nVal >= intVal*RESOLUTION);
}

bool fixed::operator!=(float floatVal)
{
	floatVal+=FLOAT_RESOLUTIONf;
	return (m_nVal != (long)::floorf(floatVal*RESOLUTION));
}

#ifdef FIXED_HAS_DOUBLE
bool fixed::operator!=(double floatVal)
{
	floatVal+=FLOAT_RESOLUTION;
	return (m_nVal != (long)::floor(floatVal*RESOLUTION));
}
#endif

bool fixed::operator!=(fixed fixedVal)
{
	return (m_nVal != fixedVal.m_nVal);
}

bool fixed::operator!=(int intVal)
{
	return (m_nVal != intVal*RESOLUTION);
}

#ifdef FIXED_HAS_LONG
bool fixed::operator!=(long intVal)
{
	return (m_nVal != intVal*RESOLUTION);
}
#endif

bool fixed::operator!=(short intVal)
{
	return (m_nVal != intVal*RESOLUTION);
}

fixed::operator float(void)
{
	return m_nVal/RESOLUTION_FLOATf;
}

#ifdef FIXED_HAS_DOUBLE
fixed::operator double(void)
{
	return m_nVal/RESOLUTION_FLOAT;
}
#endif

#ifdef FIXED_HAS_LONG
long fixed::GetLong(void)
{
	return m_nVal/RESOLUTION;
}

fixed::operator long(void)
{
	return (m_nVal/RESOLUTION);
}
#endif

fixed::operator int(void)
{
	return (int)(m_nVal/RESOLUTION);
}

fixed::operator short(void)
{
	return (short)(m_nVal/RESOLUTION);
}

fixed::operator unsigned short(void)
{
	return (unsigned short)(m_nVal/RESOLUTION);
}

fixed fixed::floor(void)
{
	return (fixed)(m_nVal/RESOLUTION);
}

fixed fixed::ceil(void)
{
	return (fixed)(m_nVal/RESOLUTION+1);
}

fixed fixed::operator%(fixed fixedVal)
{
	fixed a;
	a.m_nVal = m_nVal%fixedVal.m_nVal;
	return a;
}

#ifdef FIXED_HAS_LONG
fixed fixed::operator%(long longVal)
{
	fixed a;
	a.m_nVal = m_nVal%longVal;
	return a;
}
#endif

fixed fixed::operator%(int intVal)
{
	fixed a;
	a.m_nVal = m_nVal%intVal;
	return a;
}

fixed fixed::operator%(short shortVal)
{
	fixed a;
	a.m_nVal = m_nVal%shortVal;
	return a;
}

bool fixed::equals(fixed b)
{
	return (m_nVal == b.m_nVal);
}

fixed fixed::add(fixed b)
{
	fixed a;
	a.m_nVal = m_nVal+b.m_nVal;
	return a;
}

fixed fixed::operator+(fixed b)
{
	return add(b);
}

fixed fixed::subtract(fixed b)
{
	fixed a;
	a.m_nVal = m_nVal-b.m_nVal;
	return a;
}

fixed fixed::operator-(fixed b)
{
	return subtract(b);
}

fixed fixed::multiply(fixed b)
{
	fixed a;
	a.m_nVal = (long)(((long long)m_nVal*b.m_nVal)/RESOLUTION);
	return a;
}

fixed fixed::operator*(fixed b)
{
	return multiply(b);
}

fixed fixed::divide(fixed b)
{
	if( b == 0 ) return fixed(true, 0xFFFFFFFFL);
	fixed a;
	if( m_nVal < 0xFFFFFFFFL/RESOLUTION )
		a.m_nVal = (m_nVal*RESOLUTION/b.m_nVal);
	else
		a.m_nVal = (long)(((long long)m_nVal*RESOLUTION/b.m_nVal));
	return a;
}

fixed fixed::operator/(fixed b)
{
	return divide(b);
}

fixed fixed::add(float b)
{
	fixed _b = b;
	return add(_b);
}

fixed fixed::operator+(float b)
{
	return add(b);
}

fixed fixed::subtract(float b)
{
	fixed _b = b;
	return subtract(_b);
}

fixed fixed::operator-(float b)
{
	return subtract(b);
}

fixed fixed::multiply(float b)
{
	fixed _b = b;
	return multiply(_b);
}

fixed fixed::operator*(float b)
{
	return multiply(b);
}

fixed fixed::divide(float b)
{
	fixed _b = b;
	return divide(_b);
}

fixed fixed::operator/(float b)
{
	return divide(b);
}

#ifdef FIXED_HAS_DOUBLE
fixed fixed::add(double b)
{
	fixed _b = b;
	return add(_b);
}

fixed fixed::operator+(double b)
{
	return add(b);
}

fixed fixed::subtract(double b)
{
	fixed _b = b;
	return subtract(_b);
}

fixed fixed::operator-(double b)
{
	return subtract(b);
}

fixed fixed::multiply(double b)
{
	fixed _b = b;
	return multiply(_b);
}

fixed fixed::operator*(double b)
{
	return multiply(b);
}

fixed fixed::divide(double b)
{
	fixed _b = b;
	return divide(_b);
}

fixed fixed::operator/(double b)
{
	return divide(b);
}
#endif

fixed fixed::add(int b)
{
	fixed _b = b;
	return add(_b);
}

fixed fixed::operator+(int b)
{
	return add(b);
}

fixed fixed::subtract(int b)
{
	fixed _b = b;
	return subtract(_b);
}

fixed fixed::operator-(int b)
{
	return subtract(b);
}

fixed fixed::multiply(int b)
{
	fixed _b = b;
	return multiply(_b);
}

fixed fixed::operator*(int b)
{
	return multiply(b);
}

fixed fixed::divide(int b)
{
	fixed _b = b;
	return divide(_b);
}

fixed fixed::operator/(int b)
{
	return divide(b);
}

#ifdef FIXED_HAS_LONG
fixed fixed::add(long b)
{
	fixed _b = b;
	return add(_b);
}

fixed fixed::operator+(long b)
{
	return add(b);
}

fixed fixed::subtract(long b)
{
	fixed _b = b;
	return subtract(_b);
}

fixed fixed::operator-(long b)
{
	return subtract(b);
}

fixed fixed::multiply(long b)
{
	fixed _b = b;
	return multiply(_b);
}

fixed fixed::operator*(long b)
{
	return multiply(b);
}

fixed fixed::divide(long b)
{
	fixed _b = b;
	return divide(_b);
}

fixed fixed::operator/(long b)
{
	return divide(b);
}
#endif

fixed fixed::add(short b)
{
	fixed _b = b;
	return add(_b);
}

fixed fixed::operator+(short b)
{
	return add(b);
}

fixed fixed::subtract(short b)
{
	fixed _b = b;
	return subtract(_b);
}

fixed fixed::operator-(short b)
{
	return subtract(b);
}

fixed fixed::multiply(short b)
{
	fixed _b = b;
	return multiply(_b);
}

fixed fixed::operator*(short b)
{
	return multiply(b);
}

fixed fixed::divide(short b)
{
	fixed _b = b;
	return divide(_b);
}

fixed fixed::operator/(short b)
{
	return divide(b);
}

fixed fixed::operator*=(fixed val)
{
	m_nVal = (long)(((long long)m_nVal*val.m_nVal)/RESOLUTION);
	return *this;
}

#ifdef FIXED_HAS_DOUBLE
fixed fixed::operator*=(double val)
{
	m_nVal = (long)(m_nVal*val);
	return *this;
}

fixed fixed::operator/=(double val)
{
	m_nVal = (long)(m_nVal/val);
	return *this;
}

fixed fixed::operator-=(double val)
{
	m_nVal -= (long)(val*RESOLUTION);
	return *this;
}

fixed fixed::operator+=(double val)
{
	m_nVal += (long)(val*RESOLUTION);
	return *this;
}

#endif
#ifdef FIXED_HAS_LONG
fixed fixed::operator*=(long val)
{
	m_nVal = (long)((long long)m_nVal*val);
	return *this;
}

fixed fixed::operator/=(long val)
{
	m_nVal = (long)(((long long)m_nVal*RESOLUTION)/val);
	return *this;
}

fixed fixed::operator-=(long val)
{
	m_nVal -= val*RESOLUTION;
	return *this;
}

fixed fixed::operator+=(long val)
{
	m_nVal += val*RESOLUTION;
	return *this;
}

#endif
fixed fixed::operator*=(float val)
{
	m_nVal = (long)(m_nVal*val);
	return *this;
}

fixed fixed::operator*=(int val)
{
	m_nVal = (long)((long long)m_nVal*val);
	return *this;
}

fixed fixed::operator*=(short val)
{
	m_nVal = (long)((long long)m_nVal*val);
	return *this;
}

fixed fixed::operator/=(fixed val)
{
	m_nVal = (long)(((long long)m_nVal*RESOLUTION)/val.m_nVal);
	return *this;
}

fixed fixed::operator/=(float val)
{
	m_nVal = (long)(m_nVal/val);
	return *this;
}

fixed fixed::operator/=(int val)
{
	m_nVal = (long)(((long long)m_nVal*RESOLUTION)/val);
	return *this;
}

fixed fixed::operator/=(short val)
{
	m_nVal = (long)(((long long)m_nVal*RESOLUTION)/val);
	return *this;
}

fixed fixed::operator-=(fixed val)
{
	m_nVal -= val.m_nVal;
	return *this;
}

fixed fixed::operator-=(float val)
{
	m_nVal -= (long)(val*RESOLUTION);
	return *this;
}

fixed fixed::operator-=(int val)
{
	m_nVal -= val*RESOLUTION;
	return *this;
}

fixed fixed::operator-=(short val)
{
	m_nVal -= val*RESOLUTION;
	return *this;
}

fixed fixed::operator+=(fixed val)
{
	m_nVal += val.m_nVal;
	return *this;
}

fixed fixed::operator+=(float val)
{
	m_nVal += (long)(val*RESOLUTION);
	return *this;
}

fixed fixed::operator+=(int val)
{
	m_nVal += val*RESOLUTION;
	return *this;
}

fixed fixed::operator+=(short val)
{
	m_nVal += val*RESOLUTION;
	return *this;
}

#ifdef FIXED_HAS_DOUBLE
fixed operator-(double a, fixed b)
{
	fixed _a = a;
	return _a - b;
}
#endif

fixed operator-(float a, fixed b)
{
	fixed _a = a;
	return _a - b;
}

#ifdef FIXED_HAS_LONG
fixed operator-(long a, fixed b)
{
	fixed _a = a;
	return _a - b;
}
#endif

fixed operator-(int a, fixed b)
{
	fixed _a = a;
	return _a - b;
}

fixed operator-(short a, fixed b)
{
	fixed _a = a;
	return _a - b;
}

#ifdef FIXED_HAS_LONG
fixed operator*(fixed a, long b)
{
	return a.multiply(b);
}
#endif

float operator+=(float& a, fixed b)
{
	return a += (float)b;
}

#ifdef FIXED_HAS_DOUBLE
double operator+=(double& a, fixed b)
{
	return a += (double)b;
}
#endif

float operator-=(float& a, fixed b)
{
	return a -= (float)b;
}

#ifdef FIXED_HAS_DOUBLE
double operator-=(double& a, fixed b)
{
	return a -= (double)b;
}

double operator*=(double& a, fixed b)
{
	return a *= (double)b;
}

double operator/=(double& a, fixed b)
{
	return a /= (double)b;
}

bool operator<(double b, fixed a)
{
	return a >= b;
}

#endif

float operator*=(float& a, fixed b)
{
	return a *= (float)b;
}

float operator/=(float& a, fixed b)
{
	return a /= (float)b;
}

bool operator<(float b, fixed a)
{
	return a >= b;
}

#ifdef FIXED_HAS_LONG
bool operator<(long b, fixed a)
{
	return a >= b;
}
#endif

bool operator<(int b, fixed a)
{
	return a >= b;
}

bool operator<(short b, fixed a)
{
	return a >= b;
}

fixed operator-(fixed a)
{
	return 0-a;
}

/*
static
unsigned int isqrt(unsigned long n)
{
    unsigned long i;
    unsigned long k0, k1, nn;

    for (nn = i = n, k0 = 2; i > 0; i >>= 2, k0 <<= 1)
        ;
    nn <<= 2;
    for (;;) 
	{
        k1 = (nn / k0 + k0) >> 1;
        if (((k0 ^ k1) & ~1) == 0)
            break;
        k0 = k1;
    }
    return (unsigned int) ((k1 + 1) >> 1);
}
*/

static
fixed isqrtB(fixed p_Square)
{
	fixed   res;
	fixed   delta;
	fixed   maxError;

	if( p_Square <= 0 )
		return 0;

	/* start at half */
	res = (p_Square / 2);

	/* determine allowable error */
	maxError =  (p_Square * sqrt_error);

	do
	{
		delta =  (( res * res ) - p_Square);
		res -=  (delta / ( res * 2 ));
	}
	while( delta > maxError || delta < -maxError );

	return res;
}

static
long iabs( register long p_Base )
{
	if( p_Base < 0 ) return -p_Base;
	return p_Base;
}

fixed absx( fixed p_Base )
{
	if( p_Base < 0 ) return -p_Base;
	return p_Base;
}
/*
                     (x^h) - 1
   ln(x)  =   lim    -------      
             h -> 0     h

*/

static
fixed iLog2( fixed p_Base )
{   
    fixed w = 0;
	fixed y = 0;
	fixed z = 0;
	int num = 1;
	int dec = 0;

	if( p_Base == 1 )
		return 0;

	for( dec=0 ; absx( p_Base ) >= 2 ; ++dec )
		p_Base /= XLN_E;

	p_Base -= 1;
	z = p_Base;
	y = p_Base;
	w = 1;

	while( y != y + w )
		y += ( w = ( z = 0 - ( z * p_Base ) ) / ( num += 1 ) );

	return y + dec;
}

/*
	calculate the exponential function using the following series :

                          x^2     x^3     x^4     x^5
	exp(x) == 1  +  x  +  ---  +  ---  +  ---  +  ---  ...
                           2!      3!      4!      5!

*/

static
fixed iExp2(fixed p_Base)
{
	fixed w;
	fixed y;
	int num;

	for( w=1, y=1, num=1 ; y != y+w ; ++num )
		y += ( w *= p_Base / num );

	return y;
}

static
fixed ipow( fixed p_Base, fixed p_Power )
{
	if( p_Base < 0 && p_Power%2 != 0 )
		return - iExp2( (p_Power * iLog2( -p_Base )) );
	else
		return iExp2( (p_Power * iLog2(absx( p_Base ))) );
}

static
fixed ilog10( fixed p_Base )
{
	return iLog2( p_Base ) / XLN_10;
}

fixed fixed::sqrt(void)
{
	//return fixed(true, isqrt(m_nVal)*(RESOLUTION/1000L));
	return isqrtB(*this);
}

fixed sqrtx(fixed fixedVal)
{
	return fixedVal.sqrt();
}

fixed fixed::pow(fixed fixedPower)
{
	return ipow(*this, fixedPower);
}

fixed powx(fixed fixedVal, fixed fixedPower)
{
	return fixedVal.pow(fixedPower);
}

fixed fixed::exp(void)
{
	return iExp2(*this);
}

fixed expx(fixed fixedVal)
{
	return fixedVal.exp();
}

fixed fixed::log10(void)
{
	return ilog10(*this);
}

fixed log10x(fixed fixedVal)
{
	return fixedVal.log10();
}

fixed fixed::log(void)
{
	return iLog2(*this);
/*
	Calculate the POW function by calling EXP :

                  Y      A                 
                 X   =  e    where A = Y * log(X)
*/
}

fixed logx(fixed fixedVal)
{
	return fixedVal.log();
}

fixed floorx(fixed fixedVal)
{
	return fixedVal.floor();
}

fixed ceilx(fixed fixedVal)
{
	return fixedVal.ceil();
}

//
// Taylor Algorythm
// x - x^3/3! + x^5/5! - x^7/7! + x^9/9! ........    
//
// Note: Make xresult a float to get more precision
//
// Only accurate from -PI/2 to PI/2

static
fixed _sinx(fixed x)
{
	fixed xpwr;
	long xftl;
	fixed xresult;
	bool positive;

	xresult = 0;
	xpwr = x;
	xftl = 1;
	positive = true;

	// Note: 12! largest for long
	for(int i = 1; i < 7; i+=2)
	{
		if( positive )
			xresult += (xpwr/xftl);
		else
			xresult -= (xpwr/xftl);

		xpwr *= x;
		xpwr *= x;
		xftl *= i+1;
		xftl *= i+2;
		positive = !positive;
	}

	return xresult;
}

fixed fixed::sin(void)
{
	fixed xresult;
	bool bBottom = false;
	static fixed xPI = XPI;
	static fixed x2PI = X2PI;
	static fixed xPIO2 = XPIO2;

	fixed x(true, m_nVal%_X2PI);
	if( x < 0 )
		x += x2PI;

	if( x > xPI )
	{
		bBottom = true;
		x -= xPI;
	}

	if( x <= xPIO2 )
		xresult = _sinx(x);
	else
		xresult = _sinx(xPIO2-(x-xPIO2));

	if( bBottom )
		return -xresult;

	return xresult;
}

fixed sinx(fixed x)
{
	return x.sin();
}

fixed fixed::cos(void)
{
	fixed xresult;
	bool bBottom = false;
	static fixed xPI = XPI;
	static fixed x2PI = X2PI;
	static fixed xPIO2 = XPIO2;

	fixed x(true, (m_nVal+_XPIO2)%_X2PI);
	if( x < 0 )
		x += x2PI;

	if( x > xPI )
	{
		bBottom = true;
		x -= xPI;
	}

	if( x <= xPIO2 )
		xresult = _sinx(x);
	else
		xresult = _sinx(xPIO2-(x-xPIO2));

	if( bBottom )
		return -xresult;

	return xresult;
}

fixed cosx(fixed x)
{
	return x.cos();
}

fixed fixed::tan(void)
{
	return sin()/cos();
}

fixed tanx(fixed x)
{
	return x.tan();
}

#endif // _FIXED_H


