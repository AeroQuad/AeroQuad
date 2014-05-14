//=====================================================================================================
// AHRS.h
// S.O.H. Madgwick
// 25th August 2010
//
// 1 June 2012 Modified by J. Ihlein
// 27 Aug  2012 Extensively modified to include G.K. Egan's accel confidence calculations and
//                                                          calculation efficiency updates
//=====================================================================================================
//
// See AHRS.c file for description.
//
//=====================================================================================================

#ifndef _AQ_KINEMATICS_MARG_
#define _AQ_KINEMATICS_MARG_

#include "Accelerometer.h"
#include "Kinematics.h"
#include "MagnetometerDeclinationDB.h"

//----------------------------------------------------------------------------------------------------
// Variable declaration
float trueNorthHeading = 0.0;
boolean magDataUpdate = false;

float exAcc    = 0.0f, eyAcc    = 0.0f, ezAcc    = 0.0f; // accel error
float exAccInt = 0.0f, eyAccInt = 0.0f, ezAccInt = 0.0f; // accel integral error
float exMag    = 0.0f, eyMag    = 0.0f, ezMag    = 0.0f; // mag error
float exMagInt = 0.0f, eyMagInt = 0.0f, ezMagInt = 0.0f; // mag integral error
float kpAcc, kiAcc;

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// auxiliary variables to reduce number of repeated operations
float q0q0, q0q1, q0q2, q0q3;
float q1q1, q1q2, q1q3;
float q2q2, q2q3;
float q3q3;

float halfT;

uint8_t isAHRSInitialized = false;
float compassDeclination = 0.0;

float previousEx = 0.0;
float previousEy = 0.0;
float previousEz = 0.0;


#define SQR(x)  ((x) * (x))

//----------------------------------------------------------------------------------------------------

float accConfidenceDecay = 0.0f;
float accConfidence      = 1.0f;

#define HardFilter(O,N)  ((O)*0.9f+(N)*0.1f)

//----------------------------------------------------------------------------------------------------

//====================================================================================================
// Initialization
//====================================================================================================

void initializeKinematics(float ax, float ay, float az, float mx, float my, float mz)
{
	initializeBaseKinematicParam();

    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll  = atan2(-ay, -az);
    initialPitch = atan2( ax, -az);

    cosRoll  = cos(initialRoll);
    sinRoll  = sin(initialRoll);
    cosPitch = cos(initialPitch);
    sinPitch = sin(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2(-magY, magX);

    cosRoll = cos(initialRoll * 0.5f);
    sinRoll = sin(initialRoll * 0.5f);

    cosPitch = cos(initialPitch * 0.5f);
    sinPitch = sin(initialPitch * 0.5f);

    cosHeading = cos(initialHdg * 0.5f);
    sinHeading = sin(initialHdg * 0.5f);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}

//====================================================================================================
// Function
//====================================================================================================

void calculateKinematics(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    float dt)
{
    float norm, normR;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float q0i, q1i, q2i, q3i;

	halfT = dt * 0.5f;
	norm = sqrt(SQR(ax) + SQR(ay) + SQR(az));
	
	kpAcc = 0.2 * accConfidence;
	kiAcc = 0.0005 * accConfidence;

	normR = 1.0f / norm;
	ax *= normR;
	ay *= normR;
	az *= normR;

	// estimated direction of gravity (v)
	vx = 2.0f * (q1q3 - q0q2);
	vy = 2.0f * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;

	// error is sum of cross product between reference direction
	// of fields and direction measured by sensors
	exAcc = vy * az - vz * ay;
	eyAcc = vz * ax - vx * az;
	ezAcc = vx * ay - vy * ax;

	gx += exAcc * kpAcc;
	gy += eyAcc * kpAcc;
	gz += ezAcc * kpAcc;

	if (kiAcc > 0.0f)
	{
		exAccInt += exAcc * kiAcc;
		if (isSwitched(previousEx,exAcc)) {
			exAccInt = 0.0;
		}
		previousEx = exAcc;
		eyAccInt += eyAcc * kiAcc;
		if (isSwitched(previousEy,eyAcc)) {
			eyAccInt = 0.0;
		}
		previousEy = eyAcc;
		ezAccInt += ezAcc * kiAcc;
		if (isSwitched(previousEz,ezAcc)) {
			ezAccInt = 0.0;
		}
		previousEz = ezAcc;
		gx += exAccInt;
		gy += eyAccInt;
		gz += ezAccInt;
	}

	//-------------------------------------------
	norm = sqrt(SQR(mx) + SQR(my) + SQR(mz));
	if (( magDataUpdate == true) && (norm != 0.0f))
	{
		normR = 1.0f / norm;
		mx *= normR;
		my *= normR;
		mz *= normR;

		// compute reference direction of flux
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
		bx = sqrt((hx * hx) + (hy * hy));
		bz = hz;

		// estimated direction of flux (w)
		wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
		wy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
		wz = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));

		exMag = my * wz - mz * wy;
		eyMag = mz * wx - mx * wz;
		ezMag = mx * wy - my * wx;

		// use un-extrapolated old values between magnetometer updates
		// dubious as dT does not apply to the magnetometer calculation so
		// time scaling is embedded in KpMag and KiMag
		gx += exMag * 0.002;
		gy += eyMag * 0.002;
		gz += ezMag * 0.002;

		exMagInt += exMag * 0.00005;
		eyMagInt += eyMag * 0.00005;
		ezMagInt += ezMag * 0.00005;

		gx += exMagInt;
		gy += eyMagInt;
		gz += ezMagInt;
	}

	// integrate quaternion rate
	q0i = (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	q1i = ( q0 * gx + q2 * gz - q3 * gy) * halfT;
	q2i = ( q0 * gy - q1 * gz + q3 * gx) * halfT;
	q3i = ( q0 * gz + q1 * gy - q2 * gx) * halfT;
	q0 += q0i;
	q1 += q1i;
	q2 += q2i;
	q3 += q3i;

	// normalise quaternion
	normR = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= normR;
	q1 *= normR;
	q2 *= normR;
	q3 *= normR;

	// auxiliary variables to reduce number of repeated operations
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

	kinematicsAngle[XAXIS] = atan2( 2.0f * (q0q1 + q2q3), q0q0 - q1q1 - q2q2 + q3q3 );
	kinematicsAngle[YAXIS] = -asin( 2.0f * (q1q3 - q0q2) );
	trueNorthHeading = -atan2( 2.0f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3 );

	kinematicCorrectedAccel[0] = 2 * q1 * q3 - 2 * q0 * q2;
    kinematicCorrectedAccel[1] = 2 * q2 * q3 + 2 * q0 * q1;
    kinematicCorrectedAccel[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	#if defined UseGPS
		if( compassDeclination != 0.0 ) {
			trueNorthHeading = trueNorthHeading + compassDeclination;
			if (trueNorthHeading > M_PI)  {  // Angle normalization (-180 deg, 180 deg)
				trueNorthHeading -= (2.0 * M_PI);
			} 
			else if (trueNorthHeading < -M_PI){
				trueNorthHeading += (2.0 * M_PI);
			}
		}
	#endif
}


#if defined UseGPS
  void setDeclinationLocation(long lat, long lon) {
	// get declination ( in radians )
	compassDeclination = getMagnetometerDeclination(lat, lon);    
  }
#endif

#endif