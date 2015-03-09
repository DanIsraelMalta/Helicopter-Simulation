/**
* Math utilities
**/

#include <math.h>
#include "matlib.h"
#include "simlib.h"

/**
* Euler angles (phi, theta, psi) to direction cosine matrix
*
* @param  {double} phi [rad]
* @param  {double} theta [rad]
* @param  {double} psi [rad]
* @return {array}  3x3 DCM matrix
**/
void F_EULER_TO_DCM(double xo_dcm[3][3], const double xi_phi, const double xi_theta, const double xi_psi)
{
	// locals
	double cosPsi   = cos(xi_psi),
		   cosPhi   = cos(xi_phi),
		   cosTheta = cos(xi_theta),
		   sinPsi   = sin(xi_psi),
		   sinPhi   = sin(xi_phi),
		   sinTheta = sin(xi_theta);


	// DCM construction
	xo_dcm[_X][_X]  = cosPsi * cosTheta;
	xo_dcm[_X][_Y]  = sinPsi * cosTheta;
	xo_dcm[_X][_Z]  = -sinTheta;

	xo_dcm[_Y][_X]  = cosPsi   * sinTheta * sinPhi   - sinPsi * cosPhi;
	xo_dcm[_Y][_Y]  = sinPhi   * sinPhi   * sinTheta + cosPsi * cosPhi;
	xo_dcm[_Y][_Z]  = cosTheta * sinPhi;

	xo_dcm[_Z][_X]  = cosPsi   * sinTheta * cosPhi + sinPsi * sinPhi;
	xo_dcm[_Z][_Y]  = sinPsi   * sinTheta * cosPhi - cosPsi * sinPhi;
	xo_dcm[_Z][_Z]  = cosTheta * cosPhi;

}

/**
* quaternion to direction cosine matrix
*
* @param  {array} 1x4 quaternion
* @return {array} 3x3 DCM matrix
**/
void F_QUATERNION_TO_DCM(double xo_dcm[3][3], double xi_quat[4])
{
	// locals
	double q0 = xi_quat[_qW],
		   q1 = xi_quat[_qX],
		   q2 = xi_quat[_qY],
		   q3 = xi_quat[_qZ],
		   q02 = q0,
		   q12 = q1,
		   q22 = q2,
		   q32 = q3;

	// housekeeping
	q02 *= q0;
	q12 *= q1;
	q22 *= q2;
	q32 *= q3;

	// DCM construction
	xo_dcm[_X][_X]  = 1.0 - 2.0 * (q22 + q32);
	xo_dcm[_X][_Y]  = 2.0 * (q1 * q2 + q0 * q3);
	xo_dcm[_X][_Z]  = 2.0 * (q1 * q3 - q0 * q2);

	xo_dcm[_Y][_X]  = 2.0 * (q1 * q2 - q0 * q3);
	xo_dcm[_Y][_Y]  = 1.0 - 2.0 * (q12 + q32);
	xo_dcm[_Y][_Z]  = 2.0 * (q2 * q3 + q0 * q1);

	xo_dcm[_Z][_X]  = 2.0 * (q1 * q3 + q0 * q2);
	xo_dcm[_Z][_Y]  = 2.0 * (q2 * q3 - q0 * q1);
	xo_dcm[_Z][_Z]  = 1.0 - 2.0 * (q12 + q22);
}

/**
* quaternion normalization
*
* @param  {array} 1x4 quaternion
* @return {array} 1x4 normalized quaternion
**/
void F_QUATERNION_NORMALIZATION(double xio_quaternion[4]) {
	// locals
	double q0 = xio_quaternion[_qW],
		   q1 = xio_quaternion[_qX],
		   q2 = xio_quaternion[_qY],
		   q3 = xio_quaternion[_qZ],
		   q02 = q0 * q0,
		   q12 = q1 * q1,
		   q22 = q2 * q2,
		   q32 = q3 * q3,
		   invsqr = 1.0 / sqrt(q02 + q12 + q22 + q32); // inverse of quaternion magnitude

	// normalize quaternion
	xio_quaternion[_qW] *= invsqr;
	xio_quaternion[_qX] *= invsqr;
	xio_quaternion[_qY] *= invsqr;
	xio_quaternion[_qZ] *= invsqr;
}

/**
* geodetic {latitude, longitude, altitude [positive = up]} to ECEF {x, y, z}
*
* @return {array} ECEF vector {x, y, z}
* @param  {array} local position
* @param  {array} latitude [rad]
* @param  {array} longitude [rad]
**/
void F_GEODETIC_TO_ECEF(double xo_ecef[3], double xi_local[3], double xi_latitude, double xi_longitude) {
	// locals
	double _transformationMatrix[3][3],
		   _cosLat, _cosLon, _sinLat, _sinLon;

	// initialization
	_cosLat = cos(xi_latitude);
	_cosLon = cos(xi_longitude);
	_sinLat = sin(xi_latitude);
	_sinLon = sin(xi_longitude);

	// transformation matrix construction
	_transformationMatrix[_X][_X] = -_sinLat * _cosLon;
	_transformationMatrix[_Y][_X] = -_sinLat * _sinLon;
	_transformationMatrix[_Z][_X] = _cosLat;

	_transformationMatrix[_X][_Y] = -_sinLon;
	_transformationMatrix[_Y][_Y] = _cosLon;
	_transformationMatrix[_Z][_Y] = 0.0;

	_transformationMatrix[_X][_Z] = -_cosLat * _cosLon;
	_transformationMatrix[_Y][_Z] = -_cosLat * _sinLon;
	_transformationMatrix[_Z][_Z] = -_sinLat;

	// geodetic to ECEF
	xo_ecef[_X] = _transformationMatrix[_X][_X] * xi_local[_X] + _transformationMatrix[_X][_Y] * xi_local[_Y] + _transformationMatrix[_X][_Z] * xi_local[_Z];
	xo_ecef[_Y] = _transformationMatrix[_Y][_X] * xi_local[_X] + _transformationMatrix[_Y][_Y] * xi_local[_Y] + _transformationMatrix[_Y][_Z] * xi_local[_Z];
	xo_ecef[_Z] = _transformationMatrix[_Z][_X] * xi_local[_X] + _transformationMatrix[_Z][_Y] * xi_local[_Y] + _transformationMatrix[_Z][_Z] * xi_local[_Z];
}

/**
* ECEF {x, y, z} to GEODETIC {latitude, longitude, altitude [positive =  up]}
*
* @param  {array} 1x3 array holding ECEF XYZ coordinates [m]
* @return {array} 1x3 array {latitude [rad], longitude [rad], altitude [m]}
**/
void F_ECEF_TO_GEODETIC(double xi_ecef[3], double xo_llh[3])
{
	// locals
	unsigned int i;
	double N = C_WGS84_MAJOR, alt = 0, h_old = 100, hN = 0, sin_lat, temp, temp2;

	// longitude calculation
	xo_llh[_LON] = M_ATAN2(xi_ecef[_Y], xi_ecef[_X]);


	// iterative latitude and altitude calculation
	for(i = 0; i < 50; ++i) {
		// sine of latitude
		temp    = alt + N * (1.0 - C_WGS84_ECENT_SQR);
		sin_lat = xi_ecef[_Z] / temp;

		// common terms
		temp  = sqrt(xi_ecef[_Y] * xi_ecef[_Y] + xi_ecef[_X] * xi_ecef[_X]);
		temp2 = sin(xo_llh[_LAT]);

		// latitude
		xo_llh[_LAT]  = atan((xi_ecef[_Z] + N * sin_lat * C_WGS84_ECENT_SQR) / temp);

		// altitude
		N  = C_WGS84_MAJOR / sqrt(1.0 + - C_WGS84_ECENT_SQR * temp2 * temp2);
		hN  = temp / cos(xo_llh[_LAT]);
		alt  = hN - N;
	}

	// altitude assignment
	xo_llh[_ALT] = alt;
}

/**
* GEODETIC {latitude, longitude, altitude [positive =  up]} to ECEF {x, y, z}
*
* @param  {array} 1x3 array {latitude [rad], longitude [rad], altitude [m]}
* @return {array} 1x3 array holding ECEF XYZ coordinates [m]
**/
void F_GEODETIC_TO_ECEF(double xi_llh[3], double xo_ecef[3])
{
	// locals
	double temp = sin(xi_llh[_LAT]), N;

	// radius
	N  = C_WGS84_MAJOR / sqrt(1.0 - C_WGS84_ECENT_SQR * temp * temp);

	// ECEF coordinates calculation
	temp  = (N + xi_llh[_ALT]) * cos(xi_llh[_LAT]);
	xo_ecef[_X] = temp * cos(xi_llh[_LON]);
	xo_ecef[_Y] = temp * sin(xi_llh[_LON]);
	xo_ecef[_Z] = sin(xi_llh[_LAT]) * (xi_llh[_ALT] + N * (1.0 - C_WGS84_ECENT_SQR));
}


/**
* quaternion to Euler angles [phi, theta, psi]
*
* @param  {array} 1x4 quaternion
* @return {array} 1x3 Euler angles array {phi, theta, psi}
**/
void F_QUATERNION_TO_EULER(double xi_quat[4], double xo_euler[3])
{
	// locals
	double q0 = xi_quat[_qW],
		   q1 = xi_quat[_qX],
		   q2 = xi_quat[_qY],
		   q3 = xi_quat[_qZ],
		   q02 = q0,
		   q12 = q1,
		   q22 = q2,
		   q32 = q3;
	
	// housekeeping
	q02 *= q0;
	q12 *= q1;
	q22 *= q2;
	q32 *= q3;

	// Euler angles calculation
	xo_euler[_X] = M_ATAN2(2 * (q2 * q3 + q0 * q1), (1-2 * (q12 + q22)));
	xo_euler[_Y] = -asin(2 * (q1 * q3 - q0 * q2));
	xo_euler[_Z] = M_ATAN2(2 * (q1 * q2 + q0 * q3), (1-2 * (q22 + q32)) );
}
	
/**
* convert Euler angles [phi, theta, psi] to quaternion
*
* @param  {double} phi [rad]
* @param  {double} theta [rad]
* @param  {double} psi [rad]
* @return {array}  1x4 quaternion
**/
void F_EULER_TO_QUATERNION(double xo_quat[4], const double xi_phi, const double xi_theta, const double xi_psi)
{
	// locals
	double sinHalfPhi    = sin(0.5*xi_phi),
		    sinHalfTheta = sin(0.5*xi_theta),
			sinHalfPsi   = sin(0.5*xi_psi),
			cosHalfPhi   = cos(0.5*xi_phi),
			cosHalfTheta = cos(0.5*xi_theta),
			cosHalfPsi   = cos(0.5*xi_psi);

	// quaternion calculation
    xo_quat[_qW]  = cosHalfPhi * cosHalfTheta * cosHalfPsi + sinHalfPhi * sinHalfTheta * sinHalfPsi;
    xo_quat[_qX]  = sinHalfPhi * cosHalfTheta * cosHalfPsi - cosHalfPhi * sinHalfTheta * sinHalfPsi;
    xo_quat[_qY]  = cosHalfPhi * sinHalfTheta * cosHalfPsi + sinHalfPhi * cosHalfTheta * sinHalfPsi;
    xo_quat[_qZ]  = cosHalfPhi * cosHalfTheta * sinHalfPsi - sinHalfPhi * sinHalfTheta * cosHalfPsi;
}

/**
* atmospheric properties calculation (valid only up to 36,152ft)
*
* @param  {double} altitude above mean sea level [ft] 
* @return {double] density [slug / ft**3]
* @return {double] pressure [lb / ft**2]
* @return {double] temperature [R]
* @return {double] local speed of sound [ft / sec]
**/
void F_STANDARD_ATMOSPHERE(const double xi_altitude, double *xo_density, double *xo_pressure,
				           double *xo_temperature, double *xo_sound_speed)
{
	// locals
	double altitude;

	// housekeeping
	altitude  = xi_altitude * 1e-5;
	
	// density calculation
	*xo_density = 0.0023769 * pow(1.0 - altitude * (0.68753 -0.003264 * altitude), 4.256);
	
	// temperature calculation
	*xo_temperature  = 518.69 * (1.0 + 0.003298 * altitude * altitude - 0.687532 * altitude);
	
	// pressure calculation
	*xo_pressure  = 1716.5 * (*xo_density) * (*xo_temperature);

	// compute speed of sound
	*xo_sound_speed  = 1116.45 * (1.0 - altitude * (0.68753 -0.003264 * altitude));
}
