#ifndef matlib_h
#define matlib_h

/**
* constants
**/
#define C_FOOT_TO_METER    0.3048          
#define C_METER_TO_FOOT    3.280839895     
#define C_PI               3.1415926535897932384626433832795
#define C_TWOPI            6.283185307179586476925286766559     // 2 * PI
#define C_FOUR_PI          12.566370614359172953850573533118    // 4 * PI
#define C_EIGHT_PI		   25.132741228718345907701147066236    // 8 * PI
#define C_PI_DIV_12		   0.26179938779914943653855361527329   // PI / 12
#define C_RPM2RADSEC	   0.10471975511965977461542144610932   // 2 * PI / 60
#define C_DEG2RAD          0.017453292519943295769236907684886
#define C_RAD2DEG          57.295779513082320876798154814105
#define C_FPS_TO_KNOT      0.5924838
#define C_WGS84_MAJOR	   6378137.0		                    // WGS-84 semi major axis [m], a
#define C_WGS84_MINOR	   6356752.3142	                        // WGS-84 semi minor axis [m], b
#define C_WGS84_ECENT_SQR  0.006694380004261                    // WGS-84 eccentricity squared (e**2, e= sqrt(2 * f - f**2), f = (a - b) / a)
#define C_GM               3.9860015E14 	                    // WGS-84 gravity model constant [m**3 / s**2]
#define C_J2               0.001082629989052                    // WGS-84 gravity model constant (= -sqrt(5.0) * -0.48416685E-3)
#define C_EARTH_SPIN       7.2722052166430399038487115353692e-5 // earth rotational velocity [rad/sec]
#define C_ONE_THIRD        0.33333333333333333333333333333333	// |
#define C_ONE_SIXTH        0.16666666666666666666666666666667	//  \ RK4 coefficients

/**
* sign of a number (zero not included)
*
* @param  {numeric} a number
* @return {numeric}  sign {-1, +1} of number
**/
#define M_SIGN(xi_in) ((xi_in > 0) ? (1) : (-1))

/**
* sign of a number (zero included)
*
* @param  {numeric} a number
* @return {numeric}  sign {-1, 0, +1} of number
**/
#define M_ZSIGN(xi_in) ((xi_in > 0) ? (1) : ((xi_in < 0) ? (-1) : (0)))

/**
* absolute value return
*
* @param  {numeric} absolute value of numeric input
* @return {numeric} absolute value of numeric input
**/
#define M_ABS(xi_in) ((xi_in >= 0) ? (xi_in) : (-xi_in))

/**
* maximum value of two number
*
* @param  {numeric} a number
* @param  {numeric} a number
* @return {numeric} the largest number
**/
#define M_MAX2(xi_x, xi_y) ((xi_x > xi_y) ? (xi_x) : (xi_y))

/**
* minimum value of two numbers
*
* @param  {numeric} a number
* @param  {numeric} a number
* @return {numeric} the smallest number
**/
#define M_MIN2(xi_x, xi_y) ((xi_x > xi_y) ? (xi_y) : (xi_x))

/**
* clamping function
*
* @param  {numeric} a number
* @param  {numeric} lower value
* @param  {numeric} upper value
* @return {numeric} {@xi_in} limited in region [{@xi_min}, {@xi_max}]
**/
#define M_LIMIT(xi_in, xi_min, xi_max) ((xi_in) > (xi_max) ? (xi_max) : ((xi_in) < (xi_min) ? (xi_min) : (xi_in)))

/**
* symmetrical clamping
*
* @param  {numeric} a number
* @param  {numeric} limit value
* @return {numeric} {@xi_in} limited in region [-{@xi_clip}, {@xi_clip}]
**/
#define M_LIMIT_SYM(xi_in, xi_clip) (((xi_in) > (xi_clip)) ? (xi_clip) : (((xi_in) < (-xi_clip)) ? (-xi_clip) : (xi_in)))

/**
* some compilers and hardwares (Intel) propagate the sign bit along with zero (0.0),
* this can cause several problems, for example:
*        atan2(+0.0, +0.0) = atan2(-0.0, +0.0) = 0
*        atan2(+0.0, -0.0) = atan2(-0.0, -0.0) = PI
* the solution: the "==" operator thinks that -0.0 == +0.0,
* so I will use it to return an unambiguous value for zero (a sign less zero!)
*
* @param  {numeric} a number
* @return {numeric} if the number is zero then it is a positive zero
**/
#define M_FIX_ZERO_SIGN(xi_value) (((xi_value) == (0.0)) ? (0.0) : (xi_value))

/**
* atan2 with fixed zero sign
*
* @param  {numeric} X value
* @param  {numeric} Y value
* @return {numeric} atan2({@xi_a}, {@xi_b}) with zero sign correction
**/
#define M_ATAN2(xi_a, xi_b) (atan2(M_FIX_ZERO_SIGN(xi_a), M_FIX_ZERO_SIGN(xi_b)))

/**
* transpose a 3x3 matrix and assign into another
*
* @return {numeric} 3x3 input matrix
* param   {numeric} matrix to be transformed and assigned
**/
#define M_MAT_TRANSPOSE(xio_matrix, xi_matrix) {(xio_matrix)[_X][_X] = (xi_matrix)[_X][_X]; (xio_matrix)[_X][_Y] = (xi_matrix)[_Y][_X]; (xio_matrix)[_X][_Z] = (xi_matrix)[_Z][_X]; \
											    (xio_matrix)[_Y][_X] = (xi_matrix)[_X][_Y]; (xio_matrix)[_Y][_Y] = (xi_matrix)[_Y][_Y]; (xio_matrix)[_Y][_Z] = (xi_matrix)[_Z][_Y]; \
											    (xio_matrix)[_Z][_X] = (xi_matrix)[_X][_Z]; (xio_matrix)[_Z][_Y] = (xi_matrix)[_Y][_Z]; (xio_matrix)[_Z][_Z] = (xi_matrix)[_Z][_Z];}

/**
* square a value
*
* @param  {numeric} a value
* @return {numeric} square of input
**/
#define M_SQR(xi_in) ((xi_in) * (xi_in))


/**
* function deceleration
**/
void F_EULER_TO_DCM(double tBL[3][3], double phi, double theta, double psi);
void F_QUATERNION_TO_DCM(double tBL[3][3], double q[4]);
void F_QUATERNION_NORMALIZATION(double q[4]);
void F_GEODETIC_TO_ECEF_2args(double llh[3], double ECEF[3]);
void F_GEODETIC_TO_ECEF_4args(double xo_ecef[3], double xi_local[3], double xi_latitude, double xi_longitude);
void F_ECEF_TO_GEODETIC(double llh[3], double ECEF[3]);
void F_QUATERNION_TO_EULER(double q[4], double euler[3]);
void F_EULER_TO_QUATERNION(double q[4], const double phi, const double theta, const double psi);
void F_STANDARD_ATMOSPHERE(const double altitude, double *density, double *pressure, double *temp, double *sp_sound);


#endif
