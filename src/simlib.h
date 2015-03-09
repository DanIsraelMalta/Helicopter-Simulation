#ifndef simlib_h
#define simlib_h

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "matlib.h"


/**
* "enumerations" for easy array access
**/

// north/east/down
#define _NORTH	0
#define _EAST	1
#define _DOWN	2

// X/Y/Z
#define _X	0
#define _Y	1
#define _Z	2

// latitude, longitude, altitude
#define _LAT  0
#define _LON  1
#define _ALT  2

// roll, pitch, yaw (Euler names)
#define _PHI    0
#define _THETA  1
#define _PSI    2

// lift, drag, side
#define _LIFT  0
#define _DRAG  1
#define _SIDE  2

// moment around x, y, z axis
#define _L  0
#define _M  1
#define _N  2

// quaternion elements
#define _qW  0
#define _qX  1
#define _qY  2
#define _qZ  3

// actuators
#define _B1	 0  // B1 (pitch)
#define _A1	 1  // A1 (roll)
#define _MR	 2	// main rotor collective
#define _TR	 3  // tail rotor collective

// skids / gear
#define _GEAR_RF  0  // right-front gear
#define _GEAR_RB  1  // right-back gear
#define _GEAR_LF  2	 // left-front gear
#define _GEAR_LB  3  // left-back gear
#define _GEAR_T   4  // tail gear

/*	This is an IMU emulation function.  Given some geometric information
* on the position of the IMU relative the the position of the aircraft,
* a new inertial solution can be solved.  The IMU that is being 
* simulated is a full-state strap down IMU.  It is capable of delivering
* acceleration, velocity, position, angular rate, and attitude at the 
* IMU's current location and orientation.  
*
* See the structures below for the input and output arguments.  All of the
* input arguments must be defined.  Units are commented next to each of 
* the values.
*/

struct imu_inputs_def {
	double cg2imu[3];					// position vector from CG->IMU body axis [X Y Z] (ft)
	double body2imu[3][3];		// rotation matrix body axis->IMU axis
	double cg_accel[3];				// body axis accel. at CG [X Y Z] (ft/s/s)
	double cg_uvw[3];					// body axis velocity at CG [X Y Z] (ft/s)
	double cg_pos[3];					// ECEF position at CG [X Y Z] (ft)
	double cg_alpha[3];				// body axis angular accel. at CG [Pdot Qdot Rdot] (rad/s/s)
	double cg_pqr[3];					// body axis angular rates at CG [P Q R] (rad/s)
	double cg_THETA[3];				// body axis attitude at CG [phi theta psi] (rad)
};

struct imu_outputs_def {
	double accel[3];					// IMU axis accel at IMU [X Y Z] (ft/s/s)
	double uvw[3];					// IMU axis velocity at IMU [X Y Z] (ft/s)
	double ECEFpos[3];				// ECEF position of IMU [X Y Z] (ft)
	double LLHpos[3];					// LLH position of IMU [latitude longitude altitude] (rad)(rad)(ft MSL + up)
	double pqr[3];					// IMU axis angular rate at IMU [P Q R] (rad/s)
	double THETA[3];					// IMU axis attitude at IMU [phi theta psi] (rad)
};


/*	This is a 4th order Runge-Kutta integration routine.  It is designed to work
* with a vector of inputs, and produce a vector of outputs at the next time step.
* A time step input is needed for the dt value, and the number of states to integrate.
* There is the requirement that the function to be integrated is in a specific format...
*
*	RK4(double *state,					// state vector (INPUT and OUTPUT) 
*		double *state_dot,				// derivatives of state vector (OUTPUT)
*		double *force,					// any forcing values (INPUT)
*		double t,						// current time (INPUT)
*		double *args,					// any arguments for the called derivative function (INPUT)
*		int n,							// number of states
*		double dt,						// time step
*		void (*pfunc)					// pointer to called function (INPUT)
*		(double *Xdot,					// returned state derivative (OUTPUT)
*		 double *X,						// state vector (INPUT called by RK4)
*		 double *t,						// time (INPUT called by RK4)
*		 double *U,						// passed forcing values (INPUT called by RK4)
*		 double *args) );				// passed arguments for derivative function (INPUT called by RK4)
*
*	The state out of this function is the state at t+dt.
*/
void F_RK4(double *state, double *state_dot, double t, double *force, double *args, int n, double dt, 
		 void (*pfunc)(double *Xdot, double *X, double *t, double *U, double *args) );



/*	This is a 6 degree-of-freedom (6-DOF) rigid body simulation.  The values
* that are needed are listed in the input structure.  The other structure, the
* state structure needs to be initialized with the desired starting values.  
* An Init function is provided to help with this initialization.  It only needs
* to be called once.  The structure for the Init function is sixdof_init_inputs_def.
*	It is noted that this simulation is based on quaternion parameters.  For this 
* reason, euler2quat function needs to be used for the attitude initialization.
* Latitude, longitude, and altitude (rad)(rad)(ft MSL + up) need to be used for
* the initialization.  ECEF is generated from this.  Units are listed in the
* structure comments.  
*	Prorogation of the state vector is done with RK4.
*
*	ALL STATE VALUES ARE ASSUMED TO BE AT THE CG OF THE VEHICLE!!!
*
*	Initialization function:
*	sixdof_init(struct sixdof_init_inputs_def *pInit,		// see the structure (INPUT)
*				 struct sixdof_inputs_def *pIn,				// see the structure (RETURN)
*				 struct sixdof_state_def *pX);				// see the structure, assuming no acceleration (RETURN)
*
*	Sixdof function:
*	void sixdof(struct sixdof_state_def *pX,				// see structure (INPUT and RETURN) 
*				struct sixdof_inputs_def *pU,				// see structure (INPUT)
*				double dt);									// time step for integration (sec)
*/
struct sixdof_init_inputs_def {
	double Ixx;								// moment of inertia about body X axis (slug-ft^2)
	double Iyy;								// moment of inertia about body Y axis (slug-ft^2)
	double Izz;								// moment of inertia about body Z axis (slug-ft^2)
	double Ixz;								// cross moment of inertia about body XZ axis (slug-ft^2)
	double m;								// mass of vehicle (slug)
	double latitude;						// starting geodetic latitude (rad)
	double longitude;						// starting geodetic longitude (rad)
	double altitude;						// starting altitude (ft MSL + up)
	double uvw[3];							// starting body translational velocity [u v w] (ft/s)
	double pqr[3];							// starting body angular rates [P Q R] (rad/s)
	double THETA[3];						// starting body attitude [phi theta psi] (rad) (level to TP)
};

struct sixdof_inputs_def {
	double J[3][3];							// inertia tensor matrix (slug-ft^2)
	double Jinv[3][3];						// inverse of inertia tensor matrix (slug-ft^2)
	double m;								// vehicle mass (slug)
	double F[3];							// total force on vehicle CG (lbs)
	double M[3];							// total moment on vehicle CG (lbs-ft)
	double hold_u;							// hold the X body velocity constant (1=hold, 0=free)
	double hold_v;							// hold the Y body velocity constant (1=hold, 0=free)
	double hold_w;							// hold the Z body velocity constant (1=hold, 0=free)
	double hold_p;							// hold the X body angular rate constant (1=hold, 0=free)
	double hold_q;							// hold the Y body angular rate constant (1=hold, 0=free)
	double hold_r;							// hold the Z body angular rate constant (1=hold, 0=free)
};

struct sixdof_state_def {
	double accel[3];					// acceleration body frame at cg [X Y Z] (ft/s/s)
	double Vb[3];						// velocity body frame at cg [u v w] (ft/s)
	double Ve[3];						// velocity TP frame at cg [Vnorth Veast Vdown] (ft/s)
	double Pecef[3];					// position ECEF frame at cg [X Y Z] (ft)
	double Pllh[3];					// position LLH frame at cg [lat lon alt] (rad)(rad)(ft MSL + up)
	double alpha[3];					// angular acceleration body frame [Pdot Qdot Rdot] (rad/s/s)
	double rate[3];					// angular rates body frame [P Q R] (rad/s)
	double Q[4];						// quaternion [q0 q1 q2 q3]
	double THETA[3];					// attitude euler angles [phi theta psi] (rad) (level to TP)
};

void sixdof_init(struct sixdof_init_inputs_def *pInit, 
				 struct sixdof_inputs_def *pIn,
				 struct sixdof_state_def *pX);

void sixdof(struct sixdof_state_def *pX, struct sixdof_inputs_def *pU, double dt);


/*	This is a six degree of freedom rigid body dynamic simulator.  It is
* similar the above sixdof() function, but instead of using WGS-84 round
* earth simulation, this will use the flat-earth approximations.  The inputs
* are similar in both cases, except that for this simulation, no information is needed
* on the home latitude and longitude.  All that is needed is the starting 
* NED (north east down) position.  The main difference is that altitude is now + down
* not up.  
*	Prorogation of the dynamics is done with RK4.
*
*	NOTE: ALL VALUES ARE AT THE CG OF THE VEHCILE!
*
*	Initialization Function:
*	sixdof_fe_init(struct sixdof_fe_init_inputs_def *pInit,		// see structure (INPUT)
*				struct sixdof_fe_inputs_def *pIn,				// see structure (OUTPUT)
*				struct sixdof_fe_state_def *pX)					// see structure (OUTPUT)
*
*	Prorogation Function:
*	sixdof_fe(struct sixdof_fe_inputs_def *pIn,					// see structure (INPUT)
*			struct sixdof_fe_state_def *pX,						// see structure (INPUT/OUTPUT)
*			double dt)											// integration time step (sec)
*/
struct sixdof_fe_init_inputs_def {
	double Ixx;								// moment of inertia about body X axis (slug-ft^2)
	double Iyy;								// moment of inertia about body Y axis (slug-ft^2)
	double Izz;								// moment of inertia about body Z axis (slug-ft^2)
	double Ixz;								// cross moment of inertia about body XZ axis (slug-ft^2)
	double m;								// mass of vehicle (slug)
	double NED[3];					// starting body NED [north east down] position (ft)
	double uvw[3];					// starting body translational velocity [u v w] (ft/s)
	double pqr[3];					// starting body angular rates [P Q R] (rad/s)
	double THETA[3];					// starting body attitude [phi theta psi] (rad) (level to TP)
};

struct sixdof_fe_inputs_def {
	double J[3][3];							// inertia tensor matrix (slug-ft^2)
	double Jinv[3][3];						// inverse of inertia tensor matrix (slug-ft^2)
	double m;								// vehicle mass (slug)
	double F[3];							// total force on vehicle CG (lbs)
	double M[3];							// total moment on vehicle CG (lbs-ft)
	double hold_u;							// hold the X body velocity constant (1=hold, 0=free)
	double hold_v;							// hold the Y body velocity constant (1=hold, 0=free)
	double hold_w;							// hold the Z body velocity constant (1=hold, 0=free)
	double hold_p;							// hold the X body angular rate constant (1=hold, 0=free)
	double hold_q;							// hold the Y body angular rate constant (1=hold, 0=free)
	double hold_r;							// hold the Z body angular rate constant (1=hold, 0=free)
};

struct sixdof_fe_state_def {
	double accel[3];					// acceleration body frame at cg [X Y Z] (ft/s/s)
	double Vb[3];						// velocity body frame at cg [u v w] (ft/s)
	double Ve[3];						// velocity TP frame at cg [Vnorth Veast Vdown] (ft/s)
	double NED[3];						// position NED [north east down] (ft)
	double alpha[3];					// angular acceleration body frame [Pdot Qdot Rdot] (rad/s/s)
	double rate[3];						// angular rates body frame [P Q R] (rad/s)
	double Q[4];						// quaternion [q0 q1 q2 q3]
	double THETA[3];					// attitude Euler angles [phi theta psi] (rad) (level to TP)
};

void F_SIX_DOF_INIT(struct sixdof_fe_init_inputs_def *pInit, 
				 struct sixdof_fe_inputs_def *pIn,
				 struct sixdof_fe_state_def *pX);

void F_SIX_DOF(struct sixdof_fe_state_def *pX, struct sixdof_fe_inputs_def *pU, double dt);


/*	This function will provide a servo-actuator model.  It contains two main
* components.  The first is a second-order dynamic model which is tunable 
* via wn and zeta values.  The second, is a hysteresis model.  This feature
* is designed around basic dead band slop that may be found in system linkages.
*	See the structure for details.  Also, this system contains a state structure
* that needs to be maintained for state prorogation.
*
*	void servo(struct servo_state_def *pX,			// state prorogation 
*				struct servo_inputs_def *pIn,		// inputs for the model
*				double dt);							// time step size for integration of dynamics
*
*	The TF model of the servo used is as follows...
*
*			          S + wn^2           y
*            ------------------------ =  --
*            S^2 + 2*zeta*wn*S + wn^2    u
*
*	Nominal values of wn and zeta are...  wn = 10.43; zeta = 0.8;  (for KR2000 actuators)
*
* Use this equation to find the appropriate value of wn assuming zeta = 0.8...
*
*  wn =          pi
*        ------------------     Tp = peek time (ex: servo takes 0.6 sec to go 60 deg, Tp = 0.6)
*        Tp*sqrt(1 - zeta^2)
*
*/
struct servo_inputs_def {
	double wn;								// natural freq. of the dynamic model
	double zeta;							// damping ratio of the dynamic model
	double slop;							// half of the total slop in the system for hysterisous (ie: +/- slop)
	double command;							// driving command of the servo
};

struct servo_state_def {
	double X[2];							// state vector (internal)
	double output;							// resulting output value of the command into system
};

void F_SERVO_MODEL(struct servo_state_def *pX, struct servo_inputs_def *pIn, double dt);


/*	This is a collision/landing gear model.  The basis of this model is the 
* vertical displacement method.  The idea is that if a collision is detected between
* the point, and a surface, then the displacement is computed.  The is then 
* used to compute a force based on Hook's law.  To allow for in-elastic collisions,
* a damping constant can be given.
*	For now, this function assumes that the contact surface is the ground (for landing gear).
* For this assumption, the ground is taken as 0 altitude.
*	For more information, see the structure definitions below.
*/
struct gear_inputs_def {
	double cg2point[3];						// the vector from vehicle CG -> contact pt, body frame [X Y Z] (ft)
	double THETA[3];						// vehicle attitude [phi theta psi] (rad) (TP level)
	double pqr[3];							// vehicle body rates [P Q R] (rad/s)
	double uvw[3];							// vehicle body velocity [u v w] (ft/s)
	double altitude;						// vehicle CG altitude above the local level TP (ft MSL + down)
	double k;								// Hook's law spring constant (lbs/ft)
	double b;								// damping constant (lbs/ft/s)
	double mu_x;							// coefficient of friction on long axis
	double mu_y;							// coefficient of friction on cross axis
	double rotation;						// relative rotation about vertical axis (for steering) (rad)
};

struct gear_outputs_def {
	double F[3];						// forces on vehicle CG caused by the gear (body axis) [X Y Z] (lbs)
	double M[3];						// moments on vehicle CG caused by gear (body axis) [L M N] (lb-ft)
};

void F_GEAR_COLLISION(struct gear_inputs_def *pIn, struct gear_outputs_def *pOut);


/*	This is a basic wind/gust model.  It is a dynamic system that will generate
* random winds, up to a maximum value, in both vertical and horizontal directions.
*	To make this model work, like the servo and 6-DOF models, it is propagated over
* time for the dynamics.
*	See the structures for information
*
*	There are two functions.  The first, wind_init is used to initialize the wind model.
* The second, wind, is used to propagate the state of the wind model.
*
*	void wind_init(struct wind_inputs_def *pIn,		// input values (INPUTS and OUTPUT)
*					struct wind_state_def *pX);		// state values (OUTPUT)
*
*	void wind(struct wind_inputs_def *pIn,			// input values (INPUTS)
*			struct wind_state_def *pX,				// state values (INPUTS and OUTPUTS)
*			double dt);								// time step (sec) (INPUTS)
*
*/
struct wind_inputs_def {
	double horz_max;						// maximum horizontal wind (ft/s)
	double vert_max;						// maximum up/down drafts (ft/s)
	unsigned int seed;						// seed variable for the random number generator
};
struct wind_state_def {
	double Ve[3];							// components of wind in earth TP frame [Vn Ve Vd] (ft/s)
	double X[6];							// internal state
};

void F_WIND_MODEL_INIT(struct wind_inputs_def *pIn, struct wind_state_def *pX);
void F_WIND_MODEL(struct wind_inputs_def *pIn, struct wind_state_def *pX, double dt);


/*	This is a gravity and gravitation model.  It is based on the WGS-84 ellipsoid
* model of the Earth.  The source for this model is from Dr. Brian Steven's class.
* See the structures for information.
*/
struct grav_inputs_def {
	double latitude;						// current geodetic latitude (rad)
	double altitude;						// current altitude (ft MSL + up)
};
struct grav_outputs_def {
	double g[3];						// local gravity vector in earth TP frame [gN gE gD] (ft/s/s)
	double G[3];						// local gravitation vector in earth TP frame [GN GE GD] (ft/s/s)
};
void F_GRAVITY_MODEL(struct grav_inputs_def *pIn, struct grav_outputs_def *pOut);

#endif  
