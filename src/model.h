#ifndef model_h
#define model_h

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <windows.h>
#include <winbase.h>


#include "matlib.h"
#include "simlib.h"


// Main Rotor Parameters
struct mainrotor_def {
	double	a_sum;					// used in MR TPP dynamics
	double	b1;						// lateral TPP (tail tip plane) deflection wrt hub (+ right tilt rad)
	double	kc;						// flapping stiffness
	double	a1;						// longitudinal TPP (tail tip plane) deflection wrt hub (+ back tilt rad)
	double	db1dv;					// TPP (tail tip plane) lateral dihedral effect derivative
	double	b_sum;					// used in MR TPP (tail tip plane) dynamics
	double	da1du;					// TPP (tail tip plane) longitudinal dihedral effect derivative
	double	a1dot;					// used in MR TPP (tail tip plane) dynamics (longitudinal dynamics)
	double	b1dot;					// used in MR TPP (tail tip plane) dynamics (lateral dynamics)
	double	wr;						// z-axis velocity relative to rotor plane
	double	is;						// pitch shaft tilt (+ aft lean rad)
	double	ib;						// roll shaft tile (+ right lean rad)
	double	wb;						// z-axis velocity relative to blade
	double	omega;					// MR angular velocity (rad/s)
	double	r;						// MR radius (ft)
	double	twst;					// MR blade twist (- for washout rad)
	double	thrust;					// MR total thrust (lb)
	double	vi;						// MR induced velocity (ft/s)
	double	a;						// blade lift curve slope (Cl_alpha */rad)
	double	b;						// # of MR blades
	double	c;						// chord of MR blade (ft)
	double	vhat2;					// used in MR vi and thrust integration
	double	vi2;					// used in MR vi and thrust integration
	double	h;						// MR vertical distance of hub to aircraft CG (ft)
	double	d;						// MR horizontal distance of hub to aircraft CG (ft)
	double	v_tip;					// tip speed (ft/s)
	double	power;					// total MR power (lb-ft/s)
	double	torque;					// total MR torque (lb-ft)
	double	dir;					// MR direction of rotation viewed from top (1 = ccw; -1 = cw)
	double	x;						// MR X force (lb)
	double	y;						// MR Y force (lb)
	double	z;						// MR Z force (lb)
	double	l;						// MR rolling moment on body (lb-ft)
	double	dl_db1;					// Rolling moment due to b1 TPP (tail tip plane) tilt derivative (lb-ft/rad)
	double	k1;						// Delta-3 hinge effect
	double	m;						// MR pitching moment on body (lb-ft)
	double	n;						// MR yawing moment on body (lb-ft)
	double	fs;						// MR horizontal fuse station point (from MR hub in)
	double	wl;						// MR vertical fuse station point (from MR hub in)
	double	e;						// MR equivalent hinge offset (ft)
	double	i_b;					// MR blade inertial about root (slug-ft^2)
	double	cd0;					// profile drag coefficient for a MR blade
	double	ct;						// MR thrust coefficient
	double	ro;						// MR root cutout
	double	cutout;					// MR cutout ratio
	double lock;					// MR lock number
	double omega_f;					// natural frequency due to hinge offset
	double k2;						// cross coupling coefficient
	double w_in;					// in plane change in flap
	double w_off;					// off-plane change in flap
	double tau;						// TPP (tail tip plane) time constant (sec)
	double dm_da1;					// pitching moment due to a1 TPP tile derivative (lb-ft/rad)
	double sigma;					// solidity of rotor disk
};

// Tail Rotor Parameters
struct tailrotor_def{
	double	vr;						// velocity perpendicular to rotor disk
	double	d;						// horizontal distance of hub to aircraft CG (ft)
	double	h;						// vertical distance of hub to aircraft CG (ft)
	double	vb;						// velocity perpendicular to rotor blade
	double	omega;					// angular velocity of rotor (rad/s)
	double	r;						// TR radius (ft)
	double	r0;						// TR cutout radius (ft)
	double	twst;					// TR blade twist (- washout rad)
	double	thrust;					// TR thrust (lb)
	double	vi;						// TR induced velocity
	double	a;						// TR blade lift curve slope (Cl_alpha */rad)
	double	b;						// # of TR blades
	double	c;						// TR blade chord (ft)
	double	vhat2;					// used in thrust integration
	double	vi2;					// used in thrust integration
	double	x;						// TR X force (lb)
	double	y;						// TR Y force (lb)
	double	z;						// TR Z force (lb)
	double	l;						// TR rolling moment (lb-ft)
	double	m;						// TR pitching moment (lb-ft)
	double	n;						// TR yawing moment (lb-ft)
	double	power;					// TR power (lb-ft/s)
	double	fs;						// TR horizontal fuse station point (from MR hub in)
	double	wl;						// TR vertical waterline point (from MR hub in)
	double	rpm;					// TR revolutions per minute
	double	cd0;					// TR blade profile drag coefficient
	double	fr;						// normalized TR drag
	double	duct;					// factor to correct for duct augmentation
};

// Fly Bar (control rotor) Parameters (based on Tischler and Mettler)
struct flybar_def {
	double tau;						// fly bar time constant (sec)
	double d;						// lateral fly bar TPP tilt (rad)
	double d_dot;					// lateral fly bar TPP tilt rate (rad/s)
	double c;						// longitudinal fly bar TPP tilt (rad)
	double c_dot;					// longitudinal fly bar TPP tilt rate (rad/s)
	double Kd;						// lateral flybar->main rotor gearing ratio
	double Kc;						// longitudinal flybar->main rotor gearing ratio
};

// Vertical Fin Parameters
struct verticalfin_def {
	double	va;						// local Y velocity on fin
	double	d;						// horizontal distance of fin to aircraft CG (ft)
	double	vta;					// local air velocity on fin
	double	x;						// fin X force (lb)
	double	y;						// fin Y force (lb)
	double	yuu;					// horizontal flat plate area (ft^2)
	double	yuv;					// horizontal-side flat plate drag area (ft^2)
	double	ymax;					// flat plate drag area of stalled surface (ft^2)
	double	z;						// fin Z force (lb)
	double	h;						// vertical distance of fin to aircraft CG (ft)
	double	m;						// fin pitching moment (lb-ft)
	double	n;						// fin yawing moment (lb-ft)
	double	l;						// fin rolling moment (lb-ft)
	double	fs;						// fin horizontal fuse station point (from MR hub in)
	double	wl;						// fin vertical waterline point (from MR hub in)
	double	a;						// fin lift curve slope (Clalpha */rad)
};

// Horizontal Fin Parameters
struct horizontalfin_def {
	double	d_dw;					// down wash impinging on tail
	double	h;						// vertical distance of fin to aircraft CG (ft)
	double	d;						// horizontal distance of fin to aircraft CG (ft)
	double	eps;					
	double	wa;						// local Z velocity at fin
	double	vta;					// local air velocity at fin
	double	x;						// fin X force (lb)
	double	y;						// fin Y force (lb)
	double	z;						// fin Z force (lb)
	double	zuu;					// horizontal flat plate drag area (ft^2)
	double	zuw;					// horizontal-vertical flat plate drag area (ft^2)
	double	zmax;					// flat plate drag for stalled surface (ft^2)
	double	l;						// fin rolling moment (lb-ft)
	double	m;						// fin pitching moment (lb-ft)
	double	n;						// fin yawing moment (lb-ft)
	double	fs;						// fin horizontal fuse station point (from MR hub in)
	double	wl;						// fin vertical waterline point (from MR hub in)
};

// Fuselage Parameters
struct fuse_def {
	double	wa;						// rotor down wash on fuselage
	double	h;						// vertical distance of fuse center of pressure to CG (ft)
	double	d;						// horizontal distance of fuse center of pressure to CG (ft)
	double	x;						// fuse X force (lb)
	double	xuu;					// horizontal flat plate drag area (ft^2)
	double	y;						// fuse Y force (lb)
	double	yvv;					// side flat plate drag area (ft^2)
	double	z;						// fuse Z force (lb)
	double	zww;					// vertical flat plate drag area (ft^2)
	double	l;						// fuse rolling moment (lb-ft)
	double	m;						// fuse pitching moment (lb-ft)
	double	n;						// fuse yawing moment (lb-ft)
	double	fs;						// fuse CP horizontal station point (from MR hub [in])
	double	wl;						// fuse CP vertical waterline point (from MR hun [in])
};

// Control Surfaces for Model
struct control_def {
	double mr_col;					// main rotor collective (+ right yaw rad)
	double tr_col;					// tail rotor collective (+ climb rad)
	double A1;						// A1 swash plate angle = roll cyclic pitch (rad + right roll)
	double B1;						// B1 swash plate angle = pitch cyclic pitch (rad + nose down)
	double tr_rev;					// tail rotor RPM (rpm)
	double mr_rev;					// main rotor RPM (rpm)
	double gyro_gain;				// tail rotor hobby gyro gain
	unsigned int wow;			    // weight-on-wheel switch (1=on ground, 0=in air)
};

// CG Parameters
struct cg_def {
	double F[3];					// F[X Y Z] forces acting on CG (lb)
	double M[3];							// M[L M N] moments acting about CG (lb-ft)
	double THETA[3];					// THETA[phi theta psi] Euler body angles (rad)
	double pqr[3];						// pqr[p q r] body angular rates (rad/s)
	double uvw[3];						// uvw[u v w] velocity in body axis (ft/s)
	double V[3];						// V[velN velE velD] velocity in TP NED axis (ft/s)
	double NED[3];						// NED[North East Down] position (ft)
	double altitude;					// starting density altitude (ft + up)
	double	wt;							// vehicle weight (lb)
	double	power;						// net power from engine (lb-ft/s)
	double	hp_loss;					// HP lost in transmission (HP)
	double	m;							// mass of vehicle (slugs)
	double	ix;							// Ixx about CG (slug-ft^2)	
	double	ixz;						// Ixz about CG (slug-ft^2)
	double	iy;							// Iyy about CG (slug-ft^2)
	double	iz;							// Izz about CG (slug-ft^2)
	double	fs_cg;						// CG horizontal station point (from MR hub in)
	double	wl_cg;						// CG vertical waterline point (from MR hub in)
	double time;						// simulation time (sec)
};



// Structure for the blade element calculations
struct blade_element_def {
	double a;							// lift curve slope (*/rad)
	double R;							// blade radius (ft)
	double omega;						// rotor angular velocity (rad/s)
	double b;							// number of rotor blades
	double c;							// rotor blade chord (ft)
	double R0;							// blade root cutout (ft)
	double collective;					// blade collective pitch @ 75% R (rad)
	double twst;						// blade twist (- for washout rad)
	double Cd0;							// profile drag coefficient (non dimensional)
	double e;							// Oswald efficiency factor (non dimensional)
	double Vperp;						// perpendicular velocity to rotor disk (ft/s - in direction of T)
	double rho;							// air density (slug/ft^3)
	double T;							// output thrust (lbs)
	double Q;							// output torque (lb-ft)
	double P;							// output power (lb-ft/s)
	double avg_v1;						// output average induced velocity (ft/s)
};
#define	NUMSTATIONS 93					// number of stations to use in blade element (total is always 100)




/********** THE MAIN HELICOPTER STRUCTURE FOR EVERYTHING **********/
// Main Helicopter Parameters
struct heli_def {
	struct mainrotor_def m;
	struct flybar_def fb;
	struct tailrotor_def t;
	struct verticalfin_def v;
	struct horizontalfin_def h;
	struct fuse_def f;
	struct cg_def cg;
	struct blade_element_def MRBE;
	struct blade_element_def TRBE;
	struct control_def c;
	
	struct gear_inputs_def grIn[5];			// [front-right, back-right, front-left, back-left, tail]
	struct gear_outputs_def grOut[5];		// [front-right, back-right, front-left, back-left, tail]
	double num_gear;						// number of gear
	struct servo_inputs_def svIn[4];		// 0=B1 (pitch), 1=A1 (roll), 2=MR Coll, 3=TR Coll.
	struct servo_state_def svX[4];			// 0=B1 (pitch), 1=A1 (roll), 2=MR Coll, 3=TR Coll.
	struct sixdof_fe_inputs_def sixdofIn;	// use flat-earth sixdof model
	struct sixdof_fe_state_def sixdofX;		// use flat-earth sixdof model
};

extern struct heli_def heli;

/********** FUNCTIONS ******************/

extern double model_dt;		// This is the integration time step for the prorogation of the model.

/*	This will do a combined blade element momentum theory thrust, power, torque computation
* on a rotor.  The inputs to the function are all of the input elements of 
* blade_element_def.  Only T, P, Q, and average v are outputs.
*
*	The directions are as follows...
*	Vperp = perpendicular velocity to rotor disk (+ opposite direction of thrust)
*	avg_v1 = average induced velocity (+ opposite direction of thrust)
*
*	Source:  Keys, Pgs 96-98
*			 Proudy, Pg 96
*
*	See the structure for more information on the input and output parameters.
*/
void F_ROTOR_CALCULATION(struct blade_element_def *pBe);


/*	This will perform the forces and moments calculations on the aircraft
* before the calculations of the 6-DOF.  The landing gear calculations are done
* after this function.  The servo and wind models are run after this as well.
*/
void F_FORCES_MOMENTS(struct heli_def *heli);

/*	This will perform the landing gear calculations with the landing gear
* model provided in the simulation library.  
*/
void F_LANDING_GEAR(struct heli_def *heli);

/*	This will perform the time stepping of the servo model.
* The values of U[7] are as follows...
*	U[0] = main rotor collective (rad)
*	U[1] = A1 swash plate tilt (+ right roll) (rad)
*	U[2] = B1 swash plate tilt (+ nose down) (rad)
*	U[3] = tail rotor collective (+ right yaw)
*
*  The svIn[3] and svX[3] are as follows...
*	sv[0] = B1 (pitch servo)
*	sv[1] = A1 (roll servo)
*	sv[2] = main rotor collective
*	sv[3] = tail rotor collective (petal)	
*/
void F_ACTUATOR(struct heli_def *heli, 
			  double U[4]);


/*	This will perform the initialization of the entire model. It is also the function
* to call to reset the model.  For the case of resetting, more than the minimum calculations
* are done, but this minimizes the number of functions to deal with.
*/
void F_MODEL_INIT(void);

/*	This will perform the entire calculations of everything that needs to happen to 
* make the math model of the vehicle work.  This is the function to call to
* propagate the helicopter model, 6-DOF, landing gear, servos.
*
*	U[4] = [mr_coll, A1, B1, tr_coll]
*/
void ModelGO(double U[4]);



#endif

