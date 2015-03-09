/**
* General Helicopter Simulation
**/

#include "model.h"
#include "simlib.h"


/**
* globals
**/

struct heli_def heli;			// A/V definition structure
double model_dt = 0.001;		// model time step [sec]

/**
* prototype
**/
void F_ROTOR_FLAP_MODEL(double Xdot[2], double X[2], double *t, double A1B1[2], double args[13]);


/**
* A linear rotor flapping model accounting for coupled servo rotor dynamics
* and mechanical feedback by the gyroscopic effect of stabilizer bar, built in the following manner:
*	Xdot = [a1 b1 d c]
*	X    = [a1 b1 d c]
*	A1B1 = [A1 B1]
*	args = [u v p q db1dv da1du w_in w_off kc dir tau Kc Kd]
*
*
* Fly bar Information (Tischler and Mettler, System Identification Modeling Of a Model-Scale Helicopter, page 10, equations (8)->(13) )
*
* @return {double} flapping model state derivative
* @param  {double} flapping model state
* @param  {double} time [sec]
* @param  {double} flapping model inputs
* @param  {double} flapping model parameters
**/
void F_ROTOR_FLAP_MODEL(double Xdot[4], double X[4], double *t, double A1B1[2], double args[13]) {
	// locals
	double u = args[0],				// body velocity
		   v = args[1],				// body velocity
		   p = args[2],				// appear as feedback term due to gyroscopic effect of the stabilizer bar of the servo rotor system
		   q = args[3],				// appear as feedback term due to gyroscopic effect of the stabilizer bar of the servo rotor system
		   db1dv = args[4],			// 
		   da1du = args[5],			// 
		   w_in = args[6],			// 
		   w_off = args[7],			// 
		   kc = args[8],			// 
		   dir = args[9],			// 
		   tau = args[10],			// time constant of servo rotor response to swash plate tilt angle
		   Kc = args[11],			// 
		   Kd = args[12],			// 
		   a1 = X[0],				// 
		   b1 = X[1],				// 
		   d = X[2],				// 
		   c = X[3],				// 
		   A1 = A1B1[0] + Kd * d,	// lateral mixing (+ fly bar mixing)
		   B1 = A1B1[1] + Kc * c,   // longitudinal mixing (+ fly bar mixing)
		   a_sum, b_sum,
		   a1dot, b1dot, 
		   d_dot, c_dot;

	// A & B sum calculation
	a_sum = b1 - A1 + dir * kc * a1 - 0.3 * db1dv * v;
	b_sum = a1 + B1 - dir * kc * b1 - 0.3 * da1du * u;

	// state derivative calculation
	a1dot = -w_in * b_sum - w_off * a_sum - q;
	b1dot = -w_in * a_sum + w_off * b_sum - p;
	d_dot = -d / tau - p + 0.2731 * A1B1[0] / tau;
	c_dot = -c / tau - q - 0.2587 * A1B1[1] / tau;

	// state derivative assignment
	Xdot[0] = a1dot;
	Xdot[1] = b1dot;
	Xdot[2] = d_dot;
	Xdot[3] = c_dot;
}

/**
* Combined finite element and blade momentum theory for the calculation of a blade:
* - thrust
* - power
* - torque
*
* Source: Keys, Pgs 96-98; Proudy, Pg 96
*
* @param/return {struct} blade parameters
**/
void F_ROTOR_CALCULATION(struct blade_element_def *pBe) {
	// locals
	double _dR,									// blade element thickness
	       _r2R,								// ratio of local radius to total radius
	       _r,									// local radius
	       _thetaLocal,							// local collective angle
	       _theta0,								// root collective angle
	       _omegaLocal,							// local angular velocity
	       _alphaLocal,							// local angle of attack
	       _inducedVelocity[NUMSTATIONS],		// local induced velocity
	       _thrustIncrement[NUMSTATIONS],		// increment of thrust
	       _profileDragIncrement[NUMSTATIONS],	// increment of profile drag
	       _torqueIncrement[NUMSTATIONS],		// increment of torque
	       _powerIncrement[NUMSTATIONS],		// increment of power
	       _vv;									// a common term (a * b * c * Omega / 2 + 4 * pi * Vperp)
	unsigned int n, i;                          // current element and previous element index


	// housekeeping
	pBe->T      = 0.0;
	pBe->Q      = 0.0;
	pBe->P      = 0.0;
	pBe->avg_v1 = 0.0;

	// common term calculation (= a * b * c * Omega / 2 + 4 * pi * Vperp)
	_vv = 0.5 * pBe->a * pBe->b * pBe->c * pBe->omega + 4.0 * C_PI * pBe->Vperp;

	// blade element thickness calculation (= (R - R0) / 100)
	_dR = (pBe->R - pBe->R0) / 100.0;

	// blade root collective angle calculation (= |collective| - (0.75 - R0/R) * twist)
	_theta0 = M_ABS(pBe->collective) - pBe->twst * (0.75 - pBe->R0 / pBe->R);

	// calculate thrust / torque / power / drag on all blade elements
	for(n = 1; n <= NUMSTATIONS; ++n) {
		// previous element (n-1)
		i  = n - 1;

		// local radius to total radius ratio (= (R0 + n * dR) / R)
		_r2R = (pBe->R0 + (double)(n) * _dR) / pBe->R; 

		// local radius ( = _r2r * R)
		_r = _r2R * pBe->R;

		// local collective angle ( = root collective + twist * _r2r)
		_thetaLocal = _theta0 + _r2R * pBe->twst;

		// local velocity (= radius * angular velocity)
		_omegaLocal = pBe->omega * _r;

		// local angle of attack (= collective angle - perpendicular velocity / angular velocity)
		_alphaLocal = _thetaLocal - pBe->Vperp / _omegaLocal;

		// local induced velocity (Proudy pg 96)
		// _inducedVelocity[i] = ( -_vv + sqrt(M_ABS( M_SQR(_vv) + 8.0*C_PI*pBe->b*M_SQR(pBe->omega)*pBe->a*pBe->c*_r*_alphaLocal )))/(8.0*C_PI);
		_inducedVelocity[i] = (sqrt(M_ABS(_vv * _vv + C_EIGHT_PI * pBe->b * pBe->omega * pBe->omega * pBe->a * pBe->c * _r * _alphaLocal)) - _vv) / C_EIGHT_PI;

		// local thrust (Keys eq 3.11)
		_thrustIncrement[i] = 4.0*C_PI*pBe->rho*(pBe->Vperp + _inducedVelocity[i])*_inducedVelocity[i]*_r*_dR;
		
		// local profile drag (Keys eq 3.5)
		_profileDragIncrement[i] = 0.5*pBe->Cd0*pBe->rho*M_SQR(_omegaLocal)*pBe->c*_dR;
		
		// local torque (Keys eq 3.9a)
		_torqueIncrement[i] = (_thrustIncrement[i]*(pBe->Vperp + _inducedVelocity[i])/_omegaLocal + _profileDragIncrement[i])*_r;

		// local power increment (Keys eq 3.9b)
		_powerIncrement[i] = _torqueIncrement[i] * pBe->omega;
	}

	// sum all elements into a complete blade
	for(n = 0; n < NUMSTATIONS; ++n) {
		pBe->T      += _thrustIncrement[n];
		pBe->Q      += _torqueIncrement[n];
		pBe->P      += _powerIncrement[n];
		pBe->avg_v1 += _inducedVelocity[n];
	}

	// calculate average induced velocity
	pBe->avg_v1  = pBe->avg_v1 / (double)(NUMSTATIONS);
	
	// if collective angle is negative, transform velocity and thrust direction
	if(pBe->collective < 0.0) {
		pBe->T      *= -1.0;
		pBe->avg_v1 *= -1.0;
	}
}


/**
* A/V model initialization / reset
**/
void F_MODEL_INIT(void) {
	// locals
	struct sixdof_fe_init_inputs_def sixinit;
	double rho, temp, pres, sp_sound;
	int _sizeOfDouble3 = 3 * sizeof(double);

	// --- A/V raw data ---
	// CG information
	heli.cg.fs_cg = 0.0;					// CG horizontal station point (from main rotor hub) [in]
	heli.cg.wl_cg = 10.91;					// CG vertical station point   (from main rotor hub) [in]
	heli.cg.wt = 19.5;						// A/V weight [lbs]
	heli.cg.ix = 0.2184;					// Ixx about CG [slug-ft**2]
	heli.cg.iy = 0.3214;					// Iyy about CG [slug-ft**2]
	heli.cg.iz = 0.4608;					// Izz about CG [slug-ft**2]
	heli.cg.ixz = 0.0337;					// Ixz about CG [slug-ft**2]
	heli.cg.hp_loss = 0.1;					// power lost in transmission [HP]
	heli.cg.m = heli.cg.wt / 32.17417;      // A/V mass [slugs]
	heli.cg.altitude = 0.0;					// density altitude start height [ft]
	
	// Main Rotor Information
	heli.m.fs = 0.0;						// main rotor horizontal fuselage station point (from main rotor hub) [in]
	heli.m.wl = 0.0;						// main rotor vertical fuselage station point (from main rotor hub) [in]
	heli.m.is = 0.0;						// longitudinal shaft tilt [rad]
	heli.m.e = 0.0225;						// main rotor equivalent hinge offset [ft]
	heli.m.i_b = 0.0847;					// main rotor blade inertial, about root [slug-ft**2]
	heli.m.r = 2.25;						// main rotor radius [ft]
	heli.m.ro = 0.6;						// main rotor root cutout [ft]
	heli.m.a = 6.0;							// blade lift curve slope (Cl_alpha * 1/rad)
	heli.m.cd0 = 0.01;						// main rotor blade profile drag coefficient
	heli.m.b = 2;							// number of main rotor blades
	heli.m.c = 0.1979;						// main rotor chord [ft]
	heli.m.twst = 0.0;						// main rotor blade twist [rad]
	heli.m.k1 = 0;							// delta-3 hinge	
	heli.m.dir = -1.0;						// MR direction of rotation viewed from top (1 = CCW; -1 = CW)
	heli.m.ib = 0.0;						// lateral shaft tilt (+ = right lean) [rad]

	// Fly bar Information (Tischler and Mettler, System Identification Modeling
	// Of a Model-Scale Helicopter, page 10, equations (8)->(13) )
	heli.fb.tau = 0.36;						// Fly bar time constant [sec]
	heli.fb.Kd = 0.3;						// lateral Fly bar -> main rotor gearing ratio
	heli.fb.Kc = 0.3;						// longitudinal Fly bar -> main rotor gearing ratio

	// Fuselage Information
	heli.f.fs = 3.0;						// fin horizontal fuselage station point (from main rotor hub) [in]
	heli.f.wl = 12.0;						// fin vertical watering station point (from main rotor hub) [in]
	heli.f.xuu = -0.4240;					// horizontal flat plate drag area [ft**2]
	heli.f.yvv = -1.2518;					// side flat plate drag area [ft**2]
	heli.f.zww = -0.8861;					// vertical flat plate drag area [ft**2]

	// Horizontal Fin Information
	heli.h.fs = 0;							// fin horizontal fuselage station point (from main rotor hub) [in]
	heli.h.wl = 0;							// fin vertical watering station point (from main rotor hub) [in]
	heli.h.zuu = 0;							// horizontal flat plate drag area [ft**2]
	heli.h.zuw = 0;							// side flat plate drag area [ft**2]
	heli.h.zmax = 0;						// vertical flat plate drag area [ft**2]

	// Vertical Fin Information
	heli.v.fs = -41.5;						// fin horizontal fuselage station point (from main rotor hub) [in]
	heli.v.wl = 7.25;						// fin vertical watering station point (from main rotor hub) [in]
	heli.v.yuu = 0;							// horizontal flat plate drag area [ft**2]
	heli.v.yuv = -1.4339;					// side flat plate drag area [ft**2]
	heli.v.ymax = -0.275;					// vertical flat plate drag area [ft**2]
	heli.v.a = 5.5;							// */rad

	// Tail Rotor Information
	heli.t.fs = -41.5;						// tail rotor horizontal fuselage station point (from main rotor hub) [in]
	heli.t.wl = 7.25;						// tail rotor vertical fuselage station point (from main rotor hub) [in]// in
	heli.t.r = 0.5417;						// tail rotor radius [ft]
	heli.t.r0 = 0.083;						// tail rotor root cutout [ft]
	heli.t.a = 3.0;							// blade lift curve slope (Cl_alpha * 1/rad)
	heli.t.b = 2;							// number of blades in tail rotor
	heli.t.c = 0.099;						// tail rotor blade chord [ft]
	heli.t.twst = 0.0;						// tail rotor blade twist [rad]
	heli.t.cd0 = 0.01;						// tail rotor blade profile drag coefficient
	heli.t.duct = 0.0;						// factor to correct duct augmentation (duct * thrust; power / duct)


	// Landing Gear Information
	// right-front gear
	heli.grIn[_GEAR_RF].cg2point[_X] = (8.0 - heli.cg.fs_cg)/12.0;	// |
	heli.grIn[_GEAR_RF].cg2point[_Y] = 0.65;						// |
	heli.grIn[_GEAR_RF].cg2point[_Z] = (20.0 - heli.cg.wl_cg)/12.0;	//  \ A/V CG -> contact point distance, in body coordinate axes [ft]
	heli.grIn[_GEAR_RF].k = 120.0;									// Hook's law spring constant [lbs / ft]
	heli.grIn[_GEAR_RF].b = sqrt(2.0*heli.grIn[_GEAR_RF].k);		// damping constant [lbs / ft / sec]
	heli.grIn[_GEAR_RF].mu_x = 0.8;									// friction coefficient on long axis
	heli.grIn[_GEAR_RF].mu_y = 0.8;									// friction coefficient on cross axis
	heli.grIn[_GEAR_RF].rotation = 0.0;								// relative rotation about vertical axis (for steering) [rad]

	// right-back gear
	heli.grIn[_GEAR_RB].cg2point[_X] = (-6.0 - heli.cg.fs_cg)/12.0;	// |
	heli.grIn[_GEAR_RB].cg2point[_Y] = 0.65;						// |
	heli.grIn[_GEAR_RB].cg2point[_Z] = (20.0 - heli.cg.wl_cg)/12.0; //  \ A/V CG -> contact point distance, in body coordinate axes [ft]
	heli.grIn[_GEAR_RB].k = 120.0;									// Hook's law spring constant [lbs / ft]
	heli.grIn[_GEAR_RB].b = sqrt(2.0*heli.grIn[_GEAR_RF].k);		// damping constant [lbs / ft / sec]
	heli.grIn[_GEAR_RB].mu_x = 0.8;									// friction coefficient on long axis
	heli.grIn[_GEAR_RB].mu_y = 0.8;									// friction coefficient on cross axis
	heli.grIn[_GEAR_RB].rotation = 0.0;								// relative rotation about vertical axis (for steering) [rad]

	// left-front gear
	heli.grIn[_GEAR_LF].cg2point[_X] = (8.0 - heli.cg.fs_cg)/12.0;	// |
	heli.grIn[_GEAR_LF].cg2point[_Y] = -0.65;						// |
	heli.grIn[_GEAR_LF].cg2point[_Z] = (20.0 - heli.cg.wl_cg)/12.0;	//  \ A/V CG -> contact point distance, in body coordinate axes [ft]
	heli.grIn[_GEAR_LF].k = 120.0;									// Hook's law spring constant [lbs / ft]
	heli.grIn[_GEAR_LF].b = sqrt(2.0*heli.grIn[_GEAR_RF].k);		// damping constant [lbs / ft / sec]
	heli.grIn[_GEAR_LF].mu_x = 0.8;									// friction coefficient on long axis
	heli.grIn[_GEAR_LF].mu_y = 0.8;									// friction coefficient on cross axis
	heli.grIn[_GEAR_LF].rotation = 0.0;								// relative rotation about vertical axis (for steering) [rad]

	// left-back gear
	heli.grIn[_GEAR_LB].cg2point[_X] = (-6.0 - heli.cg.fs_cg)/12.0;	// |
	heli.grIn[_GEAR_LB].cg2point[_Y] = -0.65;						// |
	heli.grIn[_GEAR_LB].cg2point[_Z] = (20.0 - heli.cg.wl_cg)/12.0;	//  \ A/V CG -> contact point distance, in body coordinate axes [ft]
	heli.grIn[_GEAR_LB].k = 120.0;									// Hook's law spring constant [lbs / ft]
	heli.grIn[_GEAR_LB].b = sqrt(2.0*heli.grIn[_GEAR_RF].k);		// damping constant [lbs / ft / sec]
	heli.grIn[_GEAR_LB].mu_x = 0.8;									// friction coefficient on long axis
	heli.grIn[_GEAR_LB].mu_y = 0.8;									// friction coefficient on cross axis
	heli.grIn[_GEAR_LB].rotation = 0.0;								// relative rotation about vertical axis (for steering) [rad]

	// tail
	heli.grIn[_GEAR_T].cg2point[_X] = (-41.5 - heli.cg.fs_cg)/12.0;	// |
	heli.grIn[_GEAR_T].cg2point[_Y] = 0.0;							// |
	heli.grIn[_GEAR_T].cg2point[_Z] = (15.0 - heli.cg.wl_cg)/12.0;	//  \ A/V CG -> contact point distance, in body coordinate axes [ft]
	heli.grIn[_GEAR_T].k = 140.0;									// Hook's law spring constant [lbs / ft]
	heli.grIn[_GEAR_T].b = sqrt(2.0*heli.grIn[_GEAR_RF].k);			// damping constant [lbs / ft / sec]
	heli.grIn[_GEAR_T].mu_x = 0.8;									// friction coefficient on long axis
	heli.grIn[_GEAR_T].mu_y = 0.8;									// friction coefficient on cross axis
	heli.grIn[_GEAR_T].rotation = 0.0;								// relative rotation about vertical axis (for steering) [rad]

	heli.num_gear = 5;

	// Servo Information (all servos are generic)
	// B1 (pitch) servo
	heli.svIn[0].wn = 38.2261;				// natural frequency [rad / sec]
	heli.svIn[0].zeta = 0.5118;				// damping ratio
	heli.svIn[0].slop = 0.0;				// half of system total slope (for hysteresis purposed)

	// A1 (roll) servo	
	heli.svIn[1].wn = 38.2261;				// natural frequency [rad / sec]
	heli.svIn[1].zeta = 0.5118;				// damping ratio
	heli.svIn[1].slop = 0.0;				// half of system total slope (for hysteresis purposed)

	// MR Coll servo
	heli.svIn[2].wn = 38.2261;				// natural frequency [rad / sec]
	heli.svIn[2].zeta = 0.5118;				// damping ratio
	heli.svIn[2].slop = 0.0;				// half of system total slope (for hysteresis purposed)

	// TR Coll servo
	heli.svIn[3].wn = 38.2261;				// natural frequency [rad / sec]
	heli.svIn[3].zeta = 0.5118;				// damping ratio
	heli.svIn[3].slop = 0.0;				// half of system total slope (for hysteresis purposed)


	// Tail Gyro Gain
	heli.c.gyro_gain = 0.0;

	// calculate helicopter components distance from CG
	// Main Rotor
	heli.m.h = (heli.cg.wl_cg - heli.m.wl)/12.0;	// HUB vertical distance to CG (ft)
	heli.m.d = (heli.cg.fs_cg - heli.m.fs)/12.0;	// HUB horizontal distance to CG (ft)

	// Tail Rotor
	heli.t.h = (heli.cg.wl_cg - heli.t.wl)/12.0;	// vertical distance to CG (ft)
	heli.t.d = (heli.cg.fs_cg - heli.t.fs)/12.0;	// horizontal distance to CG (ft)

	// Fuselage
	heli.f.h = (heli.cg.wl_cg - heli.f.wl)/12.0;	// vertical distance to CG (ft)
	heli.f.d = (heli.cg.fs_cg - heli.f.fs)/12.0;	// horizontal distance to CG (ft)

	// Horizontal Fin
	heli.h.h = (heli.cg.wl_cg - heli.h.wl)/12.0;	// vertical distance to CG (ft)
	heli.h.d = (heli.cg.fs_cg - heli.h.fs)/12.0;	// horizontal distance to CG (ft)

	// Vertical Fin
	heli.v.h = (heli.cg.wl_cg - heli.v.wl)/12.0;	// vertical distance to CG (ft)
	heli.v.d = (heli.cg.fs_cg - heli.v.fs)/12.0;	// horizontal distance to CG (ft)

	// 6-DOF Initializations
	sixinit.Ixx = heli.cg.ix;
	sixinit.Iyy = heli.cg.iy;
	sixinit.Izz = heli.cg.iz;
	sixinit.Ixz = heli.cg.ixz;
	sixinit.m   = heli.cg.m;
	sixinit.NED[_NORTH] = 0.0;
	sixinit.NED[_EAST]  = 0.0;
	sixinit.NED[_DOWN]  = -2.0;
	memset(sixinit.THETA, 0, _sizeOfDouble3);
	memset(sixinit.pqr,   0, _sizeOfDouble3);
	memset(sixinit.uvw,   0, _sizeOfDouble3);
	F_SIX_DOF_INIT(&sixinit, &heli.sixdofIn, &heli.sixdofX);

	// CG Initialization
	heli.cg.altitude = 0.0;
	memcpy(heli.cg.NED,   heli.sixdofX.NED,   _sizeOfDouble3);
	memcpy(heli.cg.uvw,   heli.sixdofX.Vb,    _sizeOfDouble3);
	memcpy(heli.cg.V,     heli.sixdofX.Ve,    _sizeOfDouble3);
	memcpy(heli.cg.THETA, heli.sixdofX.THETA, _sizeOfDouble3);
	memcpy(heli.cg.pqr,   heli.sixdofX.rate,  _sizeOfDouble3);
	heli.cg.time = 0.0;
	memset(heli.cg.F, 0, _sizeOfDouble3);
	memset(heli.cg.M, 0, _sizeOfDouble3);

	// Initial Control Inputs
	heli.c.A1 = 0.0;						// swash plate angle = roll cyclic pitch (+ = right roll) [rad]
	heli.c.B1 = 0.0;						// swash plate angle = pitch cyclic pitch (+ = nose down) [rad]
	heli.c.mr_col = 2.5*C_DEG2RAD;			// main rotor collective (+ = right yaw rad)
	heli.c.tr_col = 4.5*C_DEG2RAD;			// tail rotor collective (+ = climb rad)
	heli.c.mr_rev = 1500.0;					// main rotor RPM [rpm]
	heli.c.tr_rev = 4.6*heli.c.mr_rev;		// tail rotor RPM [rpm]
	heli.c.gyro_gain = 0.08;

	// set/clear the holds
	heli.sixdofIn.hold_p = 0;
	heli.sixdofIn.hold_q = 0;
	heli.sixdofIn.hold_r = 0;
	heli.sixdofIn.hold_u = 0;
	heli.sixdofIn.hold_v = 0;
	heli.sixdofIn.hold_w = 0;

	// Servo Initialization
	// B1 (pitch) servo
	heli.svX[_B1].X[0]     = 0.0;
	heli.svX[_B1].X[1]     = 0.0;
	heli.svX[_B1].output   = 0.0;
	heli.svIn[_B1].command = heli.c.B1;

	// A1 (roll) servo
	heli.svX[_A1].X[0]	    = 0.0;
	heli.svX[_A1].X[1]	    = 0.0;
	heli.svX[_A1].output	= 0.0;
	heli.svIn[_A1].command = heli.c.A1;

	// MR collective servo
	heli.svX[_MR].X[0]	    = 0.0;
	heli.svX[_MR].X[1]	    = 0.0;
	heli.svX[_MR].output   = 0.0;
	heli.svIn[_MR].command = heli.c.mr_col;

	// TR collective servo
	heli.svX[_TR].X[0]     = 0.0;
	heli.svX[_TR].X[1]     = 0.0;
	heli.svX[_TR].output   = 0.0;
	heli.svIn[_TR].command = heli.c.tr_col;

	// atmospheric model	
	F_STANDARD_ATMOSPHERE(heli.cg.altitude, &rho, &pres, &temp, &sp_sound);
	
	// main rotor angular velocity [rad / sec]
	heli.m.omega = heli.c.mr_rev * C_RPM2RADSEC;

	// main rotor tip velocity [ft / sec]
	heli.m.v_tip = heli.m.r * heli.m.omega;

	// main rotor lock number
	heli.m.lock = (rho * heli.m.a * heli.m.c * M_SQR(heli.m.r) * M_SQR(heli.m.r)) / heli.m.i_b;

	// main rotor natural frequency shift
	heli.m.omega_f = (heli.m.lock * heli.m.omega / 16.0) * (1.0 + (8.0 / 3.0) * (heli.m.e / heli.m.r));

	// main rotor cross couple coefficient
	heli.m.k2 = 0.75*(heli.m.e/heli.m.r)*(heli.m.omega/heli.m.omega_f);

	// main rotor flapping stiffness (total cross couple coefficient)
	heli.m.kc = heli.m.k1 + heli.m.k2;

	// main rotor time constant
	heli.m.tau = 16.0 / (heli.m.omega * heli.m.lock);

	// main rotor off plane change in flap
	heli.m.w_off = heli.m.omega / (1.0 + M_SQR(heli.m.omega / heli.m.omega_f));

	// main rotor in plane change in flap
	heli.m.w_in = heli.m.omega / heli.m.omega_f * heli.m.w_off;

	// rolling moment due to b1 TPP tilt derivative [lb-ft / rad]
	heli.m.dl_db1 = 0.75 * (heli.m.b * heli.m.c * M_SQR(heli.m.r) * rho * M_SQR(heli.m.v_tip) * heli.m.a * heli.m.e / (heli.m.lock * heli.m.r));

	// pitching moment due to a1 TPP tilt derivative [lb-ft / rad]
	heli.m.dm_da1 = heli.m.dl_db1;

	// main rotor thrust coefficient
	heli.m.ct = heli.cg.wt / (rho * C_PI * M_SQR(heli.m.r) * M_SQR(heli.m.v_tip));

	// main rotor solidity
	heli.m.sigma = heli.m.b * heli.m.c / (C_PI * heli.m.r);

	// TPP lateral dihedral effect derivative
	heli.m.db1dv = -(2.0 / heli.m.v_tip) * (8.0 * heli.m.ct / (heli.m.a * heli.m.sigma) + sqrt(heli.m.ct / 2.0));

	// TPP longitudinal dihedral effect derivative
	heli.m.da1du = -heli.m.db1dv;

	// main rotor initialization (forces, moments, actuators)
	heli.m.vi     = 0.0;	// initial induced velocity
	heli.m.a1     = 0.0;
	heli.m.b1     = 0.0;
	heli.m.a1dot  = 0.0;
	heli.m.b1dot  = 0.0;
	heli.m.thrust = 0.0;
	heli.m.x      = 0.0;
	heli.m.y      = 0.0;
	heli.m.z      = 0.0;
	heli.m.l      = 0.0;
	heli.m.m      = 0.0;
	heli.m.n      = 0.0;

	//  fly bar initialization
	heli.fb.c     = 0.0;	// longitudinal fly bar TPP tilt [rad]
	heli.fb.c_dot = 0.0;	// longitudinal fly bar TPP tilt rate [rad / sec]
	heli.fb.d     = 0.0;	// lateral fly bar TPP tilt [rad]
	heli.fb.d_dot = 0.0;	// lateral fly bar TPP tilt rate [rad / sec]

	// tail rotor initialization (forces, moments)
	heli.t.omega  = heli.c.tr_rev * C_RPM2RADSEC;						// angular velocity [rad / sec]
	heli.t.fr     = heli.t.cd0 * heli.t.r * heli.t.b * heli.t.c;		// normalized drag
	heli.t.vi     = 10.0;												// initial induced velocity
	heli.t.thrust = 2.0;
	heli.t.x      = 0.0;
	heli.t.y      = 0.0;
	heli.t.z      = 0.0;
	heli.t.l      = 0.0;
	heli.t.m      = 0.0;
	heli.t.n      = 0.0;
}

/*	This will perform the time stepping of the servo model.
* The values of U[7] are as follows...
*	U[0] = main rotor collective (rad)
*	U[1] = A1 swash plate tilt (+ right roll) (rad)
*	U[2] = B1 swash plate tilt (+ nose down) (rad)
*	U[3] = tail rotor collective (rad)
*
*  The svIn[3] and svX[3] are as follows...
*	sv[0] = B1 (pitch servo)
*	sv[1] = A1 (roll servo)
*	sv[2] = main rotor collective
*	sv[3] = tail rotor collective
*/

/**
* servo model prorogation in time
*
* @param  {struct} helicopter definition (holding actuator inputs)
* @return {double} 1x4 array holding actuators output in the following manner:
*					U[0] = main rotor collective                [rad]
*					U[1] = A1 swash plate tilt (+ = right roll) [rad]
*					U[2] = B1 swash plate tilt (+ = nose down)  [rad]
*					U[3] = tail rotor collective                [rad]
**/
void F_ACTUATOR(struct heli_def *heli, double U[4]) {
	// statics
	static const double C_MIN_MAIN_COLLECTIVE =  2.5   * C_DEG2RAD,
						C_MAX_MAIN_COLLECTIVE =  18.0  * C_DEG2RAD,
						C_MIN_TAIL_COLLECTIVE = -20.0  * C_DEG2RAD,
						C_MAX_TAIL_COLLECTIVE =  20.0  * C_DEG2RAD,
						C_MAX_A1              =  8.0   * C_DEG2RAD,
						C_MAX_B1              =  8.0   * C_DEG2RAD;

	// locals
	struct control_def      *c         = &heli->c;
	struct cg_def           *cg        = &heli->cg;
	struct servo_inputs_def *A1svIn    = &heli->svIn[1];
	struct servo_state_def  *A1svX     = &heli->svX[1];
	struct servo_inputs_def *B1svIn    = &heli->svIn[0];
	struct servo_state_def  *B1svX     = &heli->svX[0];
	struct servo_inputs_def *mrColsvIn = &heli->svIn[2];
	struct servo_state_def  *mrColsvX  = &heli->svX[2];
	struct servo_inputs_def *trColsvIn = &heli->svIn[3];
	struct servo_state_def  *trColsvX  = &heli->svX[3];

	// housekeeping
	mrColsvIn->command  = M_LIMIT(U[0],     C_MIN_MAIN_COLLECTIVE, C_MAX_MAIN_COLLECTIVE);
	trColsvIn->command  = M_LIMIT(U[3],     C_MIN_TAIL_COLLECTIVE, C_MAX_TAIL_COLLECTIVE);
	A1svIn->command     = M_LIMIT_SYM(U[1], C_MAX_A1);
	B1svIn->command     = M_LIMIT_SYM(U[2], C_MAX_B1);

	// servo model time propagation
	F_SERVO_MODEL(A1svX,    A1svIn,    model_dt);
	F_SERVO_MODEL(B1svX,    B1svIn,    model_dt);
	F_SERVO_MODEL(mrColsvX, mrColsvIn, model_dt);
	F_SERVO_MODEL(trColsvX, trColsvIn, model_dt);

	// actuator assignment
	c->B1     = B1svX->output;
	c->A1     = A1svX->output;
	c->mr_col = mrColsvX->output;
	c->tr_col = trColsvX->output;
}

/**
* landing gear kernel
*
* @param/return {struct} helicopter definition
**/
void F_LANDING_GEAR(struct heli_def *heli) {
	// locals
	struct cg_def      *cg = &heli->cg;
	struct control_def *c  = &heli->c;
	int i;

	// calculate collision for every landing gear component
	for(i = 0; i < heli->num_gear; ++i) {

		// extract gear component altitude / orientation / linear & angular velocity
		heli->grIn[i].altitude = cg->NED[2];
		memcpy(heli->grIn[i].THETA, cg->THETA, 3 * sizeof(double));
		memcpy(heli->grIn[i].uvw,   cg->uvw,   3 * sizeof(double));
		memcpy(heli->grIn[i].pqr,   cg->pqr,   3 * sizeof(double));

		// collision
		F_GEAR_COLLISION(&heli->grIn[i], &heli->grOut[i]);

		// forces & moments assignments
		cg->F[_X] += heli->grOut[i].F[_X];
		cg->F[_Y] += heli->grOut[i].F[_Y];
		cg->F[_Z] += heli->grOut[i].F[_Z];
		cg->M[_X] += heli->grOut[i].M[_X];
		cg->M[_Y] += heli->grOut[i].M[_Y];
		cg->M[_Z] += heli->grOut[i].M[_Z];
	}	
}

/**
* module to calculate A/V forces and moments prior to 6DOF calculation.
*
* @param/return {struct} helicopter definition
**/
void F_FORCES_MOMENTS(struct heli_def *heli) {
	// locals
	struct mainrotor_def     *m  = &heli->m;
	struct flybar_def        *fb = &heli->fb;
	struct tailrotor_def     *t  = &heli->t;
	struct verticalfin_def   *v  = &heli->v;
	struct horizontalfin_def *h  = &heli->h;
	struct fuse_def          *f  = &heli->f;
	struct cg_def            *cg = &heli->cg;
	struct blade_element_def *mb = &heli->MRBE;
	struct blade_element_def *tb = &heli->TRBE;
	struct control_def       *c  = &heli->c;
	double _density,							// density of air [slug / ft**3]
		   _densityHalf,						// density divided by two
	       _pressure,							// air _pressure [lb / ft**2]
	       _temperature,						// air temperature [R]
	       _soundSpeed,							// local speed of sound [ft / sec]
	       _densityAltitude,					// current density altitude [ft]
	       _earth2body[3][3],					// earth to body transformation matrix
	       _vectorTemp[3],						// temporary array
	       Xdot[4],								// |
		   X[4],								// |
		   U[2],								// |
		   args[13];							//  \ rotor dynamical system (states, inputs) for RK4

	// calculate current atmospheric conditions
	_densityAltitude  = cg->altitude;
	F_STANDARD_ATMOSPHERE(_densityAltitude, &_density, &_pressure, &_temperature, &_soundSpeed);
	_densityHalf = 0.5 * _density;

	// calculate local gravitational vector in body coordinates
	_vectorTemp[_X] = 0.0;
	_vectorTemp[_Y] = 0.0;
	_vectorTemp[_Z] = 32.17417 * cg->m;
	F_EULER_TO_DCM(_earth2body, cg->THETA[_PHI], cg->THETA[_THETA], cg->THETA[_PSI]);
	cg->F[_X] = _earth2body[_X][_X] * _vectorTemp[_X] + _earth2body[_X][_Y] * _vectorTemp[_Y] + _earth2body[_X][_Z] * _vectorTemp[_Z];
	cg->F[_Y] = _earth2body[_Y][_X] * _vectorTemp[_X] + _earth2body[_Y][_Y] * _vectorTemp[_Y] + _earth2body[_Y][_Z] * _vectorTemp[_Z];
	cg->F[_Z] = _earth2body[_Z][_X] * _vectorTemp[_X] + _earth2body[_Z][_Y] * _vectorTemp[_Y] + _earth2body[_Z][_Z] * _vectorTemp[_Z];

	// main rotor blade element parameters needed for forces and moments calculation
	mb->a          = m->a;
	mb->b          = m->b;
	mb->c          = m->c;
	mb->Cd0        = m->cd0;
	mb->collective = c->mr_col;
	mb->e          = 0.7;
	mb->omega      = c->mr_rev * C_RPM2RADSEC;
	mb->R          = m->r;
	mb->R0         = m->ro;
	mb->rho        = _density;
	mb->twst       = m->twst;
	mb->Vperp      = -(cg->uvw[_Z] + cg->uvw[_X] * (m->is + m->a1) - cg->uvw[_Y] * (m->ib + m->b1));

	// main rotor forces calculation and data assigning
	F_ROTOR_CALCULATION(mb);
	m->thrust = mb->T;
	m->power  = mb->P;
	m->torque = mb->Q;
	m->vi     = mb->avg_v1;

	// main rotor forces
	m->x = -m->thrust * (m->is + m->a1);
	m->y =  m->thrust * (m->ib + m->b1);
	m->z = -m->thrust;

	// main rotor moments
	m->l = m->y*m->h + m->dl_db1 * m->b1;
	m->m = m->z*m->d - m->x * m->h + m->dm_da1 * m->a1;
	m->n = m->torque * m->dir;

	// tail rotor blade element parameters needed for forces and moments calculation
	tb->a          = t->a;
	tb->b          = t->b;
	tb->c          = t->c;
	tb->Cd0        = t->cd0;
	tb->collective = c->tr_col;
	tb->e          = 0.7;
	tb->omega      = c->tr_rev * C_RPM2RADSEC;
	tb->R          = t->r;
	tb->R0         = t->r0;
	tb->rho        = _density;
	tb->twst       = t->twst;
	tb->Vperp      = (cg->uvw[_Y] - t->d * cg->pqr[_Z]) * m->dir;

	// tail rotor forces calculation and data assigning
	F_ROTOR_CALCULATION(tb);
	t->thrust = tb->T + tb->T * t->duct;
	t->power  = tb->P - tb->P * t->duct;

	// tail rotor forces
	t->x = 0.0;
	t->y = t->thrust * m->dir;
	t->z = 0.0;

	// tail rotor moments
	t->l = t->y * t->h;
	t->m = 0.0;
	t->n = -t->y * t->d;

	// fuselage forces and moment calculations
	f->x = _densityHalf * cg->uvw[_X] * M_ABS(cg->uvw[_X]) * f->xuu;
	f->y = _densityHalf * cg->uvw[_Y] * M_ABS(cg->uvw[_Y]) * f->yvv;
	f->z = _densityHalf * f->zww * (cg->uvw[_Z] * M_ABS(cg->uvw[_Z]) - m->vi);
	f->l = f->y  * f->h;
	f->m = f->z  * f->d - f->x * f->h;
	f->n = -f->y * f->d;

	// vertical fin forces and moments
	v->x = 0.0;
	v->y = _densityHalf * M_ABS(cg->uvw[_Y] * cg->uvw[_Y]) * v->yuv;
	v->z = 0.0;
	v->l = v->y * v->h;
	v->m = 0.0;
	v->n = -v->y * v->d;

	// horizontal fin forces and moments
	// h->x = 0.0;
	// h->y = 0.0;
	// h->z = _densityHalf * M_ABS(cg->uvw[_X] * cg->uvw[_X]) * h->zuw;
	// h->l = 0.0;
	// h->m = v->z * v->;
	// h->n = 0.0;

	// main rotor tail tip dynamics system definition
	X[0]     = m->a1;
	X[1]     = m->b1;
	X[2]     = fb->d;
	X[3]     = fb->c;
	U[0]     = c->A1;
	U[1]     = c->B1;
	args[0]  = cg->uvw[_X];
	args[1]  = cg->uvw[_Y];
	args[2]  = cg->pqr[_X];
	args[3]  = cg->pqr[_Y];
	args[4]  = m->db1dv;
	args[5]  = m->da1du;
	args[6]  = m->w_in;
	args[7]  = m->w_off;
	args[8]  = m->kc;
	args[9]  = m->dir;
	args[10] = fb->tau;
	args[11] = fb->Kc;
	args[12] = fb->Kd;
	
	// main rotor tail tip dynamics system propagation
	F_RK4(X, Xdot, cg->time, U, args, 4, model_dt, &F_ROTOR_FLAP_MODEL);

	// main rotor tail tip dynamics system output
	m->a1     = X[0];
	m->b1     = X[1];
	fb->d     = X[2];
	fb->c     = X[3];
	m->a1dot  = Xdot[0];
	m->b1dot  = Xdot[1];
	fb->d_dot = Xdot[2];
	fb->c_dot = Xdot[3];

	// total forces and moments around C,G
	cg->F[_X] += m->x + t->x + f->x + v->x;
	cg->F[_Y] += m->y + t->y + f->y + v->y;
	cg->F[_Z] += m->z + t->z + f->z + v->z;
	cg->M[_X]  = m->l + t->l + f->l + v->l;
	cg->M[_Y]  = m->m + t->m + f->m + v->m;
	cg->M[_Z]  = m->n + t->n + f->n + v->n;
}

/**
* function to propagate helicopter mode, 6DOF, landing gear, servos and wind.
*
* @param {double} 1x4 array holding model inputs in the following manner:
*					U[4] = [main rotor collective	(+ = goes up) [rad]
*							A1					    (+ = right wind down) [rad]
*							B1					    (+ = nose down) [rad]
*							tr_coll]			    (+ = right turn) [rad]
**/
void ModelGO(double U[4]) {
	// locals
	int _sizeOfDouble3 = 3 * sizeof(double);

	// calculate forces and moments
	F_FORCES_MOMENTS(&heli);

	// calculate landing gear collision
	F_LANDING_GEAR(&heli);

	// calculate actuator
	F_ACTUATOR(&heli, U);

	// assign data
	memcpy(heli.sixdofIn.F, heli.cg.F, _sizeOfDouble3);
	memcpy(heli.sixdofIn.M, heli.cg.M, _sizeOfDouble3);

	// flat earth 6DOF
	F_SIX_DOF(&heli.sixdofX, &heli.sixdofIn, model_dt);

	// assign data
	memcpy(heli.cg.NED,   heli.sixdofX.NED,   _sizeOfDouble3);
	memcpy(heli.cg.uvw,   heli.sixdofX.Vb,    _sizeOfDouble3);
	memcpy(heli.cg.V,     heli.sixdofX.Ve,    _sizeOfDouble3);
	memcpy(heli.cg.THETA, heli.sixdofX.THETA, _sizeOfDouble3);
	memcpy(heli.cg.pqr,   heli.sixdofX.rate,  _sizeOfDouble3);

	// increase time counter
	heli.cg.time += model_dt;
}
