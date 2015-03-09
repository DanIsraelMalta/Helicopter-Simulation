/**
* rigid body 6-DOF simulation components, including:
* - contact/collision simulation.
* - IMU emulator
* - actuator model
* - WGS-84 gravity model
* - Dryden wind model
* - 4th order RK integration
**/

#include "simlib.h"

/**
* globals
**/
struct sixdofX_fe_def {
	double Vned[3];				// [Vn Ve Vd] (ft/s)
	double NED[3];				// [N E D] (ft)
	double pqr[3];				// [P Q R] (rad/s)
	double Q[4];				// [q0 q1 q2 q3]
};

struct sixdofXdot_fe_def {
	double Vned_dot[3];			// [Vndot Vedot Vddot] (ft/s/s)
	double NED_dot[3];			// [Ndot Edot Ddot] (ft/s)
	double pqr_dot[3];			// [Pdot Qdot Rdot] (rad/s/s)
	double Q_dot[4];			// [q0dot q1dot q2dot q3dot]
};

/**
* prototypes
**/
void F_SIX_DOF_DERIVATIVE(struct sixdofXdot_fe_def *pXdot, 
					  struct sixdofX_fe_def *pX, 
					  struct sixdof_fe_inputs_def *pU);
void F_SIX_DOF_INTEGRATE(struct sixdofXdot_fe_def *pXdot, 
						 struct sixdofX_fe_def *pX, 
						 struct sixdof_fe_inputs_def *pU, 
						 double dt);

/**
* full state strap down IMU emulation function.
* output IS A/V state as sensed at IMU given location.
*
* @param  {struct} IMU inputs
* @return {struct} IMU outputs
**/
void F_IMU(struct imu_inputs_def *pIn, struct imu_outputs_def *pOut) {
	// locals
	double _omega[3][3],			// rotational velocity matrix
	       _earth2body[3][3],		// earth -> body transformation matrix
	       _body2earth[3][3],		// body -> earth transformation matrix
	       _imu2body[3][3],			// IMU -> body transformation matrix
	       _body2imu[3][3],			// body -> IMU transformation matrix
	       _geodeticCG[3],			// {lat, long, alt} of the cg [rad, rad, ft]
	       _matTemp1[3][3],			// temporary matrix
	       _vectorTemp1[3],			// temporary vector
	       _VectorTemp2[3],			// temporary vector
	       _vectorTemp3[3],			// temporary vector
	       _lat, _lon;				// latitude and longitude [rad]

	// construct Euler "omega cross" matrix
	_omega[_X][_X] =  0.0;
	_omega[_X][_Y] = -pIn->cg_pqr[_Z];
	_omega[_X][_Z] =  pIn->cg_pqr[_Y];
	_omega[_Y][_X] =  pIn->cg_pqr[_Z];
	_omega[_Y][_Y] =  0.0;
	_omega[_Y][_Z] = -pIn->cg_pqr[_X];
	_omega[_Z][_X] = -pIn->cg_pqr[_Y];
	_omega[_Z][_Y] =  pIn->cg_pqr[_X];
	_omega[_Z][_Z] =  0.0;

	// create earth to body transformation matrix
	F_EULER_TO_DCM(_earth2body, pIn->cg_THETA[_PHI], pIn->cg_THETA[_THETA], pIn->cg_THETA[_PSI]);

	// create body to earth transformation matrix
	M_MAT_TRANSPOSE(_body2earth, _earth2body);

	// create body to IMU transformation matrix
	memcpy(_body2imu, pIn->body2imu, 9 * sizeof(double));

	// create IMU to body transformation matrix
	M_MAT_TRANSPOSE(_imu2body, _body2imu);

	// calculate C.G location in geodetic coordinate system [ft]
	_vectorTemp1[_X] = pIn->cg_pos[_X] * C_FOOT_TO_METER;
	_vectorTemp1[_Y] = pIn->cg_pos[_Y] * C_FOOT_TO_METER;
	_vectorTemp1[_Z] = pIn->cg_pos[_Z] * C_FOOT_TO_METER;
	F_ECEF_TO_GEODETIC(_vectorTemp1, _geodeticCG);
	_geodeticCG[_Z] = _geodeticCG[_Z] * C_METER_TO_FOOT;
	_lat = _geodeticCG[_X];
	_lon = _geodeticCG[_Y];

	// calculate acceleration vector at IMU location ( a_imu = _body2imu * (a_cg + (alpha X position) + (w X (w X position)) )
	// alpha X position
	_vectorTemp1[_X] = pIn->cg_alpha[_Y] * pIn->cg2imu[_Z] - pIn->cg_alpha[_Z] * pIn->cg2imu[_Y];
	_vectorTemp1[_Y] = pIn->cg_alpha[_Z] * pIn->cg2imu[_X] - pIn->cg_alpha[_X] * pIn->cg2imu[_Z];
	_vectorTemp1[_Z] = pIn->cg_alpha[_X] * pIn->cg2imu[_Y] - pIn->cg_alpha[_Y] * pIn->cg2imu[_X];

	// _omega X _omega
	_matTemp1[_X][_X] = _omega[_X][_X] * _omega[_X][_X] + _omega[_X][_Y] * _omega[_Y][_X] + _omega[_X][_Z] * _omega[_Z][_X];
	_matTemp1[_X][_Y] = _omega[_X][_X] * _omega[_X][_Y] + _omega[_X][_Y] * _omega[_Y][_Y] + _omega[_X][_Z] * _omega[_Z][_Y];
	_matTemp1[_X][_Z] = _omega[_X][_X] * _omega[_X][_Z] + _omega[_X][_Y] * _omega[_Y][_Z] + _omega[_X][_Z] * _omega[_Z][_Z];
	_matTemp1[_Y][_X] = _omega[_Y][_X] * _omega[_X][_X] + _omega[_Y][_Y] * _omega[_Y][_X] + _omega[_Y][_Z] * _omega[_Z][_X];
	_matTemp1[_Y][_Y] = _omega[_Y][_X] * _omega[_X][_Y] + _omega[_Y][_Y] * _omega[_Y][_Y] + _omega[_Y][_Z] * _omega[_Z][_Y];
	_matTemp1[_Y][_Z] = _omega[_Y][_X] * _omega[_X][_Z] + _omega[_Y][_Y] * _omega[_Y][_Z] + _omega[_Y][_Z] * _omega[_Z][_Z];
	_matTemp1[_Z][_X] = _omega[_Z][_X] * _omega[_X][_X] + _omega[_Z][_Y] * _omega[_Y][_X] + _omega[_Z][_Z] * _omega[_Z][_X];
	_matTemp1[_Z][_Y] = _omega[_Z][_X] * _omega[_X][_Y] + _omega[_Z][_Y] * _omega[_Y][_Y] + _omega[_Z][_Z] * _omega[_Z][_Y];
	_matTemp1[_Z][_Z] = _omega[_Z][_X] * _omega[_X][_Z] + _omega[_Z][_Y] * _omega[_Y][_Z] + _omega[_Z][_Z] * _omega[_Z][_Z];

	// (_omega X _omega) X position
	_VectorTemp2[_X] = _matTemp1[_X][_X] * pIn->cg2imu[_X] + _matTemp1[_X][_Y] * pIn->cg2imu[_Y] + _matTemp1[_X][_Z] * pIn->cg2imu[_Z];
	_VectorTemp2[_Y] = _matTemp1[_Y][_X] * pIn->cg2imu[_X] + _matTemp1[_Y][_Y] * pIn->cg2imu[_Y] + _matTemp1[_Y][_Z] * pIn->cg2imu[_Z];
	_VectorTemp2[_Z] = _matTemp1[_Z][_X] * pIn->cg2imu[_X] + _matTemp1[_Z][_Y] * pIn->cg2imu[_Y] + _matTemp1[_Z][_Z] * pIn->cg2imu[_Z];


	// (_omega X _omega) X position + (alpha X position)
	_vectorTemp3[_X] = _vectorTemp1[_X] + _VectorTemp2[_X];
	_vectorTemp3[_Y] = _vectorTemp1[_Y] + _VectorTemp2[_Y];
	_vectorTemp3[_Z] = _vectorTemp1[_Z] + _VectorTemp2[_Z];

	// a_cg + (_omega X _omega) X position + (alpha X position)
	_vectorTemp1[_X] = _vectorTemp3[_X] + pIn->cg_accel[_X];
	_vectorTemp1[_Y] = _vectorTemp3[_Y] + pIn->cg_accel[_Y];
	_vectorTemp1[_Z] = _vectorTemp3[_Z] + pIn->cg_accel[_Z];

	// a_imu = _body2imu * a_cg
	pOut->accel[_X] = _body2imu[_X][_X] * _vectorTemp1[_X] + _body2imu[_X][_Y] * _vectorTemp1[_Y] + _body2imu[_X][_Z] * _vectorTemp1[_Z];
	pOut->accel[_Y] = _body2imu[_Y][_X] * _vectorTemp1[_X] + _body2imu[_Y][_Y] * _vectorTemp1[_Y] + _body2imu[_Y][_Z] * _vectorTemp1[_Z];
	pOut->accel[_Z] = _body2imu[_Z][_X] * _vectorTemp1[_X] + _body2imu[_Z][_Y] * _vectorTemp1[_Y] + _body2imu[_Z][_Z] * _vectorTemp1[_Z];

	// calculate velocity vector at IMU location (v_imu = _body2imu * (v_cg + (w X pos)) )
	// w X position
	_vectorTemp1[_X] = _omega[_X][_X] * pIn->cg2imu[_X] + _omega[_X][_Y] * pIn->cg2imu[_Y] + _omega[_X][_Z] * pIn->cg2imu[_Z];
	_vectorTemp1[_Y] = _omega[_Y][_X] * pIn->cg2imu[_X] + _omega[_Y][_Y] * pIn->cg2imu[_Y] + _omega[_Y][_Z] * pIn->cg2imu[_Z];
	_vectorTemp1[_Z] = _omega[_Z][_X] * pIn->cg2imu[_X] + _omega[_Z][_Y] * pIn->cg2imu[_Y] + _omega[_Z][_Z] * pIn->cg2imu[_Z];

	// v_cg + (w X position)
	_VectorTemp2[_X] = _vectorTemp1[_X] + pIn->cg_uvw[_X];
	_VectorTemp2[_Y] = _vectorTemp1[_Y] + pIn->cg_uvw[_Y];
	_VectorTemp2[_Z] = _vectorTemp1[_Z] + pIn->cg_uvw[_Z];

	// v_imu = _body2imu * (v_cg + (w X position))
	pOut->uvw[_X] = _body2imu[_X][_X] * _VectorTemp2[_X] + _body2imu[_X][_Y] * _VectorTemp2[_Y] + _body2imu[_X][_Z] * _VectorTemp2[_Z];
	pOut->uvw[_Y] = _body2imu[_Y][_X] * _VectorTemp2[_X] + _body2imu[_Y][_Y] * _VectorTemp2[_Y] + _body2imu[_Y][_Z] * _VectorTemp2[_Z];
	pOut->uvw[_Z] = _body2imu[_Z][_X] * _VectorTemp2[_X] + _body2imu[_Z][_Y] * _VectorTemp2[_Y] + _body2imu[_Z][_Z] * _VectorTemp2[_Z];

	// rotate IMU position in body coordinate system to TP coordinates
	_vectorTemp1[_X] = _body2earth[_X][_X] * pIn->cg2imu[_X] + _body2earth[_X][_Y] * pIn->cg2imu[_Y] + _body2earth[_X][_Z] * pIn->cg2imu[_Z];
	_vectorTemp1[_Y] = _body2earth[_Y][_X] * pIn->cg2imu[_X] + _body2earth[_Y][_Y] * pIn->cg2imu[_Y] + _body2earth[_Y][_Z] * pIn->cg2imu[_Z];
	_vectorTemp1[_Z] = _body2earth[_Z][_X] * pIn->cg2imu[_X] + _body2earth[_Z][_Y] * pIn->cg2imu[_Y] + _body2earth[_Z][_Z] * pIn->cg2imu[_Z];

	// transform position to ECEF coordinate system
	F_GEODETIC_TO_ECEF(_VectorTemp2, _vectorTemp1, _lat, _lon);

	// calculate IMU position in ECEF coordinate system
	pOut->ECEFpos[_X] = _VectorTemp2[_X] + pIn->cg_pos[_X];
	pOut->ECEFpos[_Y] = _VectorTemp2[_Y] + pIn->cg_pos[_Y];
	pOut->ECEFpos[_Z] = _VectorTemp2[_Z] + pIn->cg_pos[_Z];

	// calculate IMU position in geodetic coordinate system [ft]
	_vectorTemp1[_X] = pOut->ECEFpos[_X] * C_FOOT_TO_METER;
	_vectorTemp1[_Y] = pOut->ECEFpos[_Y] * C_FOOT_TO_METER;
	_vectorTemp1[_Z] = pOut->ECEFpos[_Z] * C_FOOT_TO_METER;
	F_ECEF_TO_GEODETIC(_vectorTemp1, pOut->LLHpos);
	pOut->LLHpos[_ALT] = pOut->LLHpos[_ALT] * C_METER_TO_FOOT;

	// calculate IMU angular velocity vector
	pOut->pqr[_X] = _body2imu[_X][_X] * pIn->cg_pqr[_X] + _body2imu[_X][_Y] * pIn->cg_pqr[_Y] + _body2imu[_X][_Z] * pIn->cg_pqr[_Z];
	pOut->pqr[_Y] = _body2imu[_Y][_X] * pIn->cg_pqr[_X] + _body2imu[_Y][_Y] * pIn->cg_pqr[_Y] + _body2imu[_Y][_Z] * pIn->cg_pqr[_Z];
	pOut->pqr[_Z] = _body2imu[_Z][_X] * pIn->cg_pqr[_X] + _body2imu[_Z][_Y] * pIn->cg_pqr[_Y] + _body2imu[_Z][_Z] * pIn->cg_pqr[_Z];

	// calculate IMU attitude (DCM matrix)
	_matTemp1[_X][_X] = _body2imu[_X][_X] * _earth2body[_X][_X] + _body2imu[_X][_Y] * _earth2body[_Y][_X] + _body2imu[_X][_Z] * _earth2body[_Z][_X];
	_matTemp1[_X][_Y] = _body2imu[_X][_X] * _earth2body[_X][_Y] + _body2imu[_X][_Y] * _earth2body[_Y][_Y] + _body2imu[_X][_Z] * _earth2body[_Z][_Y];
	_matTemp1[_X][_Z] = _body2imu[_X][_X] * _earth2body[_X][_Z] + _body2imu[_X][_Y] * _earth2body[_Y][_Z] + _body2imu[_X][_Z] * _earth2body[_Z][_Z];
	_matTemp1[_Y][_X] = _body2imu[_Y][_X] * _earth2body[_X][_X] + _body2imu[_Y][_Y] * _earth2body[_Y][_X] + _body2imu[_Y][_Z] * _earth2body[_Z][_X];
	_matTemp1[_Y][_Y] = _body2imu[_Y][_X] * _earth2body[_X][_Y] + _body2imu[_Y][_Y] * _earth2body[_Y][_Y] + _body2imu[_Y][_Z] * _earth2body[_Z][_Y];
	_matTemp1[_Y][_Z] = _body2imu[_Y][_X] * _earth2body[_X][_Z] + _body2imu[_Y][_Y] * _earth2body[_Y][_Z] + _body2imu[_Y][_Z] * _earth2body[_Z][_Z];
	_matTemp1[_Z][_X] = _body2imu[_Z][_X] * _earth2body[_X][_X] + _body2imu[_Z][_Y] * _earth2body[_Y][_X] + _body2imu[_Z][_Z] * _earth2body[_Z][_X];
	_matTemp1[_Z][_Y] = _body2imu[_Z][_X] * _earth2body[_X][_Y] + _body2imu[_Z][_Y] * _earth2body[_Y][_Y] + _body2imu[_Z][_Z] * _earth2body[_Z][_Y];
	_matTemp1[_Z][_Z] = _body2imu[_Z][_X] * _earth2body[_X][_Z] + _body2imu[_Z][_Y] * _earth2body[_Y][_Z] + _body2imu[_Z][_Z] * _earth2body[_Z][_Z];

	pOut->THETA[_PHI]   =  M_ATAN2(_matTemp1[_Y][_Z], _matTemp1[_Z][_Z]);
	pOut->THETA[_THETA] = -asin(_matTemp1[_X][_Z]);
	pOut->THETA[_PSI]   =  M_ATAN2(_matTemp1[_X][_Y], _matTemp1[_X][_X]);
}


/**
* general 4th order Runge-Kuta integration routine
*
* @param/return {double}  state vector
* @param/return {double}  state vector derivative
* @param        {double}  current time
* @param        {double}  any type of "forcing" value
* @param        {double}  input argument for the called derivative function
* @param		{int}     number of states in state vector
* @param        {double}  time step
* @param		{pointer} pointer to called function whose arguments are:
						  (double *Xdot,	// returned state derivative (OUTPUT)
				 		   double *X,		// state vector (INPUT called by RK4)
						   double *t,		// time (INPUT called by RK4)
						   double *U,		// passed forcing values (INPUT called by RK4)
						   double *args) );	// passed arguments for derivative function (INPUT called by RK4)
**/
void F_RK4(double *state, double *state_dot, double t, double *force, double *args, int n, double dt, 
		 void (*pfunc)(double *Xdot, double *X, double *t, double *U, double *args) ) {
	// locals
	double *X0,			// for saving in inItial state vector
	       *X,			// for making the new state vector, for called function
	       *Xdot,		// state derivative vector of called function
	       *k1, *k2, *k3, *k4, time;
	int i;
	size_t numbytes = n * sizeof(double);

	// initializing the memory for the arrays
	X0   = (double *)malloc(numbytes);
	X    = (double *)malloc(numbytes);
	Xdot = (double *)malloc(numbytes);
	k1   = (double *)malloc(numbytes);
	k2   = (double *)malloc(numbytes);
	k3   = (double *)malloc(numbytes);
	k4   = (double *)malloc(numbytes);

	// backup the original state vector
	time = t;
	memcpy(X0, state, n * sizeof(double));
	/*
	for(i=0; i<n; ++i) {
		X0[i] = state[i];
	}
	*/

	// the first step
	(*pfunc)(Xdot, X0, &time, force, args);
	for(i=0; i<n; ++i) {
		// save the k value for later
		k1[i] = Xdot[i] * dt;

		// make the next state vector
		X[i] = X0[i] + 0.5 * k1[i];

		// make the time for next step
		time = t + 0.5 * dt;
	}

	// the second step
	(*pfunc)(Xdot, X, &time, force, args);
	for(i=0; i<n; ++i) {
		// save the k value for later
		k2[i] = Xdot[i] * dt;
		
		// make the next state vector
		X[i] = X0[i] + 0.5 * k2[i];
		
		// make the time for next step
		time = t + 0.5 * dt;
	}

	// the third step
	(*pfunc)(Xdot, X, &time, force, args);
	for(i=0; i<n; ++i) {
		// save the k value for later
		k3[i] = Xdot[i] * dt;

		// make the next state vector
		X[i] = X0[i] + 0.5 * k3[i];

		// make the time for next step
		time = t + 0.5 * dt;
	}
	
	// the forth step
	(*pfunc)(Xdot, X, &time, force, args);
	for(i=0; i<n; ++i) {
		k4[i] = Xdot[i] * dt;
	}

	// make the final result
	for(i=0; i<n; ++i) {
		state[i] = X0[i] + (k1[i] + k4[i]) * C_ONE_SIXTH + (k2[i] + k3[i]) * C_ONE_THIRD;
	}

	// freeing the memory allocated for the arrays (to prevent memory leaks)
	free(X0);
	free(X);
	free(Xdot);
	free(k1);
	free(k2);
	free(k3);
	free(k4);
}

/**
* flat earth 6DOF initialization
*
* @param  {struct} A/V information and initial position/orientation/velocity
* @return {struct} A/V forces and moments
* @return {struct} A/V state
**/
void F_SIX_DOF_INIT(struct sixdof_fe_init_inputs_def *pInit, 
				    struct sixdof_fe_inputs_def      *pIn,
				    struct sixdof_fe_state_def       *pX) {
	// locals
	double _gammaInv,				// inverse of (Ixx * Izz - Ixz**2)
	       _earth2body[3][3],		// earth -> body transformation matrix
	       _body2earth[3][3];		// body  -> earth transformation matrix

	// housekeeping
	memset(pIn->J, 0, 9 * sizeof(double));
	memset(pIn->Jinv, 0, 9 * sizeof(double));

	// earth to body transformation matrix [rad]
	F_EULER_TO_DCM(_earth2body, pInit->THETA[_PHI], pInit->THETA[_THETA], pInit->THETA[_PSI]);

	// body to earth transformation matrix
	M_MAT_TRANSPOSE(_body2earth, _earth2body);

	// create inertia ellipsoid [slug-ft**2]
	pIn->J[_X][_X] =  pInit->Ixx;
	pIn->J[_X][_Z] = -pInit->Ixz;
	pIn->J[_Y][_Y] =  pInit->Iyy;
	pIn->J[_Z][_X] = -pInit->Ixz;
	pIn->J[_Z][_Z] =  pInit->Izz;

	// transform the inertia ellipsoid
	_gammaInv  = 1.0 / (pInit->Ixx * pInit->Izz - pInit->Ixz * pInit->Ixz);
	pIn->Jinv[_X][_X] = pInit->Izz * _gammaInv;
	pIn->Jinv[_X][_Z] = pInit->Ixz * _gammaInv;
	pIn->Jinv[_Y][_Y] = 1.0 / pInit->Iyy;
	pIn->Jinv[_Z][_X] = pInit->Ixz * _gammaInv;
	pIn->Jinv[_Z][_Z] = pInit->Ixx * _gammaInv;

	// initialize the quaternion [rad]
	F_EULER_TO_QUATERNION(pX->Q, pInit->THETA[_PHI], pInit->THETA[_THETA], pInit->THETA[_PSI]);

	// initialize C.G linear accelerations (body coordinate) [ft / sec / sec]
	memset(pX->accel, 0, 3 * sizeof(double));
	
	// initialize C.G angular accelerations (body coordinate) [rad / sec / sec]
	memset(pX->alpha, 0, 3 * sizeof(double));

	// initialize C.G forces (body coordinate) [lb-force]
	memset(pIn->F, 0, 3 * sizeof(double));
	
	// initialize C.G moments (body coordinate) [lb-force * ft]
	memset(pIn->M, 0, 3 * sizeof(double));

	// initialize C.G angular velocities (body coordinate) [rad / sec]
	memcpy(pX->rate, pInit->pqr, 3 * sizeof(double));
	
	// initialize C.G linear velocities (body coordinate) [ft / sec]
	memcpy(pX->Vb, pInit->uvw, 3 * sizeof(double));
	
	// initialize C.G orientation (body coordinate) [rad]
	memcpy(pX->THETA, pInit->THETA, 3 * sizeof(double));
	
	// initialize C.G location (body coordinate) [ft]
	memcpy(pX->NED, pInit->NED, 3 * sizeof(double));

	// initialize CG velocity (earth coordinate) [ft / sec]
	pX->Ve[_X] = _body2earth[_X][_X] * pX->Vb[_X] + _body2earth[_X][_Y] * pX->Vb[_Y] + _body2earth[_X][_Z] * pX->Vb[_Z];
	pX->Ve[_Y] = _body2earth[_Y][_X] * pX->Vb[_X] + _body2earth[_Y][_Y] * pX->Vb[_Y] + _body2earth[_Y][_Z] * pX->Vb[_Z];
	pX->Ve[_Z] = _body2earth[_Z][_X] * pX->Vb[_X] + _body2earth[_Z][_Y] * pX->Vb[_Y] + _body2earth[_Z][_Z] * pX->Vb[_Z];

	// fill pIn mass [slug]
	pIn->m = pInit->m;
}

/**
* flat earth 6DOF state and state derivative calculation
*
* @return {struct} A/V 6DOF state derivative
* @return {struct} A/V 6DOF state
* @param  {struct} A/V forces and moments
**/
void F_SIX_DOF_DERIVATIVE(struct sixdofXdot_fe_def *pXdot, 
					  struct sixdofX_fe_def *pX,
					  struct sixdof_fe_inputs_def *pU) {
	// locals
	double _massInv,				// inverse of helicopter mass
	       _omega[3][3],			// rotational acceleration matrix
	       E[4][4],					// quaternion strap down matrix
	       _earth2body[3][3],		// earth -> body transformation matrix
	       _body2earth[3][3],		// body -> earth transformation matrix
		   _velocityBody[3],		// linear velocity (body coordinate system) [ft / sec]
	       _vectorTemp1[3],			// |
	       _vectorTemp2[3];			//  \ temporary vectors

	// housekeeping
	memset(_omega, 0, 9 * sizeof(double));
	memset(E, 0, 16 * sizeof(double));
	_massInv = 1.0 / pU->m;

	// construct Euler "omega cross" matrix
	_omega[_X][_Y] = -pX->pqr[_Z];
	_omega[_X][_Z] =  pX->pqr[_Y];
	_omega[_Y][_X] =  pX->pqr[_Z];
	_omega[_Y][_Z] = -pX->pqr[_X];
	_omega[_Z][_X] = -pX->pqr[_Y];
	_omega[_Z][_Y] =  pX->pqr[_X];

	// earth to body transformation matrix
	F_QUATERNION_TO_DCM(_earth2body, pX->Q);

	// body to earth transformation matrix
	M_MAT_TRANSPOSE(_body2earth, _earth2body);

	// transform velocity vector from earth to body coordinates
	_velocityBody[_X] = _earth2body[_X][_X] * pX->Vned[_X] + _earth2body[_X][_Y] * pX->Vned[_Y] + _earth2body[_X][_Z] * pX->Vned[_Z];
	_velocityBody[_Y] = _earth2body[_Y][_X] * pX->Vned[_X] + _earth2body[_Y][_Y] * pX->Vned[_Y] + _earth2body[_Y][_Z] * pX->Vned[_Z];
	_velocityBody[_Z] = _earth2body[_Z][_X] * pX->Vned[_X] + _earth2body[_Z][_Y] * pX->Vned[_Y] + _earth2body[_Z][_Z] * pX->Vned[_Z];

	// cross(omega, body velocity) (= w X v)
	_vectorTemp1[_X] = _omega[_X][_X] * _velocityBody[_X] + _omega[_X][_Y] * _velocityBody[_Y] + _omega[_X][_Z] * _velocityBody[_Z];
	_vectorTemp1[_Y] = _omega[_Y][_X] * _velocityBody[_X] + _omega[_Y][_Y] * _velocityBody[_Y] + _omega[_Y][_Z] * _velocityBody[_Z];
	_vectorTemp1[_Z] = _omega[_Z][_X] * _velocityBody[_X] + _omega[_Z][_Y] * _velocityBody[_Y] + _omega[_Z][_Z] * _velocityBody[_Z];

	// linear acceleration in body coordinates (a = F/m - w X v)
	_vectorTemp2[_X] = pU->F[_X] * _massInv - _vectorTemp1[_X];
	_vectorTemp2[_Y] = pU->F[_Y] * _massInv - _vectorTemp1[_Y];
	_vectorTemp2[_Z] = pU->F[_Z] * _massInv - _vectorTemp1[_Z];

	// test to see if certain body velocities are to be nullified
	if(pU->hold_u) {
		_vectorTemp2[_X] = 0.0;
	}
	if(pU->hold_v) {
		_vectorTemp2[_Y] = 0.0;
	}
	if(pU->hold_w) {
		_vectorTemp2[_Z] = 0.0;
	}

	// transform acceleration vector from body to earth
	pXdot->Vned_dot[_X] = _body2earth[_X][_X] * _vectorTemp2[_X] + _body2earth[_X][_Y] * _vectorTemp2[_Y] + _body2earth[_X][_Z] * _vectorTemp2[_Z];
	pXdot->Vned_dot[_Y] = _body2earth[_Y][_X] * _vectorTemp2[_X] + _body2earth[_Y][_Y] * _vectorTemp2[_Y] + _body2earth[_Y][_Z] * _vectorTemp2[_Z];
	pXdot->Vned_dot[_Z] = _body2earth[_Z][_X] * _vectorTemp2[_X] + _body2earth[_Z][_Y] * _vectorTemp2[_Y] + _body2earth[_Z][_Z] * _vectorTemp2[_Z];

	// fill pXdot->NED_dot array
	memcpy(pXdot->NED_dot, pX->Vned, 3 * sizeof(double));

	// angular  acceleration in body coordinates (wdot_body = -Jinv * Omega * J * w_body + Jinv * M_body)
	// J * w_body
	_vectorTemp1[_X] = pU->J[_X][_X] * pX->pqr[_X] + pU->J[_X][_Y] * pX->pqr[_Y] + pU->J[_X][_Z] * pX->pqr[_Z];
	_vectorTemp1[_Y] = pU->J[_Y][_X] * pX->pqr[_X] + pU->J[_Y][_Y] * pX->pqr[_Y] + pU->J[_Y][_Z] * pX->pqr[_Z];
	_vectorTemp1[_Z] = pU->J[_Z][_X] * pX->pqr[_X] + pU->J[_Z][_Y] * pX->pqr[_Y] + pU->J[_Z][_Z] * pX->pqr[_Z];

	// omega * (J * w_body)
	_vectorTemp2[_X] = _omega[_X][_X] * _vectorTemp1[_X] + _omega[_X][_Y] * _vectorTemp1[_Y] + _omega[_X][_Z] * _vectorTemp1[_Z];
	_vectorTemp2[_Y] = _omega[_Y][_X] * _vectorTemp1[_X] + _omega[_Y][_Y] * _vectorTemp1[_Y] + _omega[_Y][_Z] * _vectorTemp1[_Z];
	_vectorTemp2[_Z] = _omega[_Z][_X] * _vectorTemp1[_X] + _omega[_Z][_Y] * _vectorTemp1[_Y] + _omega[_Z][_Z] * _vectorTemp1[_Z];

	// 	Jinv * (Omega * J * w_body)
	_vectorTemp1[_X] = pU->Jinv[_X][_X] * _vectorTemp2[_X] + pU->Jinv[_X][_Y] * _vectorTemp2[_Y] + pU->Jinv[_X][_Z] * _vectorTemp2[_Z];
	_vectorTemp1[_Y] = pU->Jinv[_Y][_X] * _vectorTemp2[_X] + pU->Jinv[_Y][_Y] * _vectorTemp2[_Y] + pU->Jinv[_Y][_Z] * _vectorTemp2[_Z];
	_vectorTemp1[_Z] = pU->Jinv[_Z][_X] * _vectorTemp2[_X] + pU->Jinv[_Z][_Y] * _vectorTemp2[_Y] + pU->Jinv[_Z][_Z] * _vectorTemp2[_Z];

	// sign inversion
	_vectorTemp1[_X] *= -1.0;
	_vectorTemp1[_Y] *= -1.0;
	_vectorTemp1[_Z] *= -1.0;

	// Jinv * M_body
	_vectorTemp2[_X] = pU->Jinv[_X][_X] * pU->M[_X] + pU->Jinv[_X][_Y] * pU->M[_Y] + pU->Jinv[_X][_Z] * pU->M[_Z];
	_vectorTemp2[_Y] = pU->Jinv[_Y][_X] * pU->M[_X] + pU->Jinv[_Y][_Y] * pU->M[_Y] + pU->Jinv[_Y][_Z] * pU->M[_Z];
	_vectorTemp2[_Z] = pU->Jinv[_Z][_X] * pU->M[_X] + pU->Jinv[_Z][_Y] * pU->M[_Y] + pU->Jinv[_Z][_Z] * pU->M[_Z];

	// wdot_body
	pXdot->pqr_dot[_X] = _vectorTemp1[_X] + _vectorTemp2[_X];
	pXdot->pqr_dot[_Y] = _vectorTemp1[_Y] + _vectorTemp2[_Y];
	pXdot->pqr_dot[_Z] = _vectorTemp1[_Z] + _vectorTemp2[_Z];

	// test to see if certain body angular rates are to be nullified
	if(pU->hold_p) {
		pXdot->pqr_dot[_X] = 0.0;
	}
	if(pU->hold_q) {
		pXdot->pqr_dot[_Y] = 0.0;
	}
	if(pU->hold_r) {
		pXdot->pqr_dot[_Z] = 0.0;
	}

	// calculate quaternion strap down matrix ("E" matrix)
	_vectorTemp1[_X] = 0.5 * pX->pqr[_X];
	_vectorTemp1[_Y] = 0.5 * pX->pqr[_Y];
	_vectorTemp1[_Z] = 0.5 * pX->pqr[_Z];

	E[_qW][_qX] = -_vectorTemp1[_X];
	E[_qW][_qY] = -_vectorTemp1[_Y];
	E[_qW][_qZ] = -_vectorTemp1[_Z];

	E[_qX][_qW] =  _vectorTemp1[_X];
	E[_qX][_qY] =  _vectorTemp1[_Z];
	E[_qX][_qZ] = -_vectorTemp1[_Y];

	E[_qY][_qW] =  _vectorTemp1[_Y];
	E[_qY][_qX] = -_vectorTemp1[_Z];
	E[_qY][_qZ] =  _vectorTemp1[_X];

	E[_qZ][_qW] =  _vectorTemp1[_Z];
	E[_qZ][_qX] =  _vectorTemp1[_Y];
	E[_qZ][_qY] = -_vectorTemp1[_X];

	// calculate quaternion derivative (q_dot = _qW.5 * E * q)
	pXdot->Q_dot[_qW] = E[_qW][_qW] * pX->Q[_qW] + E[_qW][_qX] * pX->Q[_qX] + E[_qW][_qY] * pX->Q[_qY] + E[_qW][_qZ] * pX->Q[_qZ];
	pXdot->Q_dot[_qX] = E[_qX][_qW] * pX->Q[_qW] + E[_qX][_qX] * pX->Q[_qX] + E[_qX][_qY] * pX->Q[_qY] + E[_qX][_qZ] * pX->Q[_qZ];
	pXdot->Q_dot[_qY] = E[_qY][_qW] * pX->Q[_qW] + E[_qY][_qX] * pX->Q[_qX] + E[_qY][_qY] * pX->Q[_qY] + E[_qY][_qZ] * pX->Q[_qZ];
	pXdot->Q_dot[_qZ] = E[_qZ][_qW] * pX->Q[_qW] + E[_qZ][_qX] * pX->Q[_qX] + E[_qZ][_qY] * pX->Q[_qY] + E[_qZ][_qZ] * pX->Q[_qZ];
}

/**
* flat earth 6DOF motion equation integration (RK4)
*
* @return {struct} A/V state derivative
* @return {struct} A/V state
* @param  {struct} A/V forces and moments
* @param  {double} simulation time step [sec]
**/
void F_SIX_DOF_INTEGRATE(struct sixdofXdot_fe_def *pXdot, 
						 struct sixdofX_fe_def *pX, 
						 struct sixdof_fe_inputs_def *pU, 
						 double dt) {
	// locals
	struct sixdofX_fe_def X;	    // to run the derivative function
	struct sixdofX_fe_def X0;		// to backup the input state
	struct sixdofXdot_fe_def Xdot;	// to run the derivative function
	double k1[13], k2[13], k3[13], k4[13];	// RK4 steps

	// current input state backup
	memcpy(X0.NED,  pX->NED,  3 * sizeof(double));
	memcpy(X0.pqr,  pX->pqr,  3 * sizeof(double));
	memcpy(X0.Vned, pX->Vned, 3 * sizeof(double));
	memcpy(X0.Q,    pX->Q,    4 * sizeof(double));

	// create X state
	memcpy(X.NED,  X0.NED,  3 * sizeof(double));
	memcpy(X.pqr,  X0.pqr,  3 * sizeof(double));
	memcpy(X.Vned, X0.Vned, 3 * sizeof(double));
	memcpy(X.Q,    X0.Q,    4 * sizeof(double));
	F_QUATERNION_NORMALIZATION(X.Q);

	// calculate state derivative
	F_SIX_DOF_DERIVATIVE(&Xdot, &X, pU);

	// RK4 first step
	k1[0]  = Xdot.NED_dot[0]  * dt;
	k1[1]  = Xdot.NED_dot[1]  * dt;
	k1[2]  = Xdot.NED_dot[2]  * dt;
	k1[3]  = Xdot.pqr_dot[0]  * dt;
	k1[4]  = Xdot.pqr_dot[1]  * dt;
	k1[5]  = Xdot.pqr_dot[2]  * dt;
	k1[6]  = Xdot.Vned_dot[0] * dt;
	k1[7]  = Xdot.Vned_dot[1] * dt;
	k1[8]  = Xdot.Vned_dot[2] * dt;
	k1[9]  = Xdot.Q_dot[0]    * dt;
	k1[10] = Xdot.Q_dot[1]    * dt;
	k1[11] = Xdot.Q_dot[2]    * dt;
	k1[12] = Xdot.Q_dot[3]    * dt;

	// calculate next (half) step state value
	X.NED[0]  = X0.NED[0]  + k1[0]  * 0.5;
	X.NED[1]  = X0.NED[1]  + k1[1]  * 0.5;
	X.NED[2]  = X0.NED[2]  + k1[2]  * 0.5;
	X.pqr[0]  = X0.pqr[0]  + k1[3]  * 0.5;
	X.pqr[1]  = X0.pqr[1]  + k1[4]  * 0.5;
	X.pqr[2]  = X0.pqr[2]  + k1[5]  * 0.5;
	X.Vned[0] = X0.Vned[0] + k1[6]  * 0.5;
	X.Vned[1] = X0.Vned[1] + k1[7]  * 0.5;
	X.Vned[2] = X0.Vned[2] + k1[8]  * 0.5;
	X.Q[0]    = X0.Q[0]    + k1[9]  * 0.5;
	X.Q[1]    = X0.Q[1]    + k1[10] * 0.5;
	X.Q[2]    = X0.Q[2]    + k1[11] * 0.5;
	X.Q[3]    = X0.Q[3]    + k1[12] * 0.5;
	F_QUATERNION_NORMALIZATION(X.Q);

	// keep state derivative (needed for output)
	memcpy(pXdot->NED_dot,  Xdot.NED_dot,  3 * sizeof(double));
	memcpy(pXdot->pqr_dot,  Xdot.pqr_dot,  3 * sizeof(double));
	memcpy(pXdot->Vned_dot, Xdot.Vned_dot, 3 * sizeof(double));
	memcpy(pXdot->Q_dot,    Xdot.Q_dot,    4 * sizeof(double));
	
	// calculate state derivative
	F_SIX_DOF_DERIVATIVE(&Xdot, &X, pU);

	// RK4 second step
	k2[0]  = Xdot.NED_dot[0]  * dt;
	k2[1]  = Xdot.NED_dot[1]  * dt;
	k2[2]  = Xdot.NED_dot[2]  * dt;
	k2[3]  = Xdot.pqr_dot[0]  * dt;
	k2[4]  = Xdot.pqr_dot[1]  * dt;
	k2[5]  = Xdot.pqr_dot[2]  * dt;
	k2[6]  = Xdot.Vned_dot[0] * dt;
	k2[7]  = Xdot.Vned_dot[1] * dt;
	k2[8]  = Xdot.Vned_dot[2] * dt;
	k2[9]  = Xdot.Q_dot[0]    * dt;
	k2[10] = Xdot.Q_dot[1]    * dt;
	k2[11] = Xdot.Q_dot[2]    * dt;
	k2[12] = Xdot.Q_dot[3]    * dt;
	F_QUATERNION_NORMALIZATION(X.Q);

	// calculate state derivative
	F_SIX_DOF_DERIVATIVE(&Xdot, &X, pU);

	// RK4 third step
	k3[0]  = Xdot.NED_dot[0]  * dt;
	k3[1]  = Xdot.NED_dot[1]  * dt;
	k3[2]  = Xdot.NED_dot[2]  * dt;
	k3[3]  = Xdot.pqr_dot[0]  * dt;
	k3[4]  = Xdot.pqr_dot[1]  * dt;
	k3[5]  = Xdot.pqr_dot[2]  * dt;
	k3[6]  = Xdot.Vned_dot[0] * dt;
	k3[7]  = Xdot.Vned_dot[1] * dt;
	k3[8]  = Xdot.Vned_dot[2] * dt;
	k3[9]  = Xdot.Q_dot[0]    * dt;
	k3[10] = Xdot.Q_dot[1]    * dt;
	k3[11] = Xdot.Q_dot[2]    * dt;
	k3[12] = Xdot.Q_dot[3]    * dt;
	F_QUATERNION_NORMALIZATION(X.Q);
	
	// calculate state derivative
	F_SIX_DOF_DERIVATIVE(&Xdot, &X, pU);

	// RK4 fourth step
	k4[0]  = Xdot.NED_dot[0]  * dt;
	k4[1]  = Xdot.NED_dot[1]  * dt;
	k4[2]  = Xdot.NED_dot[2]  * dt;
	k4[3]  = Xdot.pqr_dot[0]  * dt;
	k4[4]  = Xdot.pqr_dot[1]  * dt;
	k4[5]  = Xdot.pqr_dot[2]  * dt;
	k4[6]  = Xdot.Vned_dot[0] * dt;
	k4[7]  = Xdot.Vned_dot[1] * dt;
	k4[8]  = Xdot.Vned_dot[2] * dt;
	k4[9]  = Xdot.Q_dot[0]    * dt;
	k4[10] = Xdot.Q_dot[1]    * dt;
	k4[11] = Xdot.Q_dot[2]    * dt;
	k4[12] = Xdot.Q_dot[3]    * dt;
	F_QUATERNION_NORMALIZATION(X.Q);

	// RK4 final step - result for state prorogation
	pX->NED[0]  = X0.NED[0]  + (k1[0]  + k4[0])  * C_ONE_SIXTH  + (k2[0]  + k3[0])  * C_ONE_THIRD;
	pX->NED[1]  = X0.NED[1]  + (k1[1]  + k4[1])  * C_ONE_SIXTH  + (k2[1]  + k3[1])  * C_ONE_THIRD;
	pX->NED[2]  = X0.NED[2]  + (k1[2]  + k4[2])  * C_ONE_SIXTH  + (k2[2]  + k3[2])  * C_ONE_THIRD;
	pX->pqr[0]  = X0.pqr[0]  + (k1[3]  + k4[3])  * C_ONE_SIXTH  + (k2[3]  + k3[3])  * C_ONE_THIRD;
	pX->pqr[1]  = X0.pqr[1]  + (k1[4]  + k4[4])  * C_ONE_SIXTH  + (k2[4]  + k3[4])  * C_ONE_THIRD;
	pX->pqr[2]  = X0.pqr[2]  + (k1[5]  + k4[5])  * C_ONE_SIXTH  + (k2[5]  + k3[5])  * C_ONE_THIRD;
	pX->Vned[0] = X0.Vned[0] + (k1[6]  + k4[6])  * C_ONE_SIXTH  + (k2[6]  + k3[6])  * C_ONE_THIRD;
	pX->Vned[1] = X0.Vned[1] + (k1[7]  + k4[7])  * C_ONE_SIXTH  + (k2[7]  + k3[7])  * C_ONE_THIRD;
	pX->Vned[2] = X0.Vned[2] + (k1[8]  + k4[8])  * C_ONE_SIXTH  + (k2[8]  + k3[8])  * C_ONE_THIRD;
	pX->Q[0]    = X0.Q[0]    + (k1[9]  + k4[9])  * C_ONE_SIXTH  + (k2[9]  + k3[9])  * C_ONE_THIRD;
	pX->Q[1]    = X0.Q[1]    + (k1[10] + k4[10]) * C_ONE_SIXTH  + (k2[10] + k3[10]) * C_ONE_THIRD;
	pX->Q[2]    = X0.Q[2]    + (k1[11] + k4[11]) * C_ONE_SIXTH  + (k2[11] + k3[11]) * C_ONE_THIRD;
	pX->Q[3]    = X0.Q[3]    + (k1[12] + k4[12]) * C_ONE_SIXTH  + (k2[12] + k3[12]) * C_ONE_THIRD;
	F_QUATERNION_NORMALIZATION(pX->Q);
}


/**
* flat earth 6DOF kernel
*
* @param {struct} A/V state vector
* @param {struct} A/V forces and moments
* @param {double} simulation time step
**/
void F_SIX_DOF(struct sixdof_fe_state_def *pX, struct sixdof_fe_inputs_def *pU, double dt) {
	double _earth2body[3][3],				// earth -> body  transformation matrix
	       _body2earth[3][3],				// body  -> earth transformation matrix
	       _massInv;						// A/V inverse of mass
	struct sixdofX_fe_def X;				// state vector
	struct sixdofXdot_fe_def Xdot;			// state derivative vector

	// earth to body transformation matrix
	F_QUATERNION_TO_DCM(_earth2body, pX->Q);

	// body to earth transformation matrix
	M_MAT_TRANSPOSE(_body2earth, _earth2body);

	// state vector  creation
	memcpy(X.pqr,  pX->rate, 3 * sizeof(double));
	memcpy(X.Vned, pX->Ve,   3 * sizeof(double));
	memcpy(X.NED,  pX->NED,  3 * sizeof(double));
	memcpy(X.Q,    pX->Q,    4 * sizeof(double));
	F_QUATERNION_NORMALIZATION(X.Q);

	// integrate state vector rate
	F_SIX_DOF_INTEGRATE(&Xdot, &X, pU, dt);

	// solve for the unknown state values for the next time step
	memcpy(pX->alpha, Xdot.pqr_dot, 3 * sizeof(double));
	memcpy(pX->rate,  X.pqr,        3 * sizeof(double));
	memcpy(pX->accel, pU->F,        3 * sizeof(double));
	memcpy(pX->Ve,    X.Vned,       3 * sizeof(double));
	memcpy(pX->NED,   X.NED,        3 * sizeof(double));
	memcpy(pX->Q,     X.Q,          4 * sizeof(double));

	// divide acceleration by mass
	_massInv = 1 / pU->m;
	pX->accel[_X] *= _massInv;
	pX->accel[_Y] *= _massInv;
	pX->accel[_Z] *= _massInv;

	// transform quaternion to Euler angles
	F_QUATERNION_TO_EULER(X.Q, pX->THETA);
	
	// calculate velocity in body coordinate system
	pX->Vb[_X] = _earth2body[_X][_X] * X.Vned[_X] + _earth2body[_X][_Y] * X.Vned[_Y] + _earth2body[_X][_Z] * X.Vned[_Z];
	pX->Vb[_Y] = _earth2body[_Y][_X] * X.Vned[_X] + _earth2body[_Y][_Y] * X.Vned[_Y] + _earth2body[_Y][_Z] * X.Vned[_Z];
	pX->Vb[_Z] = _earth2body[_Z][_X] * X.Vned[_X] + _earth2body[_Z][_Y] * X.Vned[_Y] + _earth2body[_Z][_Z] * X.Vned[_Z];
}

/**
* second order actuator model derivative
* 
* @return {double} actuator state derivative
* @return {double} actuator state
* @param  {double} dynamic system time [sec]
* @param  {double} dynamic system input
* @param  {double} dynamic system arguments {frequency (omega), damping factor (zeta)}
**/
void F_SERVO_MODEL_DERIVATIVE(double *Xdot, double *X, double *t, double *u, double *args) {
	// locals
	double wn   = args[0],
		   zeta = args[1];

	// servo state derivatives
	Xdot[0] = X[1];
	Xdot[1]  = *u - 2.0 * zeta * wn * X[1] - wn * wn * X[0];
}


/**
* second order actuator model propagation
*
* @return {struct} servo new angle
* @param  {struct} servo input
* @param  {double} time step
**/
void F_SERVO_MODEL(struct servo_state_def *pX, struct servo_inputs_def *pIn, double dt) {
	// locals
	double args[2], X[2], Xdot[2], u, y, wn, zeta;

	// state vector
	X[0] = pX->X[0];
	X[1] = pX->X[1];

	// servo characteristics
	wn   = pIn->wn;
	zeta = pIn->zeta;
	
	// arguments vector fill
	args[0] = wn;
	args[1] = zeta;

	// servo control input
	u = pIn->command;

	// propagate servo state
	F_RK4(X, Xdot, 0, &u, args, 2, dt, &F_SERVO_MODEL_DERIVATIVE);

	// servo output (y = X[1] + wn*wn*X[0])
	y  = X[1] + X[0] * wn * wn;

	// assign output
	pX->output = y;

	// save servo state in state vector
	pX->X[0] = X[0];
	pX->X[1] = X[1];
}

/**
* collision/landing gear model using vertical displacement method.
*
* @param  {struct} gear parameters and state
* @return {struct} gear forces and moments
**/
void F_GEAR_COLLISION(struct gear_inputs_def *pIn, struct gear_outputs_def *pOut) {
	// locals
	double _omega[3][3],			// rotational velocity matrix
	       _displacement,			// the displacement of the contact force under ground
	       _vectorTemp1[3],			// temporary vector
	       _vectorTemp2[3],			// temporary vector
	  	   _wheelVelocity_body[3],	// wheel velocity vector (body)
	  	   _wheelVelocity_earth[3],	// wheel velocity vector (earth)
	  	   _wheelPosition_body[3],	// wheel position vector (body)
	  	   _wheelPosition_earth[3],	// wheel position vector (earth)
	  	   _body2earth[3][3],		// body to earth transformation matrix
	  	   _earth2body[3][3],		// earth to body transformation matrix
	  	   _whellForces_earth[3],		// force on wheel in earth frame
	  	   _sinRotation, _cosRotation;	// sine and cosine of wheel rotation angle

	// housekeeping
	memcpy(_wheelPosition_body, pIn->cg2point, 3 * sizeof(double));
	memset(_omega, 0, 9 * sizeof(double));

	// earth to body transformation matrix (yaw is not needed)
	F_EULER_TO_DCM(_earth2body, pIn->THETA[_PHI], pIn->THETA[_THETA], 0.0);

	// body to earth transformation matrix
	M_MAT_TRANSPOSE(_body2earth, _earth2body);

	// construct Euler "omega cross" matrix
	_omega[_X][_Y] = -pIn->pqr[_Z];
	_omega[_X][_Z] =  pIn->pqr[_Y];
	_omega[_Y][_X] =  pIn->pqr[_Z];
	_omega[_Y][_Z] = -pIn->pqr[_X];
	_omega[_Z][_X] = -pIn->pqr[_Y];
	_omega[_Z][_Y] =  pIn->pqr[_X];

	// calculate wheel position on earth
	_wheelPosition_earth[_X] = _body2earth[_X][_X] * _wheelPosition_body[_X] +
		                       _body2earth[_X][_Y] * _wheelPosition_body[_Y] +
							   _body2earth[_X][_Z] * _wheelPosition_body[_Z];
	_wheelPosition_earth[_Y] = _body2earth[_Y][_X] * _wheelPosition_body[_X] +
						       _body2earth[_Y][_Y] * _wheelPosition_body[_Y] +
							   _body2earth[_Y][_Z] * _wheelPosition_body[_Z];
	_wheelPosition_earth[_Z] = _body2earth[_Z][_X] * _wheelPosition_body[_X] +
							   _body2earth[_Z][_Y] * _wheelPosition_body[_Y] +
							   _body2earth[_Z][_Z] * _wheelPosition_body[_Z];
	_wheelPosition_earth[_Z] += pIn->altitude;

	// calculate displacement
	_displacement = _wheelPosition_earth[_Z];

	if(_displacement > 0.0 ) {		// wheel is underground
		// demand wheel position to be on ground
		_wheelPosition_earth[_Z] = 0.0;

		// transform position demand to body coordinate system
		_wheelPosition_body[_X] = _earth2body[_X][_X] * _wheelPosition_earth[_X] +
								  _earth2body[_X][_Y] * _wheelPosition_earth[_Y] +
								  _earth2body[_X][_Z] * _wheelPosition_earth[_Z];
		_wheelPosition_body[_Y] = _earth2body[_Y][_X] * _wheelPosition_earth[_X] +
								  _earth2body[_Y][_Y] * _wheelPosition_earth[_Y] +
								  _earth2body[_Y][_Z] * _wheelPosition_earth[_Z];
		_wheelPosition_body[_Z] = _earth2body[_Z][_X] * _wheelPosition_earth[_X] +
								  _earth2body[_Z][_Y] * _wheelPosition_earth[_Y] +
								  _earth2body[_Z][_Z] * _wheelPosition_earth[_Z];

		// calculate wheel velocity (body coordinate system)
		_vectorTemp1[_X] = _omega[_X][_X] * _wheelPosition_body[_X] +
						   _omega[_X][_Y] * _wheelPosition_body[_Y] +
						   _omega[_X][_Z] * _wheelPosition_body[_Z];
		_vectorTemp1[_Y] = _omega[_Y][_X] * _wheelPosition_body[_X] +
						   _omega[_Y][_Y] * _wheelPosition_body[_Y] +
						   _omega[_Y][_Z] * _wheelPosition_body[_Z];
		_vectorTemp1[_Z] = _omega[_Z][_X] * _wheelPosition_body[_X] +
						   _omega[_Z][_Y] * _wheelPosition_body[_Y] +
						   _omega[_Z][_Z] * _wheelPosition_body[_Z];

		_wheelVelocity_body[_X] = _vectorTemp1[_X] + pIn->uvw[_X];
		_wheelVelocity_body[_Y] = _vectorTemp1[_Y] + pIn->uvw[_Y];
		_wheelVelocity_body[_Z] = _vectorTemp1[_Z] + pIn->uvw[_Z];

		// transform wheel velocity to earth coordinate system
		_wheelVelocity_earth[_X] = _body2earth[_X][_X] * _wheelVelocity_body[_X] +
			                       _body2earth[_X][_Y] * _wheelVelocity_body[_Y] +
								   _body2earth[_X][_Z] * _wheelVelocity_body[_Z];
		_wheelVelocity_earth[_Y] = _body2earth[_Y][_X] * _wheelVelocity_body[_X] +
								   _body2earth[_Y][_Y] * _wheelVelocity_body[_Y] +
								   _body2earth[_Y][_Z] * _wheelVelocity_body[_Z];
		_wheelVelocity_earth[_Z] = _body2earth[_Z][_X] * _wheelVelocity_body[_X] +
								   _body2earth[_Z][_Y] * _wheelVelocity_body[_Y] +
								   _body2earth[_Z][_Z] * _wheelVelocity_body[_Z];

		// if the wheel has steering ability
		if(pIn->rotation != 0.0 ) {
			// rotate the body velocity by the rotation of the wheel (+ rotation about Z body axis)
			_sinRotation = sin(pIn->rotation);
			_cosRotation = cos(pIn->rotation);
			_vectorTemp1[_X] =  pIn->uvw[_X] * _cosRotation + pIn->uvw[_Y] * _sinRotation;
			_vectorTemp1[_Y] = -pIn->uvw[_X] * _sinRotation + pIn->uvw[_Y] * _cosRotation;
			_vectorTemp1[_Z] =  pIn->uvw[_Z];

			// transform velocity to body coordinate system
			_vectorTemp2[_X] = _omega[_X][_X] * _wheelPosition_body[_X] +
							   _omega[_X][_Y] * _wheelPosition_body[_Y] +
							   _omega[_X][_Z] * _wheelPosition_body[_Z];
			_vectorTemp2[_Y] = _omega[_Y][_X] * _wheelPosition_body[_X] +
							   _omega[_Y][_Y] * _wheelPosition_body[_Y] +
							   _omega[_Y][_Z] * _wheelPosition_body[_Z];
			_vectorTemp2[_Z] = _omega[_Z][_X] * _wheelPosition_body[_X] +
							   _omega[_Z][_Y] * _wheelPosition_body[_Y] +
							   _omega[_Z][_Z] * _wheelPosition_body[_Z];

			_wheelVelocity_body[_X] = _vectorTemp1[_X] + _vectorTemp2[_X];
			_wheelVelocity_body[_Y] = _vectorTemp1[_Y] + _vectorTemp2[_Y];
			_wheelVelocity_body[_Z] = _vectorTemp1[_Z] + _vectorTemp2[_Z];

			// transform velocity to earth coordinate system
			_wheelVelocity_earth[_X] = _body2earth[_X][_X] * _wheelVelocity_body[_X] +
									   _body2earth[_X][_Y] * _wheelVelocity_body[_Y] +
									   _body2earth[_X][_Z] * _wheelVelocity_body[_Z];
			_wheelVelocity_earth[_Y] = _body2earth[_Y][_X] * _wheelVelocity_body[_X] +
									   _body2earth[_Y][_Y] * _wheelVelocity_body[_Y] +
									   _body2earth[_Y][_Z] * _wheelVelocity_body[_Z];
			_wheelVelocity_earth[_Z] = _body2earth[_Z][_X] * _wheelVelocity_body[_X] +
									   _body2earth[_Z][_Y] * _wheelVelocity_body[_Y] +
									   _body2earth[_Z][_Z] * _wheelVelocity_body[_Z];
		}

		// vertical touch force (earth coordinate system)
		_whellForces_earth[_Z] = pIn->k * _displacement + pIn->b * _wheelVelocity_earth[_Z];

		// lateral touch forces on wheel (earth coordinate system) - X
		if(_wheelVelocity_earth[_X] > 0.0) {
			_whellForces_earth[_X] = -pIn->mu_x * _whellForces_earth[_Z] *_wheelVelocity_earth[_X] / M_ABS(_wheelVelocity_earth[_X]);
		} else {
			_whellForces_earth[_X] = 0.0;
		}

		// lateral touch forces on wheel (earth coordinate system) - Y
		if(_wheelVelocity_earth[_Y] > 0.0) {
			_whellForces_earth[_Y] = -pIn->mu_y * _whellForces_earth[_Z] * _wheelVelocity_earth[_Y] / M_ABS(_wheelVelocity_earth[_Y]);
		} else {
			_whellForces_earth[_Y] = 0.0;
		}

		// vertical force sign
		_whellForces_earth[_Z] *= -1.0;

		// transform forces to body coordinate system
		pOut->F[_X] = _earth2body[_X][_X] * _whellForces_earth[_X] +
					  _earth2body[_X][_Y] * _whellForces_earth[_Y] +
					  _earth2body[_X][_Z] * _whellForces_earth[_Z];
		pOut->F[_Y] = _earth2body[_Y][_X] * _whellForces_earth[_X] +
					  _earth2body[_Y][_Y] * _whellForces_earth[_Y] +
					  _earth2body[_Y][_Z] * _whellForces_earth[_Z];
		pOut->F[_Z] = _earth2body[_Z][_X] * _whellForces_earth[_X] +
					  _earth2body[_Z][_Y] * _whellForces_earth[_Y] +
					  _earth2body[_Z][_Z] * _whellForces_earth[_Z];

		// calculate wheel moments (cross of position and force) (earth coordinate system)
		_vectorTemp2[_X] = _whellForces_earth[_Z] * _wheelPosition_earth[_Y] - _whellForces_earth[_Y] * _wheelPosition_earth[_Z];
		_vectorTemp2[_Y] = _whellForces_earth[_X] * _wheelPosition_earth[_Z] - _whellForces_earth[_Z] * _wheelPosition_earth[_X];
		_vectorTemp2[_Z] = _whellForces_earth[_Y] * _wheelPosition_earth[_X] - _whellForces_earth[_X] * _wheelPosition_earth[_Y];

		// transform moments to earth coordinate system
		pOut->M[_X] = _earth2body[_X][_X] * _vectorTemp2[_X] +
			          _earth2body[_X][_Y] * _vectorTemp2[_Y] +
					  _earth2body[_X][_Z] * _vectorTemp2[_Z];
		pOut->M[_Y] = _earth2body[_Y][_X] * _vectorTemp2[_X] +
			          _earth2body[_Y][_Y] * _vectorTemp2[_Y] +
					  _earth2body[_Y][_Z] * _vectorTemp2[_Z];
		pOut->M[_Z] = _earth2body[_Z][_X] * _vectorTemp2[_X] +
			          _earth2body[_Z][_Y] * _vectorTemp2[_Y] +
					  _earth2body[_Z][_Z] * _vectorTemp2[_Z];
	} else {  // wheel in on air
		memset(pOut->F, 0, 3 * sizeof(double));
		memset(pOut->M, 0, 3 * sizeof(double));
	}
}
	
/**
* wind model initialization
*
* @param  {struct} wind limits
* @return {struct} wind state
**/
void F_WIND_MODEL_INIT(struct wind_inputs_def *pIn, struct wind_state_def *pX) {
	// initialize state
	memset(pX->Ve, 0, 3 * sizeof(double));
	memset(pX->X,  0, 3 * sizeof(double));

	// seed random number generator
	srand(pIn->seed);
}

/**
* second order wind model built as a dynamic system to be propagated using RK4
*
* @return {double} wind state derivative
* @return {double} wind state
* @param  {double} dynamic system time [sec]
* @param  {double} dynamic system input
* @param  {double} dynamic system arguments {frequency (omega), damping factor (zeta)}
**/
void F_WIND_MODEL_DERIVATIVE(double *Xdot, double *X, double *t, double *u, double *args) {
	// locals
	double wnSqr  = args[0] * args[0],
		   twoZetaWn = -2.0 * args[0] * args[1];

	// wind north / east / down component component
	Xdot[0] = X[1]; Xdot[1] = twoZetaWn * X[1] - wnSqr * X[0] + u[0];
	Xdot[2] = X[3]; Xdot[3] = twoZetaWn * X[3] - wnSqr * X[2] + u[1];
	Xdot[4] = X[5]; Xdot[5] = twoZetaWn * X[5] - wnSqr * X[4] + u[2];
}

/**
* wind model propagation function
*
* @param  {struct} wind limits
* @return {struct} wind state
* @param  {double} simulation time
**/
void F_WIND_MODEL(struct wind_inputs_def *pIn, struct wind_state_def *pX, double dt) {
	// constants
	static const double rand_max = 1.0 / (0.5 * (double)(RAND_MAX));

	// locals
	double Xdot[6];
	double args[2], argsSqr0;
	double U[3];
	double xx, yy, zz;

	// wind module arguments
	args[0]  = 1.5;
	args[1]  = 0.00005;
	argsSqr0 = 2.25;     // args[0]**2

	// random number
	xx = -1.0 * (double)(rand()) * rand_max;
	yy = -1.0 * (double)(rand()) * rand_max;
	zz = -1.0 * (double)(rand()) * rand_max;

	// wind state space model input
	U[0] = 10.0 * xx;
	U[1] = 10.0 * yy;
	U[2] = 10.0 * yy;

	// wind state integrations
	F_RK4(pX->X, Xdot, 0, U, args, 6, dt, &F_WIND_MODEL_DERIVATIVE);

	// wind
	pX->Ve[0] = argsSqr0 * pX->X[0] * pIn->horz_max;
	pX->Ve[1] = argsSqr0 * pX->X[2] * pIn->horz_max;
	pX->Ve[2] = argsSqr0 * pX->X[4] * pIn->vert_max;
}

/**
* WGS-84 gravity model
*
* @param  {struct} geodetic location
* @return {struct} gravity vector at location
**/
void F_GRAVITY_MODEL(struct grav_inputs_def *pIn, struct grav_outputs_def *pOut) {
	// locals
	double _Recef,				// distance to ECEF
		   _geoLat,				// geocentric latitude
		   _geoLatSin,			// sine of geocentric latitude
		   N,					// parameter in gravity model
		   sinLat, cosLat,		// latitude trigonometric functions
	       _ECEF[3],			// position in ECEF
		   _LLH[3];				// position in geodetic coordinates

	// housekeeping
	sinLat = sin(pIn->latitude);
	cosLat = cos(pIn->latitude);

	// geodesic coordinate system [rad, m], longitude is of no importance
	_LLH[_LAT] = pIn->latitude;
	_LLH[_LON] = 0.0;
	_LLH[_ALT] = pIn->altitude * C_FOOT_TO_METER;

	// calculate the position in ECEF coordinates
	F_GEODETIC_TO_ECEF(_LLH, _ECEF);

	// calculate distance from earth center [m]
	_Recef = sqrt(_ECEF[_X] * _ECEF[_X] + _ECEF[_Y] * _ECEF[_Y] + _ECEF[_Z] * _ECEF[_Z]);

	// calculate the geocentric latitude [rad]
	_geoLat    = asin(_ECEF[_Z] / _Recef);
	_geoLatSin = sin(_geoLat);

	// gravity model parameters
	N = C_WGS84_MAJOR / sqrt(1.0 - C_WGS84_ECENT_SQR * sinLat * sinLat);

	// calculate Gravitation vector in earth coordinate system
	pOut->G[_X]  = C_WGS84_MAJOR;
	pOut->G[_X] /= _Recef;
	pOut->G[_X] *= -pOut->G[_X];
	pOut->G[_X] *= 3.0;
	pOut->G[_X] *= C_J2;
	pOut->G[_X] *= _geoLatSin;
	pOut->G[_X] *= cos(_geoLat);
	pOut->G[_X] *= C_GM / M_SQR(_Recef);

	pOut->G[_Y] = 0.0;

	pOut->G[_Y]  = C_WGS84_MAJOR;
	pOut->G[_Y] /= _Recef;
	pOut->G[_Y] *= -pOut->G[_Y];
	pOut->G[_Y] *= C_J2;
	pOut->G[_Y] *= 3.0 * M_SQR(_geoLatSin) - 1.0;
	pOut->G[_Y] *= -1.5;
	pOut->G[_Y] += 1.0;
	pOut->G[_Y] *= C_GM / M_SQR(_Recef);

	// calculate gravity vector in earth coordinate system (g[_Y] used as a temporary holder at the beginning)
	pOut->g[_Y]  = pIn->altitude;
	pOut->g[_Y] *= C_FOOT_TO_METER;
	pOut->g[_Y] += N;
	pOut->g[_Y] *= cosLat;
	pOut->g[_Y] *= C_EARTH_SPIN;
	pOut->g[_Y] *= -C_EARTH_SPIN;

	pOut->g[_X]  = pOut->g[_Y];
	pOut->g[_X] *= sinLat;
	pOut->g[_X] += pOut->G[_X];

	pOut->g[_Z]  = pOut->g[_Y];
	pOut->g[_Z] *= cosLat;
	pOut->g[_Z] += pOut->G[_Z];

	pOut->g[_Y] = 0.0;

	// [m / sec / sec] to [ft / sec / sec]
	pOut->G[_X] *= C_METER_TO_FOOT;
	pOut->G[_Y] *= C_METER_TO_FOOT;
	pOut->G[_Z] *= C_METER_TO_FOOT;
	pOut->g[_X] *= C_METER_TO_FOOT;
	pOut->g[_Y] *= C_METER_TO_FOOT;
	pOut->g[_Z] *= C_METER_TO_FOOT;
}
