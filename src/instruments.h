#ifndef instruments_h
#define instruments_h


#include "glos.h" // MS specific stuff

#include <stdio.h>
#include <GL/gl.h> // system OpenGL includes
#include <GL/glu.h>
//#include <GL/glaux.h>
#include <math.h>
#include <GL/glut.h>

#include "matlib.h"		// for some of the constants used

// FUNCTIONS

/*	This will draw an artificial horizon for attitude and heading
* indication for a pilot. The inputs are xi_pitch, xi_roll, and xi_yaw of the
* vehicle.  Also, the xi_altitude and xi_speed for the side bars.  In addition
* to only displaying numbers, a three character xi_message can be displayed
* as well.
*  The ranges are data are...
*	Pitch	-90 to 90 degrees
*	Roll	-180 to 180 degrees
*	Yaw		-180 to 180 degrees
*	Altitude	NO RANGE (ft)
*	Speed		NO RANGE (knots)
*	Message	3 characters
*/
void horizon(double xi_pitch, double xi_roll, double xi_yaw,
			 double xi_altitude, double xi_speed, double xi_roc,
			 double xi_mr_col, double xi_tr_col, double xi_A1,
			 double xi_B1, double xi_tr_rev, double xi_mr_rev,
			 char *xi_message);


/*	This is a useful routine to display a string message on the 
* GLUT GL window.
*/
void showMessage( GLfloat x, GLfloat y, char *message, float scale );


/*	This will draw text on the OpenGL screen.
* showMessege uses this to display the charactors.
*/
void drawText( char *message );

#endif
