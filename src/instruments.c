/**
* GLUT library based instrument drawing functions
**/

#include "instruments.h"

/**
* pilot artificial horizon drawing function.
*
* @param {double} pitch angle [-90, 90]
* @param {double} roll angle [-180, 180]
* @param {double} yaw angle [-180, 180]
* @param {double} density altitude [ft]
* @param {double} geometric speed [knots]
* @param {double} ROC [ft/min]
* @param {double} main rotor collective (+ right yaw rad)
* @param {double} tail rotor collective (+ climb rad)
* @param {double} A1 swash plate angle = roll cyclic pitch (rad + right roll)
* @param {double} B1 swash plate angle = pitch cyclic pitch (rad + nose down)
* @param {double} tail rotor RPM (rpm)
* @param {double} main rotor RPM (rpm)
* @param {char*}  3 characters message to be displayed
*
* @todo  add the following data to HUD: MR RPM, ROC, ACTUATOR ANGLES, 
**/
void horizon(double xi_pitch, double xi_roll, double xi_yaw,
			 double xi_altitude, double xi_speed, double xi_roc,
			 double xi_mr_col, double xi_tr_col, double xi_A1,
			 double xi_B1, double xi_tr_rev, double xi_mr_rev,
			 char *xi_message) {
	// static
	static const char headlabels[37][4] = {"S\0", "19\0", "20\0", "21\0", "22\0", "23\0", "24\0",
										   "25\0", "26\0", "W\0", "28\0", "29\0", "30\0", "31\0",
										   "32\0", "33\0", "34\0", "35\0", "N\0", "1\0", "2\0",
										   "3\0", "4\0", "5\0", "6\0", "7\0", "8\0", "E\0", "10\0",
										   "11\0", "12\0", "13\0", "14\0", "15\0", "16\0", "17\0",
									       "S\0"};
	// locals
	double temp;
	int n;
	char buffer[20];
	char *txt;

	// housekeeping
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	// output scree line
	glColor3f(1.0f, 1.0f, 0.0f);
	glLineWidth(1.0f);

	// main/tail RPM
	sprintf(buffer, "RPM main:%2d\0", (int)(xi_mr_rev));
	glPushMatrix();
	glTranslatef(-0.2f, -0.02f, -0.8f);
	showMessage(0.0f, 0.0f, buffer, 1.2);
	glPopMatrix();
	sprintf(buffer, "RPM tail:%2d\0", (int)(xi_tr_rev));
	glPushMatrix();
	glTranslatef(-0.2f, -0.04f, -0.8f);
	showMessage(0.0f, 0.0f, buffer, 1.2);
	glPopMatrix();

	// main/tail collective
	sprintf(buffer, "Col main:%2d\0", (int)(xi_mr_col));
	glPushMatrix();
	glTranslatef(-0.2f, -0.06f, -0.8f);
	showMessage(0.0f, 0.0f, buffer, 1.2);
	glPopMatrix();
	sprintf(buffer, "Col tail:%2d\0", (int)(xi_tr_col));
	glPushMatrix();
	glTranslatef(-0.2f, -0.08f, -0.8f);
	showMessage(0.0f, 0.0f, buffer, 1.2);
	glPopMatrix();

	// swash plate collective
	sprintf(buffer, "Cyc pitch:%2d\0", (int)(xi_A1));
	glPushMatrix();
	glTranslatef(-0.2f, -0.1f, -0.8f);
	showMessage(0.0f, 0.0f, buffer, 1.2);
	glPopMatrix();
	sprintf(buffer, "Cyc roll:%2d\0", (int)(xi_B1));
	glPushMatrix();
	glTranslatef(-0.2f, -0.12f, -0.8f);
	showMessage(0.0f, 0.0f, buffer, 1.2);
	glPopMatrix();

	// xi_altitude readout
	sprintf(buffer, "Alt:%3d ft\0", (int)(xi_altitude));
	glPushMatrix();
	glTranslatef(-0.2f, -0.16f, -0.8f);
	showMessage(0.0f, 0.0f, buffer, 1.2);
	glPopMatrix();


	// xi_speed readout
	sprintf(buffer, "Vel:%3d kts\0", (int)(xi_speed));
	glPushMatrix();
	glTranslatef(-0.2f, -0.18f, -0.8f);
	showMessage(0.0f, 0.0f, buffer, 1.2);
	glPopMatrix();

	// xi_yaw readout
	sprintf(buffer, "Hdng:%3d\0", (int)(xi_yaw));
	glPushMatrix();
	glTranslatef(-0.2f, -0.2f, -0.8f);
	showMessage(0.0f, 0.0f, buffer, 1.2);
	glPopMatrix();
	
	// WOW readout
	glLineWidth(3.0f);
	if (xi_altitude < 1) {
		glColor3f(1.0f, 0.0f, 0.0f);
		sprintf(buffer, "ON GROUND\0");
		glPushMatrix();
		glTranslatef(-0.03f, -0.2f, -0.8f);
		showMessage(0.0f, 0.0f, buffer, 1.2);
	} else {
		glColor3f(0.0f, 1.0f, 0.0f);
		sprintf(buffer, "AIRBORN\0");
		glPushMatrix();
		glTranslatef(-0.03f, -0.2f, -0.8f);
		showMessage(0.0f, 0.0f, buffer, 1.2);
	}
	glPopMatrix();

	// return line width
	glLineWidth(1.0f);

	// message readout
	//glPushMatrix();
	//glTranslatef(0.0f, 0.0f, -0.8f);
	//showMessage(0.145f, -0.17f, message, 1);
	//glPopMatrix();
	
	glPopMatrix();

	// roll tick marks
	for(n = -30; n <= 30; n += 15) {
		glPushMatrix();
		glRotatef(n, 0.0f, 0.0f, 1.0f);
		glColor3f(1.0f, 1.0f, 1.0f);
		glBegin( GL_LINES );
			glVertex3f(0.0f, 0.24f, -0.9f);
			glVertex3f(0.0f, 0.23f, -0.9f);
		glEnd();
		glPopMatrix();
	}
	glPushMatrix();
	glRotatef(xi_roll, 0.0f, 0.0f, 1.0f);
	glColor3f(0.85f, 0.5f, 0.1f);
	glBegin( GL_TRIANGLES );
		glVertex3f(0.0f, 0.23f, -0.9f);
		glVertex3f(0.01f, 0.21f, -0.9f);
		glVertex3f(-0.01f, 0.21f, -0.9f);
	glEnd();
	glPopMatrix();


	// center mark
	glPushMatrix();
	glColor3f(1.0f, 1.0f, 1.0f);
	glBegin( GL_LINES );
		// right half
		glVertex3f(0.0f, 0.0f, -0.9f);
		glVertex3f(0.015f, -0.02f, -0.9f);

		glVertex3f(0.015f, -0.02f, -0.9f);
		glVertex3f(0.03f, 0.0f, -0.9f);
		
		glVertex3f(0.03f, 0.0f, -0.9f);
		glVertex3f(0.06f, 0.0f, -0.9f);

		// left half
		glVertex3f(0.0f, 0.0f, -0.9f);
		glVertex3f(-0.015f, -0.02f, -0.9f);

		glVertex3f(-0.015f, -0.02f, -0.9f);
		glVertex3f(-0.03f, 0.0f, -0.9f);
		
		glVertex3f(-0.03f, 0.0f, -0.9f);
		glVertex3f(-0.06f, 0.0f, -0.9f);
	glEnd();
	
	glPopMatrix();

	glRotatef(xi_roll, 0.0f, 0.0f, 1.0f);
	glTranslatef(-xi_yaw*C_DEG2RAD, 0.0f, 0.0f);

	// horizon
	glColor3f(1.0f, 1.0f, 1.0f);
	glBegin( GL_LINES );
		glVertex3f(-(180.0+15)*C_DEG2RAD, 0.0f, -0.9f);
		glVertex3f((180.0+15)*C_DEG2RAD, 0.0f, -0.9f);
	glEnd();

	// yaw tick lines
	for(n = 0; n < 37; ++n) {
		glBegin( GL_LINES );
			glVertex3f( (double)(n*10 - 180)*C_DEG2RAD, 0.015f, -0.9f);
			glVertex3f( (double)(n*10 - 180)*C_DEG2RAD, 0.0f, -0.9f);
		glEnd();
		glPushMatrix();
		glTranslatef(0.0f, 0.0f, -0.9f);
		txt = (char *)&headlabels[n][0];
		showMessage((double)(n*10 - 180)*C_DEG2RAD-0.01f, 0.02, txt, 1.2);
		glPopMatrix();
	}

	// Extra tick mark past S (going W) for overview
	glBegin( GL_LINES );
		glVertex3f( 190.0*C_DEG2RAD, 0.02f, -0.9f);
		glVertex3f( 190.0*C_DEG2RAD, 0.0f, -0.9f);
	glEnd();
	glPushMatrix();
	glTranslatef(0.0f, 0.0f, -0.9f);
	showMessage( 190.0*C_DEG2RAD-0.015, 0.02, "19\0", 1.0);
	glPopMatrix();

	// Extra tick mark past S (going E) for overview
	glBegin( GL_LINES );
		glVertex3f( -190.0*C_DEG2RAD, 0.02f, -0.9f);
		glVertex3f( -190.0*C_DEG2RAD, 0.0f, -0.9f);
	glEnd();
	glPushMatrix();
	glTranslatef(0.0f, 0.0f, -0.9f);
	showMessage( -190.0*C_DEG2RAD-0.015, 0.02, "17\0", 1.0);
	glPopMatrix();

	glPushMatrix();
	glLoadIdentity();
	glRotatef(xi_roll, 0.0f, 0.0f, 1.0f);
	glTranslatef(0.0f, -xi_pitch*C_DEG2RAD, 0.0f);

	// colored part of display
	glColor3f(0.0f, 0.0f, 1.0f);
	glBegin( GL_QUADS );
		glVertex3f(-(180.0+15)*C_DEG2RAD, (90.0+15.0)*C_DEG2RAD, -1.0f);
		glVertex3f((180.0+15)*C_DEG2RAD, (90.0+15.0)*C_DEG2RAD, -1.0f);
		glVertex3f((180.0+15)*C_DEG2RAD, 0.0f, -1.0f);
		glVertex3f(-(180.0+15)*C_DEG2RAD, 0.0f, -1.0f);
	glEnd();
	
	// bottom of display
	glColor3f(0.5f, 0.2f, 0.1f);
	glBegin( GL_QUADS );
		glVertex3f(-(180.0+15)*C_DEG2RAD, -(90.0+15.0)*C_DEG2RAD, -1.0f);
		glVertex3f((180.0+15)*C_DEG2RAD, -(90.0+15.0)*C_DEG2RAD, -1.0f);
		glVertex3f((180.0+15)*C_DEG2RAD, 0.0f, -1.0f);
		glVertex3f(-(180.0+15)*C_DEG2RAD, 0.0f, -1.0f);
	glEnd();

	// pitch bars
	for(n = 0; n < 9; ++n) {
		temp = (double)(n * 10 + 10) * C_DEG2RAD;
		glColor3f(1.0f, 1.0f, 1.0f);

		// positive xi_pitch lines
		glBegin( GL_LINES );
			glVertex3f(-0.1f, temp-0.01, -1.0f);
			glVertex3f(-0.1f, temp, -1.0f);

			glVertex3f(-0.1f, temp, -1.0f);
			glVertex3f(-0.03f, temp, -1.0f);

			glVertex3f(0.1f, temp-0.01, -1.0f);
			glVertex3f(0.1f, temp, -1.0f);

			glVertex3f(0.1f, temp, -1.0f);
			glVertex3f(0.03f, temp, -1.0f);
		glEnd();

		sprintf(buffer, "%d\0", n*10+10);
		glPushMatrix();
		glTranslatef(0.0f, 0.0f, -1.0f);
		showMessage(0.11f, temp-0.007, buffer, 1.0);
		showMessage(-0.13f, temp-0.007, buffer, 1.0);
		glPopMatrix();

		// negative xi_pitch lines
		glBegin( GL_LINES );
			glVertex3f(-0.1f, -temp+0.01, -1.0f);
			glVertex3f(-0.1f, -temp, -1.0f);

			glVertex3f(-0.1f, -temp, -1.0f);
			glVertex3f(-0.03f, -temp, -1.0f);

			glVertex3f(0.1f, -temp+0.01, -1.0f);
			glVertex3f(0.1f, -temp, -1.0f);

			glVertex3f(0.1f, -temp, -1.0f);
			glVertex3f(0.03f, -temp, -1.0f);
		glEnd();

		sprintf(buffer, "%d\0", -(n*10+10));
		glPushMatrix();
		glTranslatef(0.0f, 0.0f, -1.0f);
		showMessage(0.11f, -temp, buffer, 1.0);
		showMessage(-0.14f, -temp, buffer, 1.0);
		glPopMatrix();
	}

	// +/- 5 degree tick marks
	glBegin( GL_LINES );
		glVertex3f(-0.05f, 5.0*C_DEG2RAD, -1.0f);
		glVertex3f(0.05f, 5.0*C_DEG2RAD, -1.0f);
	glEnd();
	glBegin( GL_LINES );
		glVertex3f(-0.05f, -5.0*C_DEG2RAD, -1.0f);
		glVertex3f(0.05f, -5.0*C_DEG2RAD, -1.0f);
	glEnd();
	
	glPopMatrix();
}

/**
* draw characters in stroke mode
*
* @param {char*} characters to be displayed
**/
void drawText(char *xi_message) {
  while (*xi_message) {
    glutStrokeCharacter( GLUT_STROKE_MONO_ROMAN, *xi_message );
    xi_message++;
  }
}

/**
* draw a generic string
*
* @param {GLfloat} X coordinate of message
* @param {GLfloat} y coordinate of message
* @param {char*}   characters to be displayed
* @param {float}   characters scale
**/
void showMessage(GLfloat xi_x, GLfloat xi_y, char *xi_message, float xi_scale) {
  glPushMatrix();
  glTranslatef( xi_x, xi_y, 0.0);
  glScalef(0.0001, 0.0001, 0.0001);
  glScalef(xi_scale, xi_scale, xi_scale);
  drawText(xi_message);
  glPopMatrix();
}
