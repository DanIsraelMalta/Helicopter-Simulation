
#ifndef graphics_h
#define graphics_h

#include "glos.h" // MS specific stuff

#include <stdio.h>
#include <GL/gl.h> // system OpenGL includes
#include <GL/glu.h>
//#include <GL/glaux.h>
#include <math.h>
#include <GL/glut.h>
#include <mmsystem.h>

#include "matlib.h"
#include "model.h"

#define SF 2.25	// helicopter model scale factor

void DrawXcellModel(void);
void DrawXcellShadowModel(void);
void DrawScene(short xi_type);
void drawGUI(void);
void DrawCircle(float cx, float cy, float r, int num_segments);

#endif
