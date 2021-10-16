/**
* Graphical Functions for Helicopter model drawing and GUI
**/

#include "graphics.h"

/**
* @param  {short} xi_type 0 : RC pilot view
*                         1 : fly behind view
*                         2 : upper view
* @return         helicopter model, ground, sky and shadow on screen drawing
**/
void DrawScene(short xi_type) {
    // statics (scene lighting definitions)
    static const GLfloat groundAmbient[4] = {0.02f, 0.3f, 0.1f, 1.0f},
                         local_ambient[4] = {0.7f, 0.7f, 0.7f, 1.0f},
                         ambient0[4]      = {0.0f, 0.0f, 0.0f, 1.0f},
                         diffuse0[4]      = {1.0f, 1.0f, 1.0f, 1.0f},
                         specular0[4]     = {1.0f, 0.0f, 0.0f, 1.0f},
                         position0[4]     = {2.0f, 100.5f, 1.5f, 1.0f},
                         ambient1[4]      = {0.0f, 0.0f, 0.0f, 1.0f},
                         diffuse1[4]      = {1.0f, 1.0f, 1.0f, 1.0f},
                         specular1[4]     = {1.0f, 0.0f, 0.0f, 1.0f},
                         position1[4]     = {-2.0f, 100.5f, 1.0f, 0.0f};
    // statics (squashing matrix; 3D -> 2D)
    static const float mat[16] = {1.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 1.0, 0.0,
                                  0.0, 0.0, 0.0, 1.0};

    // locals
    int e;
    float X, Y, Z;
    double camera[3], destination[3], up[3]; // view port
    
    // scene light definitions
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, local_ambient);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, 1);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 0);
    
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient0);
    glLightfv(GL_LIGHT0, GL_POSITION, position0);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse0);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular0);
    
    glEnable(GL_LIGHT1);
    glLightfv(GL_LIGHT1, GL_AMBIENT, ambient1);
    glLightfv(GL_LIGHT1, GL_POSITION, position1);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse1);
    glLightfv(GL_LIGHT1, GL_SPECULAR, specular1);
    
    // A/V position
    X =  heli.cg.NED[_NORTH];       // North = X computer frame
    Y = -heli.cg.NED[_DOWN];        // Down  = -Y computer frame
    Z =  heli.cg.NED[_EAST];        // East  = Z computer frame

    // clear view port
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    
    // view port
    if (xi_type == 0) {
        camera[_X] = 1.0;
        camera[_Y] = 6.0;
        camera[_Z] = 30.0;
        destination[_X] = X;
        destination[_Y] = Y;
        destination[_Z] = Z;
        up[_X] = 0.0;
        up[_Y] = 1.0;
        up[_Z] = 0.0;
    } else if (xi_type == 1) {
        camera[_X] = X - 10 * cos(heli.cg.THETA[_PSI]);
        camera[_Y] = Y + 1;
        camera[_Z] = Z - 10 * sin(heli.cg.THETA[_PSI]);
        destination[_X] = X;
        destination[_Y] = Y;
        destination[_Z] = Z;
        up[_X] = 0.0;
        up[_Y] = 1.0;
        up[_Z] = 0.0;
    } else if (xi_type == 2){
        camera[_X] = X;
        camera[_Y] = 50;
        camera[_Z] = Z;
        destination[_X] = X;
        destination[_Y] = Y;
        destination[_Z] = Z;
        up[_X] = 1.0;
        up[_Y] = 1.0;
        up[_Z] = 0.0;
    }
    gluLookAt(camera[_X],      camera[_Y],      camera[_Z],
              destination[_X], destination[_Y], destination[_Z],
              up[_X],          up[_Y],          up[_Z]);

    glEnable(GL_NORMALIZE);
    glDisable(GL_LIGHTING);

    // origin heli pad lines
    glLineWidth(3.0f);
    glColor3f(0.0f, 0.0f, 1.0f);
    glBegin(GL_LINES);
    glVertex3f( 0.0f, 1.0f,  0.0f);
    glVertex3f( 0.0f, 0.0f,  0.0f);
    glVertex3f( 1.0f, 1.0f,  1.0f);
    glVertex3f(-1.0f, 1.0f, -1.0f);
    glVertex3f(-1.0f, 1.0f,  1.0f);
    glVertex3f( 1.0f, 1.0f, -1.0f);
    glEnd();

    // origin heli pad circle
    DrawCircle(0.0, 0.0, 1.0, 8);
    glLineWidth(1.0f);

    //ground
    glEnable(GL_LIGHTING);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, groundAmbient);
    
    glBegin(GL_POLYGON);
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(31200.0f, -0.0f, -31200.0f);
    glVertex3f(-31200.0f, -0.0f, -31200.0f);
    glVertex3f(-31200.0f, -0.0f, 31200.0f);
    glVertex3f(31200.0f, -0.0f, 31200.0f);
    glEnd();
    
    glDisable(GL_LIGHTING);

    for(e = -1000; e <= 1000; e += 50) {
        glColor3f(0.0f, 0.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(e, 0.8, -1000.0f);
        glVertex3f(e, 0.8, 1000.0f);
        
        glVertex3f(-1000.0f, 0.8, e);
        glVertex3f(1000.0f, 0.8, e);
        glEnd();
    }
    glEnable(GL_LIGHTING);
    
    glPushMatrix();
    glTranslatef(X, Y, Z);
    glRotatef(heli.cg.THETA[_PSI]*C_RAD2DEG, 0.0f, -1.0f, 0.0);
    glRotatef(heli.cg.THETA[_THETA]*C_RAD2DEG, 0.0f, 0.0f, 1.0f);
    glRotatef(heli.cg.THETA[_PHI]*C_RAD2DEG, 1.0f, 0.0f, 0.0f);
    DrawXcellModel();
    glPopMatrix();

    glPushMatrix();
    
    // Draw the shadows
    glMultMatrixf(mat);

    // Everything now is flattened onto the ground
    glTranslatef(X, 0.01, Z);
    glRotatef(heli.cg.THETA[_PSI]   * C_RAD2DEG, 0.0f, -1.0f, 0.0);
    glRotatef(heli.cg.THETA[_THETA] * C_RAD2DEG, 0.0f,  0.0f, 1.0f);
    glRotatef(heli.cg.THETA[_PHI]   * C_RAD2DEG, 1.0f,  0.0f, 0.0f);
    DrawXcellShadowModel();
    glPopMatrix();
}


/**
* draw helicopter model
**/
void DrawXcellModel(void) {
    // statics
    static const GLfloat material1Ambient[4] = {0.4f, 0.0f, 0.0f, 1.0f},
                         material2Ambient[4] = {0.4f, 0.4f, 0.0f, 1.0f},
                         material3Ambient[4] = {0.4f, 0.4f, 0.4f, 1.0f},
                         material4Ambient[4] = {0.0f, 0.0f, 0.0f, 1.0f},
                         materialShininess[1]  ={10.0f},
                         material4Shininess[1] ={100.0f};
    static double angle;

    // locals
    float k;
    float lastx1, lasty1, lastz1, newx1, newy1, newz1;
    float lastx2, lasty2, lastz2, newx2, newy2, newz2;

    // model scaling
    glScalef(SF, SF, SF);
    
    // helicopter model start
    glEnable(GL_LIGHTING);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, material1Ambient);
    glEnable(GL_NORMALIZE);
    glBegin(GL_TRIANGLES);
    glNormal3f(0.0f, 1.0f, 1.0f);
    glVertex3f(0.0f, 0.20f, 0.10f);
    glVertex3f(0.0f, 0.1f, 0.20f);
    glVertex3f(0.60f, -0.1f, 0.0f);
    
    glNormal3f(0.0f, 0.0f, 1.0f);   
    glVertex3f(0.0f, 0.1f, 0.20f);
    glVertex3f(0.0f, -0.1f, 0.20f);
    glVertex3f(0.60f, -0.1f, 0.0f);
    
    glNormal3f(0.0f, -0.5f, 0.5f);
    glVertex3f(0.0f, -0.1f, 0.20f);
    glVertex3f(0.0f, -0.20f, 0.1f);
    glVertex3f(0.60f, -0.1f, 0.0f);
    
    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertex3f(0.0f, -0.20f, 0.1f);
    glVertex3f(0.0f, -0.20f, -0.1f);
    glVertex3f(0.60f, -0.1f, 0.0f);
    
    glNormal3f(0.0f, -0.5f, -0.5f);
    glVertex3f(0.0f, -0.20f, -0.1f);
    glVertex3f(0.0f, -0.1f, -0.20f);
    glVertex3f(0.60f, -0.1f, 0.0f);
    
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(0.0f, -0.1f, -0.20f);
    glVertex3f(0.0f, 0.1f, -0.20f);
    glVertex3f(0.60f, -0.1f, 0.0f);
    
    glNormal3f(0.0f, 0.5f, -0.5f);
    glVertex3f(0.0f, 0.1f, -0.20f);
    glVertex3f(0.0f, 0.20f, -0.1f);
    glVertex3f(0.60f, -0.1f, 0.0f);
    
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.20f, -0.1f);
    glVertex3f(0.0f, 0.20f, 0.1f);
    glVertex3f(0.60f, -0.1f, 0.0f);
    
    glEnd();
    
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, material4Ambient);
    glMaterialfv(GL_FRONT, GL_SHININESS, material4Shininess);
    glBegin(GL_POLYGON);
    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.20f, 0.1f);
    glVertex3f(0.0f, 0.1f, 0.20f);
    glVertex3f(0.0f, -0.1f, 0.20f);
    glVertex3f(0.0f, -0.20f, 0.1f);
    glVertex3f(0.0f, -0.20f, -0.1f);
    glVertex3f(0.0f, -0.1f, -0.20f);
    glVertex3f(0.0f, 0.1f, -0.20f);
    glVertex3f(0.0f, 0.20f, -0.1f);
    glEnd();
    
    //tail boom
    //glColor3f(0.4f, 0.4f, 0.0f);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, material2Ambient);
    glMaterialfv(GL_FRONT, GL_SHININESS, materialShininess);
    glBegin(GL_POLYGON);
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.02f, 0.02f);
    glVertex3f(-1.4f, 0.02f, 0.02f);
    glVertex3f(-1.4f, -0.02f, 0.02f);
    glVertex3f(0.0f, -0.02f, 0.02f);
    
    glEnd();
    glBegin(GL_POLYGON);
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(0.0f, 0.02f, -0.02f);
    glVertex3f(-1.4f, 0.02f, -0.02f);
    glVertex3f(-1.4f, -0.02f, -0.02f);
    glVertex3f(0.0f, -0.02f, -0.02f);
    
    glEnd();
    glBegin(GL_POLYGON);
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.02f, 0.02f);
    glVertex3f(-1.4f, 0.02f, 0.02f);
    glVertex3f(-1.4f, 0.02f, -0.02f);
    glVertex3f(0.0f, 0.02f, -0.02f);
    
    glEnd();
    glBegin(GL_POLYGON);
    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertex3f(0.0f, -0.02f, 0.02f);
    glVertex3f(-1.4f, -0.02f, 0.02f);
    glVertex3f(-1.4f, -0.02f, -0.02f);
    glVertex3f(0.0f, -0.02f, -0.02f);
    glEnd();
    

    // vertical fin
    glBegin(GL_POLYGON);
    glVertex3f(-0.6f,  0.0f,  0.0f);
    glVertex3f(-0.6f, -0.15f, 0.0f);
    glVertex3f(-0.8f, -0.15f, 0.0f);
    glVertex3f(-0.8f,  0.0f,  0.0f);
    glEnd();

    // horizontal fin
    glBegin(GL_POLYGON);
    glVertex3f(-0.6f, 0.0f,  0.15f);
    glVertex3f(-0.6f, 0.0f, -0.15f);
    glVertex3f(-0.8f, 0.0f, -0.15f);
    glVertex3f(-0.8f, 0.0f,  0.15f);
    glEnd();

    // tail rotor
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, material3Ambient);
    lastx1=0.12-1.4;
    lasty1=0.0;
    lastz1=0.02;
    //glColor3f(0.4f, 0.4f, 0.4f);

    for(k = -C_PI_DIV_12; k <= C_TWOPI; k += C_PI_DIV_12) {
        newx1 = -1.4 + 0.12 * cos(k);
        newy1 = 0.12 * sin(k);
        newz1 = 0.02;

        glBegin(GL_TRIANGLES);
        glNormal3f(0.0f, 0.0f, 1.0f);
        glVertex3f(-1.4f, 0.0f, 0.02f);
        glVertex3f(lastx1, lasty1, lastz1);
        glVertex3f(newx1, newy1, newz1);
        glEnd();

        lastx1=newx1;
        lasty1=newy1;
        lastz1=newz1;
    }

    // main rotor
    glPushMatrix();

    // YAW
    glRotatef(heli.cg.THETA[_PHI]*C_RAD2DEG, 0.0f, -1.0f, 0.0f);
    // PITCH
    glRotatef(heli.m.a1*C_RAD2DEG, 0.0f, 0.0f, 1.0f);
    // ROLL
    glRotatef(heli.m.b1*C_RAD2DEG, 1.0f, 0.0f, 0.0f);   

    lastx2=0.0;
    lastx1=0.0;
    lasty1=0.34;
    lasty2=0.34;
    lastz1=0.90;
    lastz2=1.0;
    //glColor4f(0.4f, 0.4f, 0.4f, 0.5f);
    for(k = -C_PI_DIV_12; k <= 2*C_PI; k += C_PI_DIV_12) {
        newx1=1.0*sin(k);   // 1.0 = outer radius
        newx2=0.90*sin(k);  // 0.9 = inner radius
        newy1=0.34;
        newy2=0.34;
        newz1=1.0*cos(k);
        newz2=0.90*cos(k);
        
        glNormal3f(0.0f, 1.0f, 0.0f);
        
        lastx1=newx1;
        lasty1=newy1;
        lastz1=newz1;
        lastx2=newx2;
        lasty2=newy2;
        lastz2=newz2;
    }

    // draw rotating blades
    lastx1  = 1.0 * sin(angle);
    lastx2  = 1.0 * sin(angle + 0.10);
    lastz1  = 1.0 * cos(angle);
    lastz2  = 1.0 * cos(angle + 0.10);
    angle += 0.5;
    
    // Blade 1
    glBegin(GL_TRIANGLES);
    glVertex3f(lastx1, newy1, lastz1);
    glVertex3f(lastx2, newy1, lastz2);
    glVertex3f(0,  newy1, 0);
    glEnd();
    
    // Blade 2
    glBegin(GL_TRIANGLES);
    glVertex3f(-lastx1, newy1, -lastz1);
    glVertex3f(-lastx2, newy1, -lastz2);
    glVertex3f(0,  newy1, 0);
    glEnd();

    glPopMatrix();

    //main mast
    glColor3f(0.4f, 0.7f, 0.2f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.34f, 0.0f);
    glEnd();
    
    // skids
    glColor3f(0.0f, 0.0f, 1.0f);
    glBegin(GL_LINES);
    glVertex3f(0.2f, -0.2f, 0.1f);
    glVertex3f(0.2f, -0.3f, 0.2f);
    
    glVertex3f(0.2f, -0.2f, -0.1f);
    glVertex3f(0.2f, -0.3f, -0.2f);
    
    glVertex3f(0.0f, -0.2f, 0.1f);
    glVertex3f(0.0f, -0.3f, 0.2f);
    
    glVertex3f(0.0f, -0.2f, -0.1f);
    glVertex3f(0.0f, -0.3f, -0.2f);
    
    glVertex3f(-0.1f, -0.3f, 0.2f);
    glVertex3f(0.4f, -0.3f, 0.2f);
    
    glVertex3f(-0.1f, -0.3f, -0.2f);
    glVertex3f(0.4f, -0.3f, -0.2f);
    glEnd();
    glEnable(GL_LIGHTING);
}


/**
* draw helicopter shadow model
**/
void DrawXcellShadowModel(void) {
    // statics
    static const GLfloat material1Ambient[4] = {0.0f, 0.0f, 0.0f, 1.0f},
                         material2Ambient[4] = {0.0f, 0.0f, 0.0f, 1.0f},
                         material3Ambient[4] = {0.0f, 0.0f, 0.0f, 1.0f},
                         material4Ambient[4] = {0.0f, 0.0f, 0.0f, 1.0f},
                         materialShininess[1]  = {10.0f},
                         material4Shininess[1] = {100.0f};
    static double angle;

    // locals
    float k;
    float lastx1, lasty1, lastz1, newx1, newy1, newz1;
    float lastx2, lasty2, lastz2, newx2, newy2, newz2;

    // shadow scaling
    glScalef(SF, SF, SF);

    // start of helicopter model
    glEnable(GL_LIGHTING);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, material1Ambient);
    glEnable(GL_NORMALIZE);
    glBegin(GL_TRIANGLES);
    glNormal3f(0.0f, 1.0f, 1.0f);
    glVertex3f(0.0f, 0.20f, 0.10f);
    glVertex3f(0.0f, 0.1f, 0.20f);
    glVertex3f(0.60f, -0.1f, 0.0f);
    
    glNormal3f(0.0f, 0.0f, 1.0f);   
    glVertex3f(0.0f, 0.1f, 0.20f);
    glVertex3f(0.0f, -0.1f, 0.20f);
    glVertex3f(0.60f, -0.1f, 0.0f);
    
    glNormal3f(0.0f, -0.5f, 0.5f);
    glVertex3f(0.0f, -0.1f, 0.20f);
    glVertex3f(0.0f, -0.20f, 0.1f);
    glVertex3f(0.60f, -0.1f, 0.0f);
    
    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertex3f(0.0f, -0.20f, 0.1f);
    glVertex3f(0.0f, -0.20f, -0.1f);
    glVertex3f(0.60f, -0.1f, 0.0f);
    
    glNormal3f(0.0f, -0.5f, -0.5f);
    glVertex3f(0.0f, -0.20f, -0.1f);
    glVertex3f(0.0f, -0.1f, -0.20f);
    glVertex3f(0.60f, -0.1f, 0.0f);
    
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(0.0f, -0.1f, -0.20f);
    glVertex3f(0.0f, 0.1f, -0.20f);
    glVertex3f(0.60f, -0.1f, 0.0f);
    
    glNormal3f(0.0f, 0.5f, -0.5f);
    glVertex3f(0.0f, 0.1f, -0.20f);
    glVertex3f(0.0f, 0.20f, -0.1f);
    glVertex3f(0.60f, -0.1f, 0.0f);
    
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.20f, -0.1f);
    glVertex3f(0.0f, 0.20f, 0.1f);
    glVertex3f(0.60f, -0.1f, 0.0f);
    
    glEnd();
    
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, material4Ambient);
    glMaterialfv(GL_FRONT, GL_SHININESS, material4Shininess);
    glBegin(GL_POLYGON);
    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.20f, 0.1f);
    glVertex3f(0.0f, 0.1f, 0.20f);
    glVertex3f(0.0f, -0.1f, 0.20f);
    glVertex3f(0.0f, -0.20f, 0.1f);
    glVertex3f(0.0f, -0.20f, -0.1f);
    glVertex3f(0.0f, -0.1f, -0.20f);
    glVertex3f(0.0f, 0.1f, -0.20f);
    glVertex3f(0.0f, 0.20f, -0.1f);
    glEnd();
    
    // tail boom
    //glColor3f(0.4f, 0.4f, 0.0f);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, material2Ambient);
    glMaterialfv(GL_FRONT, GL_SHININESS, materialShininess);
    glBegin(GL_POLYGON);
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.02f, 0.02f);
    glVertex3f(-1.4f, 0.02f, 0.02f);
    glVertex3f(-1.4f, -0.02f, 0.02f);
    glVertex3f(0.0f, -0.02f, 0.02f);
    
    glEnd();
    glBegin(GL_POLYGON);
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(0.0f, 0.02f, -0.02f);
    glVertex3f(-1.4f, 0.02f, -0.02f);
    glVertex3f(-1.4f, -0.02f, -0.02f);
    glVertex3f(0.0f, -0.02f, -0.02f);
    
    glEnd();
    glBegin(GL_POLYGON);
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.02f, 0.02f);
    glVertex3f(-1.4f, 0.02f, 0.02f);
    glVertex3f(-1.4f, 0.02f, -0.02f);
    glVertex3f(0.0f, 0.02f, -0.02f);
    
    glEnd();
    glBegin(GL_POLYGON);
    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertex3f(0.0f, -0.02f, 0.02f);
    glVertex3f(-1.4f, -0.02f, 0.02f);
    glVertex3f(-1.4f, -0.02f, -0.02f);
    glVertex3f(0.0f, -0.02f, -0.02f);
    
    glEnd();
    
    // tail rotor
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, material3Ambient);
    lastx1=0.12-1.4;
    lasty1=0.0;
    lastz1=0.02;
    //glColor3f(0.4f, 0.4f, 0.4f);

    for(k = -C_PI_DIV_12; k <= C_TWOPI; k += C_PI_DIV_12) {
        newx1=-1.4 + 0.12*cos(k);
        newy1=0.12 * sin(k);
        newz1=0.02;
        glBegin(GL_TRIANGLES);
        glNormal3f(0.0f, 0.0f, 1.0f);
        glVertex3f(-1.4f, 0.0f, 0.02f);
        glVertex3f(lastx1, lasty1, lastz1);
        glVertex3f(newx1, newy1, newz1);
        glEnd();
        lastx1=newx1;
        lasty1=newy1;
        lastz1=newz1;
    }

    // main rotor
    lastx2=0.0;
    lastx1=0.0;
    lasty1=0.34;
    lasty2=0.34;
    lastz1=0.90;
    lastz2=1.0;
    //glColor4f(0.4f, 0.4f, 0.4f, 0.5f);

    for(k = -C_PI_DIV_12; k <= C_TWOPI; k += C_PI_DIV_12) {
        newx1=1.0*sin(k);
        newx2=0.90*sin(k);
        newy1=0.34;
        newy2=0.34;
        newz1=1.0*cos(k);
        newz2=0.90*cos(k);
        
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, material3Ambient);
        glBegin(GL_POLYGON);
        glNormal3f(0.0f, 1.0f, 0.0f);
        glEnd();
        
        lastx1=newx1;
        lasty1=newy1;
        lastz1=newz1;
        lastx2=newx2;
        lasty2=newy2;
        lastz2=newz2;
    }   
    
    // draw rotating blades
    lastx1  = 1.0 * sin( angle );
    lastx2  = 1.0 * sin( angle + 0.10 );
    lastz1  = 1.0 * cos( angle );
    lastz2  = 1.0 * cos( angle + 0.10 );
    angle += 0.5;
    
    // Blade 1
    glBegin( GL_TRIANGLES );
    glVertex3f( lastx1, newy1, lastz1 );
    glVertex3f( lastx2, newy1, lastz2 );
    glVertex3f( 0,  newy2, 0  );
    glEnd();
    
    // Blade 2
    glBegin( GL_TRIANGLES );
    glVertex3f( -lastx1, newy1, -lastz1 );
    glVertex3f( -lastx2, newy1, -lastz2 );
    glVertex3f( 0,  newy2, 0  );
    glEnd();

    // main mast
    glColor3f(0.4f, 0.7f, 0.2f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.34f, 0.0f);
    glEnd();
    
    // skids
    glColor3f(0.0f, 0.0f, 1.0f);
    glBegin(GL_LINES);
    glVertex3f(0.2f, -0.2f, 0.1f);
    glVertex3f(0.2f, -0.3f, 0.2f);
    
    glVertex3f(0.2f, -0.2f, -0.1f);
    glVertex3f(0.2f, -0.3f, -0.2f);
    
    glVertex3f(0.0f, -0.2f, 0.1f);
    glVertex3f(0.0f, -0.3f, 0.2f);
    
    glVertex3f(0.0f, -0.2f, -0.1f);
    glVertex3f(0.0f, -0.3f, -0.2f);
    
    glVertex3f(-0.1f, -0.3f, 0.2f);
    glVertex3f(0.4f, -0.3f, 0.2f);
    
    glVertex3f(-0.1f, -0.3f, -0.2f);
    glVertex3f(0.4f, -0.3f, -0.2f);
    glEnd();
    glEnable(GL_LIGHTING);
}

/**
* draw GUI
**/
void drawGUI(void) {
    // housekeeping
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
}


/**
* draw a circle
*
* @param  {float} X center
* @param  {float} Y center
* @param  {float} radius
* @param  {int}   number of circumference segments
**/
void DrawCircle(float cx, float cy, float r, int num_segments) { 
    // locals
    int i;
    float theta = C_TWOPI / (float)num_segments,
          c = cos(theta), s = sin(theta),
          x = r, y = 0; 
    
    // drawing
    glBegin(GL_LINE_LOOP); 
    for(i = 0; i < num_segments; i++) { 
        //output vertex 
        glVertex3f(x + cx, 1.0, y + cy);
        
        //apply the rotation matrix
        float t = x;
        x = c * x - s * y;
        y = s * t + c * y;
    } 
    glEnd(); 
}
