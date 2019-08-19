/**
* Helicopter Simulation Main Module
*
* TODO:
* 1) Add GUI window which allows to change helicopter parameters in real time
*    and control simulation (initialize, play, stop)
* 2) Make helicopter graphical model appear with proportion to its defined geometrical characteristics.
*    Also, make tail rotor and fly-bar rotate
* 3) align numbers arranged in U, svIn & svX; give them a #define symbol
* 4) Add strip chart of chosen parameters
* 5) Add/Correct vertical/horizontal plate forces/moments. It seems wrong at current moment.
*    notice that current calculation doesn't include vertical fin geometry correct.
* 6) Add a cyclic/collective and RPM window for helicopter control.
* 7) Add general data other the controllers (such as power, thrust,...) to any chosen window.
* 8) Add joystick support
*
* Dan I. Malta
**/

#include <stdlib.h>
#include <stdio.h>
#include "helisim.h"

/**
* globals
**/
double windows_dt = 0.05;   // time step for graphical output
unsigned int simWindow;     // handle for simulation 3D output window (GLUT handler)
unsigned int hudWindow;     // handle for simulation HUD output window (GLUT handler)
unsigned int guiWindow;     // handle for simulation GUI window (GLUT handler)
static int view = 0;        // 3D view (RC, pilot) handler
static int SimPlay = 1;     // simulation state (play, stop) handler

/**
* prototype
**/
static void ResizeGuiWindow(GLsizei w, GLsizei h);
static void ResizeSimWindow(GLsizei w, GLsizei h);
static void ResizeHudWindow(GLsizei w, GLsizei h);
static void Animate(void);
static void FlightInstruments(void);
void SimWindowInit(void);
void openHUD(void);
void display(void);
void Dynamics(void);
void MouseButtonSimWin(int button, int state, int x, int y);
void MouseButtonHUD(int xi_button, int xi_state, int xi_x, int xi_y);

/**
* function responsible for helicopter animation
**/
static void Animate(void) {
    DrawScene(view);
    display();
}

/**
* GUI graphical function caller
**/
static void callGUI(void) {
    drawGUI();
    display();
}

/**
* HUD graphical function caller
**/
static void FlightInstruments(void) {
    // locals
    double speed = sqrt(M_SQR(heli.cg.V[0]) + M_SQR(heli.cg.V[1]) + M_SQR(heli.cg.V[2])) * C_FPS_TO_KNOT;
    char txt[4];

    // initialize text message
    sprintf(txt, "DAN\0");

    // call HUD graphical function
    horizon(heli.cg.THETA[1]*C_RAD2DEG, heli.cg.THETA[0]*C_RAD2DEG, heli.cg.THETA[2]*C_RAD2DEG, -heli.cg.NED[2],
            speed, -heli.sixdofX.Ve[2], heli.c.mr_col*C_RAD2DEG, heli.c.tr_col*C_RAD2DEG,
            heli.c.A1*C_RAD2DEG, heli.c.B1*C_RAD2DEG, heli.c.tr_rev, heli.c.mr_rev, txt);

    // call display()
    display();
}


/**
* a function which allow the resize of 3D window while perspective is kept
*
* @param  {GLsizei} width
* @param  {GLsizei} height
**/
static void ResizeGuiWindow(GLsizei xi_width, GLsizei xi_height) {
    // housekeeping
    xi_height = (xi_height == 0) ? 1 : xi_height;
    xi_width  = (xi_width == 0)  ? 1 : xi_width;

    // view port
    glViewport( 0, 0, xi_width, xi_height );
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    
    // perspective
    gluPerspective( 30.0, (GLfloat)xi_width/(GLfloat)xi_height, 0.1f, 2500.0f );
    
    // select the Model-view matrix
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
}

/**
* a function which allow the resize of 3D window while perspective is kept
*
* @param  {GLsizei} width
* @param  {GLsizei} height
**/
static void ResizeSimWindow(GLsizei xi_width, GLsizei xi_height) {
    // housekeeping
    xi_height = (xi_height == 0) ? 1 : xi_height;
    xi_width = (xi_width == 0) ? 1 : xi_width;

    // view port
    glViewport( 0, 0, xi_width, xi_height );
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    
    // perspective
    gluPerspective( 30.0, (GLfloat)xi_width/(GLfloat)xi_height, 0.1f, 2500.0f );
    
    // select the Model-view matrix
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
}

/**
* a function which allow the resize of HUD window while perspective is kept
*
* @param  {GLsizei} width
* @param  {GLsizei} height
**/
static void ResizeHudWindow(GLsizei xi_width, GLsizei xi_height) {
    // housekeeping
    xi_height = (xi_height == 0) ? 1 : xi_height;
    xi_width  = (xi_width == 0)  ? 1 : xi_width;

    // view port
    glViewport( 0, 0, xi_width, xi_height );
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();

    // perspective
    gluPerspective( 30.0, (GLfloat)xi_width/(GLfloat)xi_height, 0.1f, 2500.0f );

    // select the Model-view matrix
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
}

/**
* HUD window
**/
void openHUD(void) {
    // create HUD window
    hudWindow = glutCreateWindow("HUD (Malta simulation)");
    glutSetWindow(hudWindow);
    glutPositionWindow(630, 30);
    glutReshapeWindow(600, 600);
    glShadeModel(GL_FLAT);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClearDepth(1.0f);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);

    // display FlightInstruments() output on window 
    glutDisplayFunc(FlightInstruments); 
    glutReshapeFunc(ResizeHudWindow);

    // allow button press to mitigate simulation status
    glutMouseFunc(MouseButtonHUD);
}

/**
* 3D simulation window
**/
void SimWindowInit(void) {
    // create simulation window
    simWindow = glutCreateWindow("3D (Malta simulation)");
    glutSetWindow(simWindow);
    glutPositionWindow(10, 30);
    glutReshapeWindow(600, 600);
    glShadeModel(GL_SMOOTH);
    glClearDepth(1.0f);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.49, 0.62, 0.9, 0.0);

    // display Animate() function output on windows
    glutDisplayFunc(Animate); 
    glutReshapeFunc(ResizeSimWindow);

    // allow button press to mitigate camera position
    glutMouseFunc(MouseButtonSimWin);

    // call animation of HUD window
    openHUD();
}

/**
* graphic simulation button click to transition 3D simulation camera location
*
* @param  {int} type of mouse button pressed
* @param  {int} state of pressed button
* @param  {int} X location of pressed button
* @param  {int} Y location of pressed button
* @return {int} {0 = RC pilot view, 1 = fly behind view}
**/
void MouseButtonSimWin(int xi_button, int xi_state, int xi_x, int xi_y) {
    // right button
    if (xi_button == GLUT_RIGHT_BUTTON) {
        view = 0;
    } else {
        view = 1;
    }
}

/**
* HUD simulation button click to play/stop simulation
*
* @param  {int} type of mouse button pressed
* @param  {int} state of pressed button
* @param  {int} X location of pressed button
* @param  {int} Y location of pressed button
* @return {int} {0 = simulation stop, 1 = simulation play}
**/
void MouseButtonHUD(int xi_button, int xi_state, int xi_x, int xi_y) {
    // right button
    if (xi_button == GLUT_RIGHT_BUTTON) {
        SimPlay = 0;
    } else {
        SimPlay = 1;
    }
}

/**
* graphical layout display
**/
void display(void) {
    glutPostRedisplay();
    glutSwapBuffers();
}

//  This is the function to propagate the model and the other stuff

/**
* main function which propagate the model and call flight code
**/
void Dynamics(void) {
    // locals
    int n;          // counter
    double U[4];    // {main rotor collective, A1 (roll), B1 (pitch), tail rotor collective}

    // simulation prorogation
    for(n = 0; n < (int)(windows_dt / model_dt); ++n) {
        // actuators
        U[0] =  10.5 /*10.0, 9.5*/ * C_DEG2RAD;
        U[1] =  0.0  * C_DEG2RAD;
        U[2] = -1.0/*0.0*/  * C_DEG2RAD;
        U[3] =  6.0  * C_DEG2RAD;

        // A/V model
        ModelGO(U);
    }
        
    // ...flight control code goes here...
    // ...
}

/**
* Helicopter Simulation Main
**/
int main(int argc, char** argv) {
    // Initialize the helicopter model
    F_MODEL_INIT();
    printf("Helicopter Simulation Initialized.\n\n");

    // call dynamical model @ 50Hz
    SetTimer(NULL, NULL, (int)(1000*windows_dt), (TIMERPROC)Dynamics);

    // initialize graphical library
    glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    
    // GUI
    // ... add gui here ...

    // open simulation window
    SimWindowInit();

    // graphical loop
    glutMainLoop();

    return(0);
}
