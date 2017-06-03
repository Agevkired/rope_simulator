//Derik Vega
#include <GL/gl.h>
#include <GL/glut.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include "shape.h"
#include "particlesystem.h"

#define BALL_POSITION_X 6
#define BALL_POSITION_Y -2
#define BALL_POSITION_Z 0
#define BALL_RADIUS 1
#define BIG_RADIUS 2
#define TRUE 1
#define FALSE 0
//#define SPRING_MIN 2.4
//#define SPRING_REST 2.7
//#define SPRING_MAX 3.4
#define SPRING_MIN 2.0
#define SPRING_REST 2.3
#define SPRING_MAX 2.6
//#define DIAG_SPRING_MIN 2.83
//#define DIAG_SPRING_REST 3.25
//#define DIAG_SPRING_MAX 3.68
#define SPRING_K 400
#define FAR_SPRING_K 500
#define TIMESTEP 10 //100 increment per second, default: 10
#define DAMPENING 0.01
#define MASS 1
#define RESTL 3

// *************************************************************************************
// * GLOBAL variables. Not ideal but necessary to get around limitations of GLUT API... *
// *************************************************************************************
int pause = FALSE;
struct shape ball, nodes[NODEAMOUNT][NODEAMOUNT];
int length = 3;
int oldtime, newtime; //Delta-time
float gravity = 10; // round to 10
float DIAG_SPRING_MIN, DIAG_SPRING_REST, DIAG_SPRING_MAX;
float DIAG_SPRING_K, DIAG_FAR_SPRING_K;
float fTimeStep = .1;

float getdistance3d (struct shape s1, struct shape s2);
float hookes (struct shape s1, struct shape s2);
float diaghookes (struct shape s1, struct shape s2);
float getunitvectx (struct shape s1, struct shape s2);
float getunitvecty (struct shape s1, struct shape s2);
float getunitvectz (struct shape s1, struct shape s2);
void getangles (struct shape s1, struct shape s2, float *theta, float *phi);
void handlesprings (struct shape *s1, struct shape *s2, float *fx, float *fy, float *fz);
void diaghandlesprings (struct shape *s1, struct shape *s2, float *fx, float *fy, float *fz);
void satisfyconstraints (struct shape *s1, struct shape *s2);
void diagsatisfyconstraints (struct shape *s1, struct shape *s2);

void init (void)
{
	glShadeModel (GL_SMOOTH);
	glClearColor (0.2f, 0.2f, 0.4f, 0.5f);
	glClearDepth (1.0f);
	glEnable (GL_DEPTH_TEST);
	glDepthFunc (GL_LEQUAL);
	glEnable (GL_COLOR_MATERIAL);
	glHint (GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glEnable (GL_LIGHTING);
	glEnable (GL_LIGHT0);
	GLfloat lightPos[4] = {-1.0, 1.0, 0.5, 0.0};
	glLightfv (GL_LIGHT0, GL_POSITION, (GLfloat *) &lightPos);
	glEnable (GL_LIGHT1);
	GLfloat lightAmbient1[4] = {0.0, 0.0,  0.0, 0.0};
	GLfloat lightPos1[4]     = {1.0, 0.0, -0.2, 0.0};
	GLfloat lightDiffuse1[4] = {0.5, 0.5,  0.3, 0.0};
	glLightfv (GL_LIGHT1,GL_POSITION, (GLfloat *) &lightPos1);
	glLightfv (GL_LIGHT1,GL_AMBIENT, (GLfloat *) &lightAmbient1);
	glLightfv (GL_LIGHT1,GL_DIFFUSE, (GLfloat *) &lightDiffuse1);
	glLightModeli (GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
}

void display (void)
{
	int dumi = 0, n, m; //int dummy
	float storedist = 0, springv; //float dummy
	float theta, phi;
	float dumf = 0, dumf2, vmag, reangle;
	float fx=0, fy=0, fz=0;
	float ax=0, ay=0, az=0;
	float x=0, y=0, z=0; //TEMP COORDINATE

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity ();
	glDisable (GL_LIGHTING);
	glBegin (GL_POLYGON);
	glColor3f (0.8f, 0.8f, 1.0f);
	glVertex3f (-200.0f, -100.0f, -100.0f);
	glVertex3f (200.0f, -100.0f, -100.0f);
	glColor3f (0.4f, 0.4f, 0.8f);
	glVertex3f (200.0f, 100.0f, -100.0f);
	glVertex3f (-200.0f, 100.0f, -100.0f);
	glEnd ();
	glEnable (GL_LIGHTING);
	//glTranslatef (-6.5, 6, -100.0f); // move camera out and center on the rope
	glTranslatef (70, 30, -80.0f);

	/*glPushMatrix (); //Draw big ball
	glTranslatef (ball.x, ball.y, ball.z);
	glColor3f (1.0f, 0.0f, 0.0f);
	glutSolidSphere (BIG_RADIUS - 0.1, 16, 16); // draw the ball, but with a slightly lower radius, otherwise we could get ugly visual artifacts of rope penetrating the ball slightly
	glPopMatrix ();*/

	for(n = 0; n < NODEAMOUNT; n++) //render the ROPE
	{
		glPushMatrix (); //Draw next sphere
		glTranslatef(m_x[n].x, m_x[n].y, m_x[n].z);
		//printf("n: %d  x: %f y: %f z: %f\n", n, nodes[clothxy(n,m)].x, nodes[clothxy(n,m)].y, nodes[clothxy(n,m)].z);
		glColor3f (1.0f, 1.0f, 0.0f);
		glutSolidSphere (BALL_RADIUS - 0.1, 4, 4); // draw the ball, but with a slightly lower radius, otherwise we could get ugly visual artifacts of rope penetrating the ball slightly
		glPopMatrix ();
	}

	for(n = 0; n < NODEAMOUNT-1; n++) //render the ROPE line
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glBegin (GL_LINES);
		glColor3f ( (n+1)%2, n%2, 1);
		glVertex3f(m_x[n].x, m_x[n].y, m_x[n].z);
		glVertex3f(m_x[n+1].x, m_x[n+1].y, m_x[n+1].z);
		glEnd();
	}
	glFlush();

	glutSwapBuffers();
	glutPostRedisplay();

	//if(((newtime - oldtime) > TIMESTEP - 1) && pause) //Once timestep complete
	if(pause) //REMOVED STEPS
	{
		TimeStep();
		oldtime = newtime; //start another count

		for(n = 0; n < NUM_PARTICLES; n++){
			//printf("n: %d x: %f y: %f z: %f\n", n, m_x[n].x, m_x[n].y, m_x[n].z);
		}
	}

	//printf("%d, %d\n", oldtime, newtime);
	newtime = glutGet(GLUT_ELAPSED_TIME); //Get new time
}

void handlesprings (struct shape *s1, struct shape *s2, float *fx, float *fy, float *fz)
{
	float storedist, dumf, theta, phi;
	storedist = getdistance3d (*s1, *s2);
	if((SPRING_REST - storedist) != 0) //HANDLE SPRING
	{
		dumf = hookes(*s1, *s2); //velocity from spring
		*fx += dumf * (s2->x - s1->x)/storedist;
		*fy += dumf * (s2->y - s1->y)/storedist;
		*fz += dumf * (s2->z - s1->z)/storedist;
	}
}

void diaghandlesprings (struct shape *s1, struct shape *s2, float *fx, float *fy, float *fz)
{
	float storedist, dumf, theta, phi;
	storedist = getdistance3d (*s1, *s2);
	if((SPRING_REST - storedist) != 0) //HANDLE SPRING
	{
		dumf = diaghookes(*s1, *s2); //velocity from spring
		*fx += dumf * (s2->x - s1->x)/storedist;
		*fy += dumf * (s2->y - s1->y)/storedist;
		*fz += dumf * (s2->z - s1->z)/storedist;
	}
}

void satisfyconstraints (struct shape *s1, struct shape *s2)
{
	float delta, deltalength, diff;
	delta = s2->x - s1->x;
	deltalength = sqrt(delta * delta);
	diff = (deltalength - SPRING_REST)/(deltalength * (2/MASS));
	s1->nx -= MASS*delta*diff;
	s2->nx += MASS*delta*diff;

	delta = s2->y - s1->y;
	deltalength = sqrt(delta * delta);
	diff = (deltalength - SPRING_REST)/(deltalength * (2/MASS));
	s1->ny -= MASS*delta*diff;
	s2->ny += MASS*delta*diff;

	delta = s2->z - s1->z;
	deltalength = sqrt(delta * delta);
	diff = (deltalength - SPRING_REST)/(deltalength * (2/MASS));
	s1->nz -= MASS*delta*diff;
	s2->nz += MASS*delta*diff;
}

void diagsatisfyconstraints (struct shape *s1, struct shape *s2)
{
	float delta, deltalength, diff;
	delta = s2->x - s1->x;
	deltalength = sqrt(delta * delta);
	diff = (deltalength - DIAG_SPRING_REST)/(deltalength * (2/MASS));
	s1->nx -= MASS * delta * diff;
	s2->nx += MASS * delta * diff;

	delta = s2->y - s1->y;
	deltalength = sqrt(delta * delta);
	diff = (deltalength - DIAG_SPRING_REST)/(deltalength * (2/MASS));
	s1->ny -= MASS*delta*diff;
	s2->ny += MASS*delta*diff;

	delta = s2->z - s1->z;
	deltalength = sqrt(delta * delta);
	diff = (deltalength - DIAG_SPRING_REST)/(deltalength * (2/MASS));
	s1->nz -= MASS*delta*diff;
	s2->nz += MASS*delta*diff;
}

float getdistance3d (struct shape s1, struct shape s2) //xyz coordinate
{
	float x = s2.x - s1.x;
	float y = s2.y - s1.y;
	float z = s2.z - s1.z;
	return sqrt((x * x) + (y * y) + (z * z));
}

float hookes (struct shape s1, struct shape s2) //returns the force
{
    float a, v, dist;
    dist = getdistance3d(s1, s2);
	dist = SPRING_REST - dist;
	//printf("dist from spring: %f\n", dist);
	if((dist > SPRING_MAX) || (dist < SPRING_MIN)) //EXCEEDING SPRING THRESHOLD
	{
		return FAR_SPRING_K * dist/2;
	}
	return SPRING_K * dist / 2;
}

float diaghookes (struct shape s1, struct shape s2) //returns the force
{
    float a, v, dist;
    dist = getdistance3d(s1, s2);
	dist = SPRING_REST - dist;
	//printf("dist from spring: %f\n", dist);
	if((dist > DIAG_SPRING_MAX) || (dist < DIAG_SPRING_MIN)) //EXCEEDING SPRING THRESHOLD
	{
		return DIAG_FAR_SPRING_K * dist/2;
	}
	return DIAG_SPRING_K * dist / 2;
}

float getunitvectx (struct shape s1, struct shape s2)
{
	float dumf = getdistance3d (s1, s2);
	float x = s2.x - s1.x;
	return x / dumf;
}

float getunitvecty (struct shape s1, struct shape s2)
{
	float dumf = getdistance3d (s1, s2);
	float y = s2.y - s1.y;
	return y / dumf;
}

float getunitvectz (struct shape s1, struct shape s2)
{
	float dumf = getdistance3d (s1, s2);
	float z = s2.z - s1.z;
	return z / dumf;
}

void getangles (struct shape s1, struct shape s2, float *theta, float *phi) //s2: current node
{
	float x, y, z;
	float r, p;
	x = s2.x - s1.x;
	y = s2.y - s1.y;
	z = s2.z - s1.z;
	r = sqrtf((x*x) + (y*y) + (z*z));
	p = sqrtf((x*x) + (y*y));
	*theta = acosf(z/r);
	*phi = atanf(y/x);
	if(x < 0) //IF VECTOR IS IN 4th QUADRANT
	{
		*phi += M_PI;
	}
	if(*phi < 0) //MAKE PHI: 0 <= phi <= 2 * M_PI
	{
		*phi += 2 * M_PI;
	}
}

//*****************
//NEW FUNCTIONS
//SET CONSTRAINT
void SetConstraint(int A, int B, float rest, int *id){
	m_constraints[*id].particleA = A;
	m_constraints[*id].particleB = B;
	m_constraints[*id].restlength = rest;
	*id+=1;
}

//GET XY OF CLOTH and VICE VERSA
int clothxy(int x, int y){
	return x + (y*50); //(0,0) -> (x,y) standard array procedure
}
int getclothx(int particle){
	return particle % 50;
}
int getclothy(int particle){
	return particle / 50;
}

void TimeStep() {
	AccumulateForces();
	Verlet();
	SatisfyConstraints();
}

// This function should accumulate forces for each particle
void AccumulateForces(){
	int i;
	// All particles are influenced by gravity
	for(i=0; i<NODEAMOUNT; i++)
		m_a[i] = m_vGravity;
}

// Verlet integration step
void Verlet() {
	struct Vector3 x, temp, oldx, a;
	int i;
	for(i=0; i<NODEAMOUNT; i++) {
		x = m_x[i];
		temp = x;
		oldx = m_oldx[i];
		a = m_a[i];
		x.x += (x.x - oldx.x + a.x * fTimeStep*fTimeStep);  //<-- a*t^2
		x.y += (x.y - oldx.y + a.y * fTimeStep*fTimeStep);  //<-- a*t^2
		x.z += (x.z - oldx.z + a.z * fTimeStep*fTimeStep);  //<-- a*t^2
		m_x[i] = x; //set new position
		oldx = temp;
		m_oldx[i] = oldx; //set old to previous position
		//printf("i: %d x: %f y: %f z: %f\n", i, m_x[i].x, m_x[i].y, m_x[i].z);
	}
	//m_x[0]  = anchorl;
	//m_x[49] = anchorr;
}

// Assume that an array of constraints, m_constraints, exists
void SatisfyConstraints() {
	struct Vector3 delta;
	struct Vector3 x1, x2;
	struct Constraint c;
	float deltalength, diff;
	int i, j;
	for(j=0; j<NUM_ITERATIONS; j++) {
		for(i=0; i<NUM_CONSTRAINTS; i++) {
			c = m_constraints[i];
			x1 = m_x[c.particleA];
			x2 = m_x[c.particleB];
			//GET DELTA
			delta.x = x2.x - x1.x;
			delta.y = x2.y - x1.y;
			delta.z = x2.z - x1.z;
			deltalength = sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z);
			diff = (deltalength - c.restlength) / deltalength;
			x1.x += delta.x * 0.5*diff;
			x1.y += delta.y * 0.5*diff;
			x1.z += delta.z * 0.5*diff;

			x2.x -= delta.x * 0.5*diff;
			x2.y -= delta.y * 0.5*diff;
			x2.z -= delta.z * 0.5*diff;
			m_x[c.particleA] = x1;
			m_x[c.particleB] = x2;
			//printf("i: %d x1.x: %f x1.y: %f x1.z: %f\n", i, x1.x, x1.y, x1.z);
		}
		// Constrain one particle of the cloth to origo
		//m_x[0] = Vector3(0,0,0);
		m_x[0]  = anchorl;
		m_x[49] = anchorr;
	}
}
//END FUNCTIONS
//*******************

void reshape (int w, int h)
{
	glViewport (0, 0, w, h);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	if (h == 0)
	{
		gluPerspective (80, (float) w, 1.0, 5000.0);
	}
	else
	{
		gluPerspective (80, (float) w / (float) h, 1.0, 5000.0);
	}
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
}

void keyboard (unsigned char key, int x, int y)
{
	switch (key)
	{
		case 27:
			exit (0);
		break;
		case 32: //SPACE KEY
			pause = 1 - pause;
		break;
		default:
		break;
	}
}

void arrow_keys (int a_keys, int x, int y)
{
	switch(a_keys)
	{
		case GLUT_KEY_UP:
			glutFullScreen();
		break;
		case GLUT_KEY_DOWN:
			glutReshapeWindow (960, 540 );
		break;
		default:
		break;
	}
}

int main (int argc, char *argv[])
{
	int n, m;
	int constrcounter = 0; //CONSTRAINT COUNTER
	oldtime = glutGet(GLUT_ELAPSED_TIME);
	oldtime = 0;
	for(n = 0; n<NODEAMOUNT; n++) //0-49
	{
		m_x[n].x = -130 + (3 * n); //COORDINATES
		m_x[n].y = 0;
		m_x[n].z = 0;
		m_oldx[n].x = -130 + (3 * n); //COPY COORDINATES
		m_oldx[n].y = 0;
		m_oldx[n].z = 0;
		m_a[n].x = 0; //ACCELERATION
		m_a[n].y = 0;
		m_a[n].z = 0;

	}
	m_vGravity.x = 0; //GRAVITY VECTOR
	m_vGravity.y = -gravity;
	m_vGravity.z = 0;
	//SETUP TWO ANCHORS
	anchorl = m_x[0];
	anchorr = m_x[49];
	//SET CONSTRAINTS
	for(n = 0; n<NODEAMOUNT-1; n++){ //0-48  prevent going beyond size
		SetConstraint(n, n+1, RESTL, &constrcounter);
	}
	NUM_ITERATIONS = 50;
	printf("constraint counter: %d\n", constrcounter);

	ball.x = -110;
	ball.x = -80;
	ball.y = -4;
	ball.z = 0;
	glutInit (&argc, argv);
	glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize (960, 540 );
	//glutInitWindowSize (1600, 900 );
	//glutInitWindowSize (1280, 720 );
	glutCreateWindow ("Rope simulator");
	init ();
	glutDisplayFunc (display);
	glutReshapeFunc (reshape);
	glutKeyboardFunc (keyboard);
	glutSpecialFunc (arrow_keys);
	glutMainLoop ();
}
