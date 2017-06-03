#ifndef PARTICLESYSTEM_H_INCLUDED
#define PARTICLESYSTEM_H_INCLUDED

#include <stdlib.h>

#define NODEAMOUNT 50
#define NUM_PARTICLES 2500
#define NUM_CONSTRAINTS 49

struct Vector3 {
	float x, y, z;
};

struct Constraint {
	int  particleA, particleB;
	float restlength;
};

struct Vector3 m_x[NODEAMOUNT];
struct Vector3 m_oldx[NODEAMOUNT];
struct Vector3 m_a[NODEAMOUNT];
struct Vector3 m_vGravity;
struct Vector3 anchorl, anchorr; //THE TOP CORNERS OF CLOTH, left and right
struct Constraint m_constraints[NUM_CONSTRAINTS];
int NUM_ITERATIONS;

int clothxy(int x, int y);
int getclothx(int particle);
int getclothy(int particle);
void TimeStep();
void AccumulateForces();
void Verlet();
void SatisfyConstraints();
void SetConstraint(int A, int B, float rest, int *id);


/*
1-48
48*48*8

//edge rows and columns
4 total
1-48
5 constraint each
48*5*4

//corner
2 total
3 each
2*3

(48*48*8)+(48*5*4)+(2*3)
*/
#endif
