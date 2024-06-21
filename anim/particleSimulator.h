#ifndef MY_PARTSIM_H
#define MY_PARTSIM_H

#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include <iostream>
#include <string>
#include <sstream>
#include <string>
#include "ParticleSystem.h"
#include <string.h>
#include <print>
#include "anim.h"
using namespace std;
// a sample simulator

extern double timeStep;


class particleSimulator : public BaseSimulator 
{
public:

	particleSimulator( const std::string& name, BaseSystem* target );
	~particleSimulator();

	double springForce(double* pos1, double* vel1, double* pos2, double* vel2,double* info, int index);
	double Force(double* pos, double* vel, int index);
	virtual void springVector(double* pos, double* vel, int i, double* target);

	//double velocity(double vel, double force);
	virtual void Euler(double* pos, double* vel, double time, int i);
	virtual void Symplectic(double* pos, double* vel, double time, int i);
	virtual void Verlet(double* pos, double* vel, double time, int i);
	double fn(double* pos, double* vel, int index);

	int step(double time);
	int init(double time) 
	{ 
		/*
		timeStep = 0.01;
		prevTime = 0;
		gravity[0] = 0;
		gravity[1] = -9.8;
		gravity[2] = 0;
		kdrag = 5;
		fix = -1;
		*/
	
		

		return 0;
	};

	int command(int argc, myCONST_SPEC char** argv);

protected:





	BaseSystem* m_object;


	int spring_ind;
	long double prevTime;
	double gravity[3];
	double mass;
	double kdrag;
	bool euler;
	bool symplectic;
	bool verlet;
	int fix;
	double kd;
	double ks;
	//int fix;
	double previousPositions[1000][3];
	int spring_num;

};


#endif