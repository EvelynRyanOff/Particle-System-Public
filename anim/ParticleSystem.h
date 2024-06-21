#ifndef MY_PARTICLE_H
#define MY_PARTICLE_H

/*

	This is a sample system. It accepts the command "read" followed by the 
	path to an OBJ model.

*/


#include "BaseSystem.h"
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <GLmodel/GLmodel.h>
#include <iostream>
#include <string>
#include <sstream>
#include "shared/opengl.h"

// a sample system
class ParticleSystem : public BaseSystem
{ 

public:
	ParticleSystem( const std::string& name );
	virtual void getState( double *p );
	virtual void setState( double  *p );
	void reset( double time );

	void displayPart(Vector p, float r);
	void displaySpring(int index);
	void display( GLenum mode = GL_RENDER );

	
	int command(int argc, myCONST_SPEC char **argv) ;

protected:

	float m_sx;
	float m_sy;
	float m_sz;

	Vector m_pos ;
	Vector m_vel;
	double particles[1000][7]; //mass, x , y, z, vx, vy, vz, x0, y0, z0, vx0, vy0, vz0
	
	double springs[1000][9]; //index 1, index 2, ks, kd, rest_length, forcex, forcey, forcez, boolean force taken
	int part_num;
	int spring_num;

	//GLMmodel m_model ;

} ;
#endif
