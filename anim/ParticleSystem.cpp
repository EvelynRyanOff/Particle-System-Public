#include "ParticleSystem.h"
#include <string.h>
using namespace std;

ParticleSystem::ParticleSystem( const std::string& name ):
	BaseSystem( name ),
	m_sx( .5f ),
	m_sy( .5f ),
	m_sz( .5f )
{ 

	setVector( m_pos, 0, 0, 0 );
	

	for (int i = 0; i < 1000; i++) {
		particles[i][0] = -1;
		particles[i][1] = 0;
		particles[i][2] = 0;
		particles[i][3] = 0;
		particles[i][4] = 0;
		particles[i][5] = 0;
		particles[i][6] = 0;

		


		springs[i][0] = -1;
		springs[i][1] = -1;
		springs[i][2] = 0;
		springs[i][3] = 0;
		springs[i][4] = 0;
		springs[i][5] = 0;
		springs[i][6] = 0;
		springs[i][7] = 0;

		springs[i][8] = 0;



	}
	part_num = 0;

}	// ParticleSystem

void ParticleSystem::getState( double* p )
{ 

	VecCopy( p, m_pos ); 

}	// ParticleSystem::getState

void ParticleSystem::setState( double  *p )
{ 

	VecCopy(m_pos,p); 

}	// ParticleSystem::setState

void ParticleSystem::reset( double time ) 
{ 
	
	
	
	
}	// ParticleSystem::Reset


int ParticleSystem::command(int argc, myCONST_SPEC char **argv) 
{
	if( argc < 1 )
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str()) ;
		return TCL_ERROR ;
	}
	else if( strcmp(argv[0], "read") == 0 )
	{
		if( argc == 2 )
		{
			//m_model.ReadOBJ(argv[1]) ;
			//glmFacetNormals(&m_model) ;
			//glmVertexNormals(&m_model, 90) ;
			return TCL_OK ;
		}
		else 
		{
			animTcl::OutputMessage("Usage: read <file_name>") ;
			return TCL_ERROR ;
		}
	}
	else if( strcmp(argv[0], "scale") == 0 )
	{
		if( argc == 4 )
		{
			m_sx = (float)atof(argv[1]) ;
			m_sy = (float)atof(argv[2]) ;
			m_sz = (float)atof(argv[3]) ;
		}
		else
		{
			animTcl::OutputMessage("Usage: scale <sx> <sy> <sz> ") ;
			return TCL_ERROR ;

		}
	}
	else if( strcmp(argv[0], "pos") == 0 )
	{
		if( argc == 4 )
		{
			m_pos[0] = atof(argv[1]) ;
			m_pos[1] = atof(argv[2]) ;
			m_pos[2] = atof(argv[3]) ;
		}
		else
		{
			animTcl::OutputMessage("Usage: pos <x> <y> <z> ") ;
			return TCL_ERROR ;

		}
	}
	else if( strcmp(argv[0], "flipNormals") == 0 )
	{
		//flipNormals() ;
		return TCL_OK ;
		
	}
	else if( strcmp(argv[0], "reset") == 0)
	{
		double p[3] = {0,0,0} ;
		setState(p) ;
	}
	else if (strcmp(argv[0], "dim")==0) {
		if (argc == 2) {
			if (atoi(argv[1]) > 1000) {
				animTcl::OutputMessage("too many particles");
			}
			part_num = atoi(argv[1]);
			animTcl::OutputMessage(argv[1]);
		}
	}
	else if (strcmp(argv[0], "particle") == 0) {
		if (argc == 9) {
			int index = atoi(argv[1]);
			particles[index][0] = std::stod(argv[2]); //mass
			particles[index][1] = std::stod(argv[3]); //x
			particles[index][2] = std::stod(argv[4]); //y
			particles[index][3] = std::stod(argv[5]); //z
			particles[index][4] = std::stod(argv[6]); //vx
			particles[index][5] = std::stod(argv[7]); //vy
			particles[index][6] = std::stod(argv[8]); //vz
		
			
			animTcl::OutputMessage(argv[5]);
		}
	}
	else if (strcmp(argv[0], "getPos") == 0) {
		if (argc == 2) {
			int index = atoi( argv[1] );
			//animTcl::OutputMessage(argv[1]);
			m_pos[0] = particles[index][1];
			m_pos[1] = particles[index][2];
			m_pos[2] = particles[index][3];
		}
	}
	else if (strcmp(argv[0], "setPos") ==0) {

		if (argc == 2) {
			int index = atoi(argv[1]);
			//animTcl::OutputMessage(argv[1]);
			particles[index][1] = m_pos[0];
			particles[index][2] = m_pos[1];
			particles[index][3] = m_pos[2];
			/*
			stringstream strs;
			strs << particles[index][1];
			string temp_str = strs.str();
			char* char_type = (char*)temp_str.c_str();
			animTcl::OutputMessage(char_type);
			*/
		}
	}
	else if (strcmp(argv[0], "getVel") == 0) {
		if (argc == 2) {
			int index = atoi(argv[1]);
			//animTcl::OutputMessage(argv[1]);
			m_pos[0] = particles[index][4];
			m_pos[1] = particles[index][5];
			m_pos[2] = particles[index][6];
		}
	}
	else if (strcmp(argv[0], "setVel") == 0) {
		if (argc == 2) {
			int index = atoi(argv[1]);
			//animTcl::OutputMessage(argv[1]);
			particles[index][4] = m_pos[0];
			particles[index][5] = m_pos[1];
			particles[index][6] = m_pos[2];

		}
	}
	else if (strcmp(argv[0], "getMass") == 0) {
		if (argc == 2) {
			int index = atoi(argv[1]);
			//animTcl::OutputMessage(argv[1]);
			m_pos[0] = particles[index][0];
		}
		}
	else if (strcmp(argv[0], "setMass") == 0) {
		if (argc == 2) {
			int index = atoi(argv[1]);
			//animTcl::OutputMessage(argv[1]);
			particles[index][0] = m_pos[0];
		}
	}//
	else if (strcmp(argv[0], "getPartNum") == 0) {
		if (argc == 1) {
			
			//animTcl::OutputMessage(argv[1]);
			m_pos[0] = part_num;

			/*
			animTcl::OutputMessage("in getPartNum");
			stringstream strs;
			strs << part_num;
			string temp_str = strs.str();
			char* char_type = (char*)temp_str.c_str();
			animTcl::OutputMessage(char_type);
			*/
		}
		}
	else if (strcmp(argv[0], "all_velocities") ==0) {
			if (argc == 4) {

				for (int i = 0; i < part_num;i++) {
					particles[i][4] = stod(argv[1]);
					particles[i][5] = stod(argv[2]);
					particles[i][6] = stod(argv[3]);
				}

			}
	}if (strcmp(argv[0], "getIndex") == 0) {
		if (argc == 2) {
			int index = atoi(argv[1]);
			//animTcl::OutputMessage(argv[1]);
			m_pos[0] = springs[index][0];
			m_pos[1] = springs[index][1];
			m_pos[2] = 0;
		}
	}
	else if (strcmp(argv[0], "setIndex") == 0) {

		if (argc == 2) {
			int index = atoi(argv[1]);
			//animTcl::OutputMessage(argv[1]);
			springs[index][0] = m_pos[0];
			springs[index][1] = m_pos[1];

			/*
			stringstream strs;
			strs << springs[index][0];
			string temp_str = strs.str();
			char* char_type = (char*)temp_str.c_str();
			animTcl::OutputMessage(char_type);
			*/
		}
	}
	else if (strcmp(argv[0], "getInfo") == 0) {
		if (argc == 2) {
			int index = atoi(argv[1]);
			//animTcl::OutputMessage(argv[1]);
			m_pos[0] = springs[index][2];
			m_pos[1] = springs[index][3];
			m_pos[2] = springs[index][4];
		}
	}
	else if (strcmp(argv[0], "setInfo") == 0) {

		if (argc == 2) {
			int index = atoi(argv[1]);
			//animTcl::OutputMessage(argv[1]);
			springs[index][2] = m_pos[0];
			springs[index][3] = m_pos[1];
			springs[index][4] = m_pos[2];
			
			/*
			stringstream strs;
			strs << springs[index][2];
			string temp_str = strs.str();
			char* char_type = (char*)temp_str.c_str();
			animTcl::OutputMessage(char_type);
			*/

		}
	}
	else if (strcmp(argv[0], "getForce") == 0) {
		if (argc == 2) {
			int index = atoi(argv[1]);
			//animTcl::OutputMessage(argv[1]);
			m_pos[0] = springs[index][5];
			m_pos[1] = springs[index][6];
			m_pos[2] = springs[index][7];
			
			/*
			animTcl::OutputMessage("getforce");
			stringstream strs;
			strs << m_pos[0];
			string temp_str = strs.str();
			char* char_type = (char*)temp_str.c_str();
			animTcl::OutputMessage(char_type);*/
		}
	}
	else if (strcmp(argv[0], "setForce") == 0) {

		if (argc == 2) {
			int index = atoi(argv[1]);
			//animTcl::OutputMessage(argv[1]);
			springs[index][5] = m_pos[0];
			springs[index][6] = m_pos[1];
			springs[index][7] = m_pos[2];

			/*

			animTcl::OutputMessage("setforce");
			stringstream strs;
			strs << springs[index][5];
			string temp_str = strs.str();
			char* char_type = (char*)temp_str.c_str();
			animTcl::OutputMessage(char_type);
			animTcl::OutputMessage(argv[1]);*/
			
		}
	}
	else if (strcmp(argv[0], "getBool") == 0) {
		if (argc == 2) {
			int index = atoi(argv[1]);
			
			m_pos[0] = springs[index][8];

			/*
			stringstream strs;
			strs << m_pos[0];
			string temp_str = strs.str();
			char* char_type = (char*)temp_str.c_str();
			animTcl::OutputMessage(char_type);*/
			
		}
	}
	else if (strcmp(argv[0], "setBool") == 0) {

		if (argc == 2) {
			int index = atoi(argv[1]);
			//animTcl::OutputMessage(argv[1]);
			springs[index][8] = m_pos[0];

			/*
			stringstream strs;
			strs << particles[index][1];
			string temp_str = strs.str();
			char* char_type = (char*)temp_str.c_str();
			animTcl::OutputMessage(char_type);
			*/
		}
	}
	else if (strcmp(argv[0], "setSpring") == 0) {

		if (argc == 2) {

			//animTcl::OutputMessage(argv[1]);
			spring_num = atoi(argv[1]);

			/*
			stringstream strs;
			strs << spring_num;
			string temp_str = strs.str();
			char* char_type = (char*)temp_str.c_str();
			animTcl::OutputMessage(char_type);
			*/

		}
	}
    
    glutPostRedisplay() ;
	return TCL_OK ;

}	// ParticleSystem::command

void ParticleSystem::displayPart(Vector p, float r) {
	/*
	stringstream strs;
	strs << p[0];
	string temp_str = strs.str();
	char* char_type = (char*)temp_str.c_str();
	animTcl::OutputMessage(char_type );
	*/
	glPointSize(r);
	//glBegin(GL_POINTS);
	glVertex3d(p[0], p[1],p[2]);
	//glEnd();
}
void ParticleSystem::displaySpring(int index) {
	glLineWidth(1.5);

	glBegin(GL_LINE_STRIP);
	int i1, i2;
	i1 = springs[index][0];
	i2 = springs[index][1];
	glVertex3f(particles[i1][1], particles[i1][2], particles[i1][3]);
	glVertex3f(particles[i2][1], particles[i2][2], particles[i2][3]);

	glEnd();


}

void ParticleSystem::display( GLenum mode ) 
{
	glEnable(GL_LIGHTING) ;
	glMatrixMode(GL_MODELVIEW) ;
	glEnable(GL_COLOR_MATERIAL);
	glPushMatrix() ;
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glScalef(m_sx, m_sy, m_sz);
	
	
	
	
	glColor3f(1, 0, 1);
	glPointSize(10);
	glBegin(GL_POINTS);
	for (int i = 0; i < part_num; i++) {
		//if (particles[i][0] != -1) {
			//glTranslated(particles[i][1], particles[i][2], particles[i][3]);
			

			
			//glutSolidSphere(0.5, 20, 20);
			Vector p;
			p[0] = particles[i][1];
			p[1] = particles[i][2];
			p[2] = particles[i][3];
			displayPart(p, 10);



		//}
	}
	glEnd();
	for (int i = 0; i < spring_num; i++) {
		if (springs[i][0] != -1) {
			glColor3f(0, 1, 0);
			displaySpring(i);



		}

	}
	

	glPopMatrix();
	glPopAttrib();

}	// ParticleSystem::display
