#include "particleSimulator.h"


particleSimulator::particleSimulator( const std::string& name, BaseSystem* target ):
	BaseSimulator( name ),
	m_object( target )
{
	euler = false;
	symplectic = true;
	verlet = false;
	timeStep = 0.01;
	prevTime = 0;
	gravity[0] = 0;
	gravity[1] = -9.8;
	gravity[2] = 0;
	spring_num = 0;
	kdrag = 0.05;
	fix = -1;
	kd = 300;
	ks = 50;
	mass = 0;
	spring_ind = -1;

}	// particleSimulator

particleSimulator::~particleSimulator()
{
}	// particleSimulator::~particleSimulator
int particleSimulator::command(int argc, myCONST_SPEC char** argv) {
	if (argc == 0) {
		return TCL_OK;
	}
	if (strcmp(argv[0], "link") == 0) {
		spring_num = atoi(argv[2]);
		spring_ind = 0;
		char** arg = new char* [2];
		arg[0] = "setSpring";
		arg[1] = argv[2];
	

		m_object->command(2, arg);


	}
	else if (strcmp(argv[0], "integration") == 0) {
		if (argc == 3) {
			if (strcmp(argv[1], "euler")== 0 ) {
				euler = true;
				symplectic = false;
				verlet = false;
			}
			else if (strcmp(argv[1], "symplectic")==0) {
				euler = false;
				symplectic = true;
				verlet = false;
			}
			else if (strcmp(argv[1], "verlet") == 0) {
				euler = false;
				symplectic = false;
				verlet = true;
			}

			timeStep = stod(argv[2]);
		}
		
	}
	else if (strcmp(argv[0], "gravity") == 0) {
		if (argc == 2) {
			gravity[1] = stod(argv[1]);
		}
	}else if (strcmp(argv[0], "fix")==0) {
		if (argc == 2) {
			
			fix = stoi(argv[1]);

		}
	}
	else if (strcmp(argv[0], "spring") == 0) {
		if (spring_ind == -1) {
			animTcl::OutputMessage("link first");
			return TCL_OK;

		}
		if (argc == 6) {
			if (spring_ind == spring_num|| spring_ind == 1000) {
				animTcl::OutputMessage("Max num springs reached");
				return TCL_OK;
			}
			Vector index, info;
			char** arg = new char* [2];
			index[0] = stod(argv[1]);
			index[1] = stod(argv[2]);
			index[2] = 0;
			info[0] = stod(argv[3]);
			info[1] = stod(argv[4]);
			info[2] = stod(argv[5]);

			m_object->setState(index);
			arg[0] = "setIndex";
			stringstream strs;
			strs << spring_ind;
			string temp_str = strs.str();
			char* char_type = (char*)temp_str.c_str();
			arg[1] = char_type;


			m_object->command(2, arg);

			arg[0] = "setInfo";

			m_object->setState(info);
			m_object->command(2, arg);

			spring_ind++;

		}

	}
	return TCL_OK;
}



double particleSimulator::Force(double* pos, double* vel, int index) {
	//to add spring and collision force soon!
	return (vel[index] * (-1 * kdrag) + mass * gravity[index]) / mass;
}
double particleSimulator::springForce(double* pos1, double* vel1, double* pos2, double* vel2, double* info, int index){
	
	double spFrc, length, dot, length2;
	Vector vec, vec2;
	vec[0] = pos1[0] - pos2[0];
	vec[1] = pos1[1] - pos2[1];
	vec[2] = pos1[2] - pos2[2];

	length = sqrt(pow(vec[0], 2)+ pow(vec[1], 2)+ pow(vec[2], 2));

	vec[0] = vec[0] / length;
	vec[1] = vec[1] / length;
	vec[2] = vec[2] / length;

	vec2[0] = vel1[0] - vel2[0];
	vec2[1] = vel1[1] - vel2[1];
	vec2[2] = vel1[2] - vel2[2];

	//length2 = sqrt(pow(vec2[0], 2) + pow(vec2[1], 2) + pow(vec2[2], 2));

	dot = vec2[0] * vec[0] + vec2[1] * vec[1] + vec2[2] * vec[2];

	spFrc = (info[0]*(info[2]-length)*vec[index])-(info[1]*dot*vec[index]);

	
	
	
	return spFrc;
}

void particleSimulator::springVector(double* pos, double* vel, int i, double* target) {

	Vector total_spr_force;
	total_spr_force[0] = 0;
	total_spr_force[1] = 0;
	total_spr_force[2] = 0;
	Vector ind, info, svd_frc, bol, pos2, vel2, temp_spr_force;
	char** arg = new char* [2];

	if (spring_num == 0) {
		return;
	}

	for (int j = 0; j < spring_num; j++) {
		

		arg[0] = "getIndex";
		stringstream strs;
		strs << j;
		string temp_str = strs.str();
		char* char_type = (char*)temp_str.c_str();
		//animTcl::OutputMessage("spring number");
		//animTcl::OutputMessage(char_type);


		arg[1] = char_type;
		m_object->command(2, arg);
		m_object->getState(ind);
		if (i == ind[0] || i == ind[1]) {
			//animTcl::OutputMessage("spring found");
			arg[0] = "getBool";
			m_object->command(2, arg);
			m_object->getState(bol);
			if (bol[0] == 0) {
				//no force saved
				//animTcl::OutputMessage("bol = 0");
				bol[0] = 1;
				m_object->setState(bol);
				arg[0] = "setBool";
				m_object->command(2, arg);
				stringstream strs1;
				
				
				if (i == ind[0]) {
					
					strs1 << ind[1];
					
				}
				else if (i == ind[1]) {
					
					strs1 << ind[0];
					
				}
				string temp_str1 = strs1.str();
				char* char_type1 = (char*)temp_str1.c_str();
				//collect spring info
				arg[0] = "getInfo";
				m_object->command(2, arg);
				m_object->getState(info);

				//collect data about other particle
					
				arg[0] = "getPos";
				arg[1] = char_type1;
				m_object->command(2, arg);
				m_object->getState(pos2);
				arg[0] = "getVel";
				m_object->command(2, arg);
				m_object->getState(vel2);

				//call spring force
				temp_spr_force[0] = springForce(pos, vel, pos2, vel2, info, 0);
				temp_spr_force[1] = springForce(pos, vel, pos2, vel2, info, 1);
				temp_spr_force[2] = springForce(pos, vel, pos2, vel2, info, 2);

				//save spring force


				arg[0] = "setForce";
				arg[1] = char_type;
				m_object->setState(temp_spr_force);
				m_object->command(2, arg);

				/*

				arg[0] = "getForce";
				m_object->command(2, arg);
				m_object->getState(svd_frc);
				*/



				//add the spring force to the particles total spring force
				total_spr_force[0] = total_spr_force[0] + temp_spr_force[0];
				total_spr_force[1] = total_spr_force[1] + temp_spr_force[1];
				total_spr_force[2] = total_spr_force[2] + temp_spr_force[2];

			


			}
			else if (bol[0] == 1) {
				//force saved
				//animTcl::OutputMessage("bol = 1");

				//animTcl::OutputMessage("index calling");
				//animTcl::OutputMessage(arg[1]);

				bol[0] = 0;
				m_object->setState(bol);
				arg[0] = "setBool";
				m_object->command(2, arg);
				arg[0] = "getForce";

				m_object->command(2, arg);
				m_object->getState(svd_frc);


				total_spr_force[0] -= svd_frc[0];
				total_spr_force[1] -= svd_frc[1];
				total_spr_force[2] -= svd_frc[2];

				/*
				stringstream strs1;
				strs1 << svd_frc[0];
				string temp_str1 = strs1.str();
				char* char_type1 = (char*)temp_str1.c_str();
				animTcl::OutputMessage("saved springforce");
				animTcl::OutputMessage(char_type1);*/
				
			}


		}

	}

	total_spr_force[0] = total_spr_force[0]/(sqrt(pow(total_spr_force[0], 2) + pow(total_spr_force[1], 2) + pow(total_spr_force[2], 2)));
	total_spr_force[1] = total_spr_force[1] / (sqrt(pow(total_spr_force[0], 2) + pow(total_spr_force[1], 2) + pow(total_spr_force[2], 2)));
	total_spr_force[2] = total_spr_force[2] / (sqrt(pow(total_spr_force[0], 2) + pow(total_spr_force[1], 2) + pow(total_spr_force[2], 2)));

	VecCopy(target, total_spr_force);
}

double particleSimulator::fn(double* pos, double* vel, int index) {
	double Fn;
	Fn = 0;
	double N[3];
	N[0] = 0;
	N[1] = 1;
	N[2] = 0;
	double P[3];
	P[0] = 0;
	P[1] = -11;
	P[2] = 0;
	double dot, dot2;


	//vel[0] = vel[0] / (sqrt(pow((vel[0]), 2) + pow(vel[1], 2)) + pow(vel[2], 2));
	//vel[1] = vel[1] / (sqrt(pow((vel[0]), 2) + pow(vel[1], 2)) + pow(vel[2], 2));
	//vel[2] = vel[2] / (sqrt(pow((vel[0]), 2) + pow(vel[1], 2)) + pow(vel[2], 2));		

	P[0] = pos[0] - P[0];
	P[1] = pos[1] - P[1];
	P[2] = pos[2] - P[2];

	P[0] = P[0] / (sqrt(pow((P[0]), 2) + pow(P[1], 2) + pow(P[2], 2)));
	P[1] = P[1] / (sqrt(pow((P[0]), 2) + pow(P[1], 2) + pow(P[2], 2)));
	P[2] = P[2] / (sqrt(pow((P[0]), 2) + pow(P[1], 2) + pow(P[2], 2)));

	dot = (N[0] * P[0]) + (N[1] * P[1]) + (N[2] * P[2]);
	dot2 = (vel[0] * N[0]) + (vel[1] * N[1]) + (vel[2] * N[2]);
	if (dot < 0 ) {

		Fn = -1 * (ks/1000) * (dot * N[index]) - (kd) * dot2 * N[index];

		/*
		if (Fn<gravity[1]*mass && index == 1) {
			Fn = gravity[1] * mass;
		}
		//Fn = 0;
		*/
		

		//return Fn;
	}
	return Fn/mass;

}

void particleSimulator::Euler(double* pos, double* vel, double time, int i) {
	


	Vector spring_force;
	springVector(pos, vel, i, spring_force);



	vel[0] = vel[0] + time * Force(pos, vel, 0)+ (spring_force[0]/mass)*time;
	vel[1] = vel[1] + time * Force(pos, vel, 1) + (spring_force[1] / mass) * time;
	vel[2] = vel[2] + time * Force(pos, vel, 2) + (spring_force[2] / mass) * time;


	double nu_pos[3];
	nu_pos[0] = pos[0] + time * vel[0];
	nu_pos[1] = pos[1] + vel[1] * time;
	nu_pos[2] = pos[2] + vel[2] * time;

	vel[0] += (fn(nu_pos, vel, 0)) * time;
	vel[1] += (fn(nu_pos, vel, 1)) * time;
	vel[2] += (fn(nu_pos, vel, 2)) * time;

	pos[0] = pos[0] + vel[0] * time;
	pos[1] = pos[1] + vel[1] * time;
	pos[2] = pos[2] + vel[2] * time;

	
	stringstream strs;
	strs << pos[1];
	string temp_str = strs.str();
	char* char_type = (char*)temp_str.c_str();
	animTcl::OutputMessage(char_type);
	animTcl::OutputMessage("euler");
	
	//   pos[1]= 2 * sin(2*3.14*time) ;
	
	if (pos[1] < 45) {

		
		pos[1] = -11;
		vel[0] -= (fn(nu_pos, vel, 0)) * time;
		vel[1] -= (fn(nu_pos, vel, 1)) * time;
		vel[2] -= (fn(nu_pos, vel, 2)) * time;
		
	}

}
void particleSimulator::Symplectic(double* pos, double* vel, double time, int i) {
	
	Vector spring_force;
	springVector(pos, vel, i, spring_force);

	/*
	stringstream strs;
	strs << spring_force[0];
	string temp_str = strs.str();
	char* char_type = (char*)temp_str.c_str();
	animTcl::OutputMessage(char_type);
	animTcl::OutputMessage("spring_force");
	*/
	vel[0] = vel[0] + (time+timeStep)* Force(pos,vel, 0) + (spring_force[0]/mass)*time;
	vel[1] = vel[1] + (time+timeStep) * Force(pos, vel, 1)+ (spring_force[1] / mass) * time;
	vel[2] = vel[2] + (time+timeStep) * Force(pos,vel, 2) + (spring_force[2] / mass) * time;

	double nu_pos[3];
	nu_pos[0] = pos[0] + time * vel[0];
	nu_pos[1] = pos[1] + vel[1] * time;
	nu_pos[2] = pos[2] + vel[2] * time;

	
	double force_n[3];
	force_n[0] = fn(nu_pos, vel, 0);
	force_n[1] = fn(nu_pos, vel, 1);
	force_n[2] = fn(nu_pos, vel, 2);
	/*
	force_n[0] = force_n[0] / (sqrt(pow((force_n[0]), 2) + pow(force_n[1], 2) + pow(force_n[2], 2)));
	force_n[1] = force_n[1] / (sqrt(pow((force_n[0]), 2) + pow(force_n[1], 2) + pow(force_n[2], 2)));
	force_n[2] = force_n[2] / (sqrt(pow((force_n[0]), 2) + pow(force_n[1], 2) + pow(force_n[2], 2)));
	*/

	vel[0] += (force_n[0]) * time;
	vel[1] += (force_n[1]) * time;
	vel[2] += (force_n[2]) * time;

	pos[0] = pos[0] + vel[0] * time;
	pos[1] = pos[1] + vel[1] * time;
	pos[2] = pos[2] + vel[2] * time;

	if (pos[1] < -45) {


		pos[1] = -11;
		vel[0] -= (fn(nu_pos, vel, 0)) * time;
		vel[1] -= (fn(nu_pos, vel, 1)) * time;
		vel[2] -= (fn(nu_pos, vel, 2)) * time;

	}
}
void particleSimulator::Verlet(double* pos, double* vel, double time, int i) {
	Vector spring_force;
	springVector(pos, vel, i, spring_force);

	Vector prevPos;
	if (prevTime == 0) {
		prevPos[0] = pos[0] - vel[0];
		prevPos[1] = pos[1] - vel[1];
		prevPos[2] = pos[2] - vel[2];
	}
	else {
		prevPos[0] = previousPositions[i][0];
		prevPos[1] = previousPositions[i][1];
		prevPos[2] = previousPositions[i][2];
	}

	previousPositions[i][0] = pos[0];	
	previousPositions[i][1] = pos[1];
	previousPositions[i][2] = pos[1];
	pos[0] = 2*pos[0] -( prevPos[0]) + (time)*(Force(pos,vel, 0)+(spring_force[0]/mass)) * time;
	pos[1] = 2 * pos[1] - (prevPos[1]) + (time)*(Force(pos, vel, 1) + (spring_force[1] / mass)) * time;
	pos[2] = 2 * pos[2] - (prevPos[2]) + (time)*(Force(pos,vel, 2) + (spring_force[2] / mass)) * time;
	
	
	pos[0] += time * fn(pos, vel, 0);
	pos[1] += time * fn(pos, vel, 1);
	pos[2] += time * fn(pos, vel, 2);
	
	vel[0] = (pos[0] - prevPos[0])/ (2*time);
	vel[1] = (pos[0] - prevPos[1]) / (2 * time);
	vel[2] = (pos[0] - prevPos[1]) / (2 * time);

	
	stringstream strs;
	strs << prevPos[1];
	string temp_str = strs.str();
	char* char_type = (char*)temp_str.c_str();
	animTcl::OutputMessage(char_type);
	animTcl::OutputMessage("verlet");


	if (pos[1] < -45) {


		pos[1] = -11;
		vel[0] -= (fn(pos, vel, 0)) * time;
		vel[1] -= (fn(pos, vel, 1));
		vel[2] -= (fn(pos, vel, 2)) * time;

	}
}





int particleSimulator::step(double time)
{
	
		
	Vector pos, vel;
	char** args = new char* [2];
	int part_num;
		

	Vector num;
		
	args[0] = "getPartNum";

	m_object->command(1, args);
	m_object->getState(num);


	part_num = num[0];

	for (int i = 0; i < part_num; i++) {


			
			
		stringstream strs;
		strs << i;
		string temp_str = strs.str();
		char* index = (char*)temp_str.c_str();
			//animTcl::OutputMessage(index);
		args[1] = index;

			
		args[0] = "getMass";
		m_object->command(2, args);
		m_object->getState(vel);
		mass = vel[0];
			
			/*
			
			*/

		args[0] = "getPos";
		args[1] = index;
		m_object->command(2, args);
		m_object->getState(pos);

			
			
			
		args[0] = "getVel";

		m_object->command(2, args);
		m_object->getState(vel);

		if (euler == true && i!= fix ) {
				
			Euler(pos, vel, time, i);	
		}
		else if (symplectic == true && i != fix) {
			Symplectic(pos, vel, time, i);

		}
		else if (verlet == true && i != fix) {
			Verlet(pos, vel, time, i);

		}


		args[0] = "setPos";
		//args[1] = "0";

		m_object->setState(pos);
		m_object->command(2, args);

		/*
		args[0] = "setVel";
		m_object->setState(vel);
		m_object->command(2, args);
		*/

		prevTime = time;
	}

	
	return 0;

}	// particleSimulator::step
