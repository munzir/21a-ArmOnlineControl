// 00-singlearm


// #include "../../common/initModules.h"
//initModules stuff moved here
#include <unistd.h> //for usleep thing
#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <string>
#include <ctime>

#include <fstream>

using namespace std;

#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION



// Intialize variables
somatic_d_t daemon_cx;
somatic_motor_t singlearm;

bool start = true;
const int dof = 7;

double wf = 0.558048373585;

double currTime = 0.0;
double prevTime = 0.0;
double duration = 0.0;
double currentTime = 0.0;
clock_t startTime;
clock_t st;
double prev_Vel[7];
double qddot[7];
double prev_time;
bool flag = true;

int jj = 2;
//time_t startTime;

Eigen::Matrix<double, 7, 4> a, b;
double q0 	 [7] =  {-0.187, 0.085, -0.09, -0.014, -0.053, 0.187, 0.066};
//double qDOT[7] = {0, 0, 0, 0, 0, 0, 0};
// double dqref [8] = 	{0.0000,    0.0000,    0.0000,    0.0000,    0.0000,   	0.0000,    0.0000,    0.0000};

ofstream dataQ;
ofstream dataDotQ;
ofstream dataCur;
ofstream dataTimeStamp;
ofstream dataDDotQ;


/* ********************************************************************************************* */
void controlArm(){

		// Get currTime by diff with startTime
	
	
		duration = clock() - st;
		currentTime = (duration *100)/CLOCKS_PER_SEC;
		/*
		if (flag == true){
			duration = clock() - st;
			currentTime = (duration)/CLOCKS_PER_SEC;
		}
		else{
			duration = clock() - st;
			currentTime = (duration)/CLOCKS_PER_SEC;
			cout<<"heyyy"<<endl;
			currentTime -= 0.01;
		}
		*/
		//cout<<"CLOCKS_PER_SEC: "<< CLOCKS_PER_SEC<<endl;
		//cout<< "duration: "<<duration<<endl;
		//cout<< "clock(): "<<clock()<<endl;
		//struct timeval tv;
		//gettimeofday(&tv, NULL);

		//currTime = (tv.tv_sec)*1000 + (tv.tv_usec)/1000;
		

		currTime = (difftime(time(0), startTime));

		//cout<<"time(0)"<<time(0)<<endl;
		//cout<< " startTime: "<<startTime<<endl;
		//cout << "Seconds: " << currentTime << " s" << endl;
		//somatic_motor_update(&daemon_cx, &singlearm);
		// Print the current position
		//somatic_motor_update(&daemon_cx, &singlearm);
		if(currentTime - prevTime >= 0.02) {
			double timeStep = currentTime - prevTime;
			//cout<<"timeStep: "<<timeStep<<endl;
			prevTime = currentTime;
			somatic_motor_update(&daemon_cx, &singlearm);
			cout << "pos: "; for(int i=0; i<7; i++) { cout << singlearm.pos[i] <<  ", "; } cout << endl;
			cout << "vel: "; for(int i=0; i<7; i++) { cout << singlearm.vel[i] <<  ", "; } cout << endl;
			cout << "cur: "; for(int i=0; i<7; i++) { cout << singlearm.cur[i] <<  ", "; } cout << endl;
			//cout << "q0: "; for(int i=0; i<7; i++) { cout << q0[i] <<  ", "; } cout << endl;
			//cout << "qDOT: "; for(int i=0; i<7; i++) { cout << qDOT[i] <<  ", "; } cout << endl;
			if(flag == true){
				for(int i=0; i<7; i++) { prev_Vel[i] = singlearm.vel[i]; } cout << endl;
				prev_time = currentTime;
				cout << "Seconds: " << currentTime << " s" << endl;
				flag = false;
			}
			
			else{


				for(int i=0; i<7; i++) { dataQ << singlearm.pos[i] <<  ", "; }  dataQ <<endl;
				for(int i=0; i<7; i++) { dataDotQ << singlearm.vel[i] <<  ", "; } dataDotQ << endl;
				for(int i=0; i<7; i++) { dataCur << singlearm.cur[i] <<  ", "; } dataCur << endl;
				//for(int i=0; i<7; i++) { dataDOOOTQ << qDOT[i] <<  ", "; } dataDOOOTQ << endl;
				dataTimeStamp << currentTime << endl;

				for(int i=0; i<7; i++) 
				{ 
					qddot[i] = (singlearm.vel[i] - prev_Vel[i])/(currentTime - prev_time); 
				} 
		

				cout << "qddot: "; for(int i=0; i<7; i++) { cout << qddot[i] <<  ", "; } cout << endl;
				cout<< "current time: "<<currentTime<<endl;

				for(int i=0; i<7; i++) { dataDDotQ << qddot[i] <<  ", "; } dataDDotQ << endl;

				for(int i=0; i<7; i++) { prev_Vel[i] = singlearm.vel[i]; } cout << endl;
				prev_time = currentTime;
			
			//dataQ << singlearm.&pos <<  ", " << endl;
			}
			

		}

		for (int i = 0; i < dof; i++) {
				q0[i] = 0;
			}

		// Update qref value
		for (int joint = 0; joint < dof; joint++) {
		    for (int l = 1; l <= 4; l++) {

		    	q0[joint] = q0[joint] + (a(joint, l-1)/(wf*l))*sin(wf*l*currTime)
                    - (b(joint, l-1)/(wf*l))*cos(wf*l*currTime);

                  //qDOT[joint] = a(joint, l-1)*cos(wf*l*currTime)
                    //+ b(joint, l-1)*sin(wf*l*currTime);

			    // dqref[joint] = dqref[joint] + a(joint,l-1)*cos(wf*l*currTime)
	      // 			+ b(joint, l-1)*sin(wf*l*currTime);
		  }
		}
		

		// Output q0 vector nicely
		// cout << "q0: [ ";
		// for (int i = 0; i < dof; i++) {
		// 	cout << q0[i] << " ";
		// }
		// cout << "]" << endl << endl;

		//Test continuous position input (works)
			// if (q0[5] < 0.5){
			// 	q0[5] = q0[5]+0.05;
			// }
			// else{
			// 	q0[5] = q0[5]-0.05;
			// }
		somatic_motor_cmd(&daemon_cx, &singlearm, POSITION, q0, 7, NULL);
		
		// Give velocity input
		// somatic_motor_cmd(&daemon_cx, &singlearm, VELOCITY, dqref, 8, NULL);
		return;
}

// void readKeyboard(){
// }
// static inline void aa_fset( double *dst, double val, size_t n ) {
//     for( size_t i = 0; i < n; i ++ )
//         dst[i] = val;
// }

/* ********************************************************************************************* */
/// Initializes the arm
static void initArm (somatic_d_t& daemon_cx, somatic_motor_t& arm, const char* armName) {	

	// Get the channel names
	char cmd_name [16], state_name [16];
	sprintf(cmd_name, "%s-cmd", armName);
	sprintf(state_name, "%s-state", armName);

	// Initialize the arm with the daemon context, channel names and # of motors
	somatic_motor_init(&daemon_cx, &arm, 7, cmd_name, state_name);
	usleep(1e5);

	// Set the min/max values for valid and limits values
	double** limits [] = {
		&arm.pos_valid_min, &arm.vel_valid_min, 
		&arm.pos_limit_min, &arm.pos_limit_min, 
		&arm.pos_valid_max, &arm.vel_valid_max, 
		&arm.pos_limit_max, &arm.pos_limit_max};
		// **limits[0] = -1024.1;
		// **limits[1] = -1024.1;
		// **limits[2] = -1024.1;
		// **limits[3] = -1024.1;
		// **limits[4] = 1024.1;
		// **limits[5] = 1024.1;
		// **limits[6] = 1024.1;
		// **limits[7] = 1024.1;
	for(size_t i = 0; i < 4; i++) aa_fset(*limits[i], -1024.1, 7);
	for(size_t i = 4; i < 8; i++) aa_fset(*limits[i], 1024.1, 7);
	// Update and reset them
	somatic_motor_update(&daemon_cx, &arm);
	somatic_motor_cmd(&daemon_cx, &arm, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
	usleep(1e5);
}

/* ********************************************************************************************* */
void init () {
	
	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt));
	dopt.ident = "00-singlearm";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the arm
	initArm(daemon_cx, singlearm, "singlearm");

	// Set variables a, b
  	// a << -0.009, -0.36, 0.311, -0.362,
   //      -0.0, 0.0, -0.0, -0.0,
   //      -0.0, 0.0, -0.0, -0.0,
   //      -0.0, 0.0, -0.0, -0.0,
   //      -0.0, 0.0, -0.0, -0.0,
   //      -0.0, 0.0, -0.0, -0.0,
   //      -0.0, 0.0, -0.0, -0.0;

   //  b <<  -0.051, 0.027, 0.003, -0.332,
   //      -0.0, 0.0, -0.0, -0.0,
   //      -0.0, 0.0, -0.0, -0.0,
   //      -0.0, 0.0, -0.0, -0.0,
   //      -0.0, 0.0, -0.0, -0.0,
   //      -0.0, 0.0, -0.0, -0.0,
   //      -0.0, 0.0, -0.0, -0.0;

        a << -0.009, -0.36, 0.311, -0.362,
        0.095, -0.132, -0.363, 0.474,
        -0.418, -0.25, -0.12, 0.119,
        0.023, 0.113, 0.497, 0.213,
        -0.23, -0.237, 0.153, -0.147,
        0.366, 0.366, 0.302, -0.373,
        -0.247, -0.166, 0.315, 0.031;

    	b <<  -0.051, 0.027, 0.003, -0.332,
        -0.292, 0.358, -0.056, -0.436,
        -0.355, 0.039, -0.397, -0.445,
        0.328, 0.256, -0.36, 0.143,
        0.428, 0.093, 0.035, -0.28,
        -0.39, -0.085, 0.388, 0.46,
        -0.046, 0.135, -0.428, 0.387;

     // Set initial position
     somatic_motor_cmd(&daemon_cx, &singlearm, POSITION, q0, 7, NULL);
     usleep(3e6);
     //somatic_motor_update(&daemon_cx, &singlearm);
			//cout << "vel: "; for(int i=0; i<7; i++) { cout << singlearm.cur[i] <<  ", "; } cout << endl;
}

/* ********************************************************************************************* */
// Continuously process the data and set commands to the modules
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// To get currTime from difference with startTime at this point
	//struct timeval tv;
	//gettimeofday(&tv, NULL);

	startTime = time(0);
	st = clock();
	//cout<<"first startTime"<<startTime<<endl;
	//cout<<endl;
	//double st = clock();
	// Unless an interrupt or terminate message is received, process the new message
	while(!somatic_sig_received) {

		// Read the keyboard data - TO BE IMPLEMENTED IN THE FUTURE
		// readKeyboard();
		
		//double duration = clock() - st;
		//double currentTime = duration/CLOCKS_PER_SEC;
		//cout<<"current Time: "<<currentTime<<endl;
		// Control the arm
		controlArm();
		/*
		if (jj == 2){
			flag = false;
			jj = 3; 
		}
		*/
		// Free buffers allocated during this cycle
		aa_mem_region_release(&daemon_cx.memreg);	
		usleep(1e4);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ********************************************************************************************* */
void destroy() {

	// Halt the Schunk modules
	somatic_motor_cmd(&daemon_cx, &singlearm, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);

	cout << "Destroyed daemons and halted modules" << endl;
}

/* ********************************************************************************************* */
int main() {

	init();
	
	
	dataQ.open("/home/munzir/Documents/Software/experiments/teleop/build/dataQ.txt");
	dataDotQ.open("/home/munzir/Documents/Software/experiments/teleop/build/dataDotQ.txt");
	dataCur.open("/home/munzir/Documents/Software/experiments/teleop/build/dataCur.txt");
	dataTimeStamp.open("/home/munzir/Documents/Software/experiments/teleop/build/dataTimeStamp.txt");

	dataDDotQ.open("/home/munzir/Documents/Software/experiments/teleop/build/dataDDotQ.txt");
	

	 run();

	 dataQ.close();
	 dataDotQ.close();
	 dataCur.close();
	 dataTimeStamp.close();

	 dataDDotQ.close();

	 destroy();

	return 0;
}

//I think we should just iterater the control while reading file instead of reading the whole thing.
/*void controlArm_Tianhang(){


		int row = 0;
		int col;
		string delimiter = " ";
		size_t pos = 0;
		string value;

		string line;
		ifstream qdots("dataQdot.text");
		//get each line of the text and put it into variable "line"
		while (getline(qdots, line)){
			if (row != 0){
				col = 0; //start from zeroth column for each row


				double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

				//delimit by commas in the line and put values in velData matrix
				while ((pos = line.find(delimiter)) != string::npos) {
				    //get first value after delimiting
				    value = line.substr(0, pos);
				    //add delimited value to matrix
				    dq[col] = atof(value.c_str());
				    //erase first value + delimiter from line
				    line.erase(0, pos + delimiter.length());
				    //to access next column in velData matrix in the next loop
				    ++col;
				}
				++row;

				somatic_motor_cmd(&daemon_cx, &singlearm, VELOCITY, dq, 7, NULL);
			}
		}
}*/