
#include "ConfigReader.h"
#include "SockStream.h"
#include "stringUtility.h"
#include "galilController4143.h"
#include <time.h>

#include <string>
#include <iostream>
#include <sstream>


using namespace std;
using namespace tthread;

client_tcpsocket galilController4143::sock;

// PUBLIC FUNCTIONS

galilController4143::galilController4143()
{
}

bool galilController4143::initialize()
{
	if(setupSocket())
			initialized = startup();
	else
		initialized = 0;
	return initialized;
}

bool galilController4143::isInitialized()
{
	return initialized;
}

void galilController4143::running(void * aArg) {
	galilController4143* a = (galilController4143*)aArg;
	
	clock_t last_time, current_time;
	int loop_counter = 0;
	current_time = clock();
	last_time = current_time;
	float dt = 0.0;
	float phi_old = 0.0;

	while(1)
	{
		/****Calculate dt****/
		clock_t current_time = clock();
		dt = (current_time - last_time)/CLOCKS_PER_SEC;
		last_time = current_time;
		/********************/


		float L1 = a->chairParameter[0];
		float L2 = a->chairParameter[1];
		float L3 = a->chairParameter[2];
		float L5 = a->chairParameter[4];
		
		Matrix JwhA_3D(6,2);
		JwhA_3D(0,0) = ((L5*(cos(phi_old) + (2*L3*cos(phi_old) + 2*L2*sin(phi_old))/L1))/2);
		JwhA_3D(0,1) = ((L5*(cos(phi_old) - (2*L3*cos(phi_old) + 2*L2*sin(phi_old))/L1))/2);
		JwhA_3D(1,0) = ((L5*(sin(phi_old) - (2*L2*cos(phi_old) - 2*L3*sin(phi_old))/L1))/2);
		JwhA_3D(1,1) = ((L5*(sin(phi_old) + (2*L2*cos(phi_old) - 2*L3*sin(phi_old))/L1))/2);
		JwhA_3D(2,0) = (0);
		JwhA_3D(2,1) = (0);
		JwhA_3D(3,0) = (0);
		JwhA_3D(3,1) = (0);
		JwhA_3D(4,0) = (0);
		JwhA_3D(4,1) = (0);
		JwhA_3D(5,0) = (-L5/L1);
		JwhA_3D(5,1) = (L5/L1);

	}
}

void galilController4143::stop()
{
	this->command("ST AB");
}

bool galilController4143::setPosition(long position,int motor)
{
	string command;
	if(motor == 1)
	{
		command = "DPA";
	}
	else if(motor == 2)
	{
		command = "DPB";
	}
	else
	{
		return false;
	}

	this->command(command + "=" + to_string(long double(position)));
	return true;
}

bool galilController4143::setVelocity(long velocity,int motor)
{
	string command;
	if(motor == 1)
	{
		command = "SPA";
	}
	else if(motor == 2)
	{
		command = "SPB";
	}
	else
	{
		return false;
	}
			
	this->command(command + "=" + to_string(long double(velocity)));
	return true;
}

bool galilController4143::setAcceleration(long acceleration,int motor)
{
	string command;
	if(motor == 1)
	{
		command = "ACA";
	}
	else if(motor == 2)
	{
		command = "ACB";
	}
	else
	{
		return false;
	}
			
	this->command(command + "=" + to_string(long double(acceleration)));
	return true;
}

long galilController4143::getPosition(int motor)
{
	long encoderVal;
	string command, result;
	if(motor == 1)
	{
		command = "TPA";
	}
	else if(motor == 2)
	{
		command = "TPB";
	}
	else
	{
		cerr << "motor number outside range" << endl;
		throw std::out_of_range ("MotorNum out_of_range");
		return false;
	}

	result = this->command(command);
	istringstream stream(result);
	stream >> encoderVal;

	return encToAng(motor, encoderVal);   
}

long galilController4143::getVelocity(int motor)
{
	long encoderVal;
	string command, result;
	if(motor == 1)
	{
		command = "TVA";
	}
	else if(motor == 2)
	{
		command = "TVB";
	}
	else
	{
		cerr << "motor number outside range" << endl;
		throw std::out_of_range ("MotorNum out_of_range");
		return false;
	}

	result = this->command(command);
	
	istringstream stream(result);
	stream >> encoderVal;
	return encToAng(motor, encoderVal); 
}

long galilController4143::getTorque(int motor)
{
	long encoderVal;
	string command, result;
	if(motor == 1)
	{
		command = "TTA";
	}
	else if(motor == 2)
	{
		command = "TTB";
	}
	else
	{
		cerr << "motor number outside range" << endl;
		throw std::out_of_range ("MotorNum out_of_range");
		return false;
	}

	result = this->command(command);
	
	istringstream stream(result);
	stream >> encoderVal;
	return encToAng(motor, encoderVal); 
}

std::string galilController4143::command(std::string Command)
{
	char com[300];
	char ret[300];
	std::string c = Command + "\r";
	strcpy(com, c.c_str());
	commandGalil(com, ret, sizeof(ret));
	std::string ret_str(ret);
	return ret_str;
}


// PRIVATE FUNCTIONS

bool galilController4143::setupSocket()
{
	std::string IP = "192.168.1.40"; // #DEBUG - Change to wheelchair IP address

	const char * c = IP.c_str();
	sock.open(c,23);

	return galilController4143::sock.connected(); // #DEBUG - breaks if trying to setup sock while WMRA is off
	//return sock.is_open();
}

bool galilController4143::startup()
{
	t = new thread(running,this);
	float L1 = chairParameter[0];
	float L5 = chairParameter[4];
	Matrix q_dot_chair(2,1);
	q_dot_chair(0,0) = 0;
	q_dot_chair(1,0) = 0;
	Matrix dt = q_dot_chair;
	Matrix Xphi_dot(2,2);
	Xphi_dot(0,0) = (L5/2);
	Xphi_dot(0,1) = (L5/2);
	Xphi_dot(1,0) = (-L5/L1);
	Xphi_dot(1,1) = (L5/L1);
	Xphi_dot = Xphi_dot * q_dot_chair;

	Matrix DXphi_dot = Xphi_dot*dt;

	return galilController4143::sock.connected(); // #DEBUG - breaks if trying to setup sock while WMRA is off
	//return sock.is_open();
}

int galilController4143::commandGalil(char* Command, char* Response, int ResponseSize) //returns the number of bytes read
{
	//command() sends an ASCII Command (e.g. "TPX") to the controller and retrieves a Response (e.g. "123\r\n:").
	//The size of Response should be supplied as ResponseSize so that unallocated memory is not overwritten.
	//If you statically allocate your response buffer (e.g. char buff[100]) use sizeof(buff).
	char acPartialResponse[512] = {0}; //buffer to contain partial responses (which will be concatenated together to form the final response)
	int iPartialBytesRead = 0; //number of bytes read each time through the loop
	int iTotalBytesRead = 0;   //the total number of bytes read.  Can't exceed ResponseSize.  
	Response[0] = 0; //set response to null string    
	sock.write(Command, strlen(Command));

	//keep reading until we (a) get a colon (b) get a question mark (c) fill up the callers Response buffer
	while(1)
	{
		iPartialBytesRead = sock.read(acPartialResponse, sizeof(acPartialResponse)); //read some characters

		if(iPartialBytesRead <= 0)   //nothing read, keep reading until :
			continue;
		else if(iTotalBytesRead + iPartialBytesRead > ResponseSize) //get out of the loop if we will fill up the caller's buffer, iPartialBytesRead >= 1
			break;
		else {
			strncat(Response, acPartialResponse, iPartialBytesRead); //add the partial response to the full response.  Response is null terminated
			iTotalBytesRead += iPartialBytesRead; //tally up the total number of bytes read
			if (acPartialResponse[iPartialBytesRead - 1] == ':' || acPartialResponse[iPartialBytesRead - 1] == '?') //got a colon, iPartialBytesRead >= 1
				break;
		}
	}
	return(iTotalBytesRead);
}

bool galilController4143::setDefaults()
{
	chairParameter.resize(5); 
	ConfigReader reader;
	reader.parseFile("settings_wheelchair_controller.conf");
	reader.setSection("PARAMETERS");
	if(reader.keyPresent("L1")){			
		chairParameter[0] = reader.getInt("L1");
	}
	else{
		cout << "'L1' default not found" << endl;			
		return 0;
	}
	if(reader.keyPresent("L2")){			
		chairParameter[1] = reader.getInt("L2");
	}
	else{
		cout << "'L2' default not found" << endl;			
		return 0;
	}
	if(reader.keyPresent("L3")){			
		chairParameter[2] = reader.getInt("L3");
	}
	else{
		cout << "'L3' default not found" << endl;			
		return 0;
	}
	if(reader.keyPresent("L4")){			
		chairParameter[3] = reader.getInt("L4");
	}
	else{
		cout << "'L4' default not found" << endl;			
		return 0;
	}
	if(reader.keyPresent("L5")){			
		chairParameter[4] = reader.getInt("L5");
	}
	else{
		cout << "'L5' default not found" << endl;			
		return 0;
	}

	reader.setSection("MOTOR_CONTROLLER_DEFAULTS");
	if(reader.keyPresent("encoderPerRevolution1")){			
		enc2Radian[0] = 2*M_PI/reader.getInt("encoderPerRevolution1"); //calculate conversion values
	}
	else{
		cout << "'encoderPerRevolution1' default not found" << endl;			
		return 0;
	}
	if(reader.keyPresent("encoderPerRevolution2")){
		enc2Radian[1] = 2*M_PI/reader.getInt("encoderPerRevolution2"); //calculate conversion values
	}
	else{
		cout << "'encoderPerRevolution2' default not found" << endl;			
		return 0;
	}


	return 1;
}

double galilController4143::encToAng(int motor, long encCount)
{

	if(motor == 1)
	{
		return encCount * enc2Radian[0]; ;
	}
	else if(motor == 2)
	{
		return encCount * enc2Radian[1]; ;
	}
	else
	{
		cerr << "motor number outside range" << endl;
		throw std::out_of_range ("MotorNum out_of_range");
		return false;
	}
}