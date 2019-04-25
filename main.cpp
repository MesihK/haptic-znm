/**
 * Zenom - Hard Real-Time Simulation Enviroment
 * @author zenom
 *
 * Sine
 * A simple example of a control program.
 * Does not require any hardware, just generates a sine signal.
 */
#define termios asmtermios
#include <asm/termios.h>
#undef termios
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <stropts.h>
#include <iostream>
#include <controlbase.h>
#include <math.h>
#include <QFile>
#include <thread>
#include <mutex>
#define pi 3.141592653

#pragma pack(push, 1)
typedef struct {
	uint16_t sync1;
	uint16_t sync2;

	uint16_t tim1;
	uint16_t tim2;
	uint16_t tim3;
} mcu_msg_t;

typedef struct {
	uint16_t sync;

	float dac1;
	float dac2;
	float dac3;

	float curr1;
	float curr2;
	float curr3;

	uint16_t stop;
} zenom_msg_t;

typedef struct{
	uint16_t sync;

	uint16_t enc1;
	uint16_t enc2;
	uint16_t enc3;
} enc_msg_t;
#pragma pack(pop)

#define eps (pi/1800)
#define INTEGRAL_LIMIT 10
typedef struct {
	int enc;
	float err;
	float integral;
	float time;
	float kp;
	float ki;
	float kd;
} pid_param;

float pulse_to_radian(uint16_t pulse, pid_param pid){
	return (float)(pulse)/(float)(pid.enc)*pi*2;
}

float angle_to_radian(float angle){
	return (fmod(angle,360.0))/180.0*pi;
}

float radian_to_angle(float rad){
	return fmod(rad/pi*180.0,360.0);
}

float pulse_to_angle(uint16_t pulse, pid_param pid){
	return radian_to_angle(pulse_to_radian(pulse, pid));
}

float pid(float target, float curr, pid_param *param, float t){
	int dir = 1;
	float err, d1, d2 = 0;
	float d;
	float out;
	float dt = (float)(t - param->time); //time diff in sec.
	//printf("t %f, pt %f, dt:%f, le: %f\n", t, param->time, dt, param->err);
	param->time = t;

	d1 = target - curr;
	d2 = d1 + 2*pi;
	if(fabs(d2) >= fabs(d1)){
		dir = 1;
		err = d1;
	}
	else {
		err = d2;
		dir = -1;
	}

	//err = err / param->enc;
	if(fabs(err) > eps){
		param->integral = param->integral + err*dt;

		if(param->integral > INTEGRAL_LIMIT) param->integral = INTEGRAL_LIMIT;
		if(param->integral < -1*INTEGRAL_LIMIT) param->integral = -1*INTEGRAL_LIMIT;
	}
	if(dt > 0){
		d = (err - param->err) / dt;
	} else {
		d = 0;
	}

	out = err*param->kp + param->integral*param->ki + param->kd*d; 
	//out = out * (float)dir;

	//printf(" t: %5d, c: %5d, e:%8.5f, i:%8.5f, d:%8.5f, o: %6.2f m: %4d dt: %7.3f\r\n",
	//		target, curr, err, param->integral, d, out, vout_to_dac(out), dt);

	param->err = err;

	return out;
}

int set_interface_attribs (int fd, int speed, int parity) {

	struct termios tty;
	if (tcgetattr (fd, &tty) != 0)
	{
		printf ("error %d from tcgetattr: %s\n", errno, strerror(errno));
		return -1;
	}

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);
	tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;         /* 8-bit characters */
	tty.c_cflag &= ~PARENB;     /* no parity bit */
	tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 0;
	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		printf ("error %d from tcsetattr: %s\n", errno, strerror(errno));
		return -1;
	}
	return 0;
}

double radian_to_angle(double rad){
	return rad/pi*180.0;
}

double angle_to_radian(double angle){
	return angle/180.0*pi;
}

// robot geometry
const double e = 210.0;     // end effector
const double f = 105.0;     // base
const double re = 135.0;
const double rf = 60.0;

// trigonometric constants
const double sqrt3 = sqrt(3.0);
const double sin120 = sqrt3/2.0;   
const double cos120 = -0.5;        
const double tan60 = sqrt3;
const double sin30 = 0.5;
const double tan30 = 1/sqrt3;

// forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
// returned status: 0=OK, -1=non-existing position
int delta_calcForward(double theta1, double theta2, double theta3, double *x0, double *y0, double *z0) {
	double t = (f-e)*tan30/2;
	double dtr = pi/(double)180.0;

	theta1 *= dtr;
	theta2 *= dtr;
	theta3 *= dtr;

	double y1 = -(t + rf*cos(theta1));
	double z1 = -rf*sin(theta1);

	double y2 = (t + rf*cos(theta2))*sin30;
	double x2 = y2*tan60;
	double z2 = -rf*sin(theta2);

	double y3 = (t + rf*cos(theta3))*sin30;
	double x3 = -y3*tan60;
	double z3 = -rf*sin(theta3);

	double dnm = (y2-y1)*x3-(y3-y1)*x2;

	double w1 = y1*y1 + z1*z1;
	double w2 = x2*x2 + y2*y2 + z2*z2;
	double w3 = x3*x3 + y3*y3 + z3*z3;

	// x = (a1*z + b1)/dnm
	double a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
	double b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;

	// y = (a2*z + b2)/dnm;
	double a2 = -(z2-z1)*x3+(z3-z1)*x2;
	double b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;

	// a*z^2 + b*z + c = 0
	double a = a1*a1 + a2*a2 + dnm*dnm;
	double b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
	double c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);

	// discriminant
	double d = b*b - (double)4.0*a*c;
	if (d < 0) return -1; // non-existing point

	*z0 = -(double)0.5*(b+sqrt(d))/a;
	*x0 = (a1*(*z0) + b1)/dnm;
	*y0 = (a2*(*z0) + b2)/dnm;
	return 0;
}

// inverse kinematics
// helper functions, calculates angle theta1 (for YZ-pane)
int delta_calcAngleYZ(double x0, double y0, double z0, double *theta) {
	double y1 = -0.5 * tan30 * f; // f/2 * tg 30
	y0 -= 0.5 * tan30 * e;    // shift center to edge
	// z = a + b*y
	double a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
	double b = (y1-y0)/z0;
	// discriminant
	double d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
	if (d < 0) return -1; // non-existing point
	double yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
	double zj = a + b*yj;
	*theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
	return 0;
}

// inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
// returned status: 0=OK, -1=non-existing position
int delta_calcInverse(double x0, double y0, double z0, double *theta1, double *theta2, double *theta3) {
	*theta1 = 0;
	*theta2 = 0;
	*theta3 = 0;
	int status = delta_calcAngleYZ(x0, y0, z0, theta1);
	if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
	if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
	return status;
}

class Sine : public ControlBase
{
	public:
		// ----- User Functions -----
		// This functions need to be implemented by the user.
		int initialize();
		int start();
		int doloop();
		int stop();
		int terminate();
		uint16_t vout_to_dac(float);
		float v_to_dac_v(float v);

	private:
		// ----- Log Variables -----
		double sine;
		double tim1;
		double tim2;
		double tim3;

		double target1;
		double target2;
		double target3;

		double v1;
		double v2;
		double v3;

		double t_err1;
		double t_err2;
		double t_err3;

		// ----- Control Parameters -----
		double kp1;
		double kp2;
		double kp3;

		double kd1;
		double kd2;
		double kd3;

		double ki1;
		double ki2;
		double ki3;

		double enc1;
		double enc2;
		double enc3;

		double curr1;
		double curr2;
		double curr3;


		// ----- Variables -----
		QByteArray uart_buf;
		std::mutex uart_mcu_mutex; //lock for mcu_msg;
		std::thread uart_thread; //don't use state, uart, uart_buf outside of thread
		//thread variables
		int state;
		int uart_file;
		//shared variable
		zenom_msg_t znm_msg;
		mcu_msg_t mcu_msg;
		enc_msg_t enc_msg;

		void uart_work();
		void send_znm_msg();
		void send_enc_msg();

		pid_param pid1;
		pid_param pid2;
		pid_param pid3;

		double angle;
};

void Sine::send_znm_msg(){
	znm_msg.curr1 = curr1;
	znm_msg.curr2 = curr2;
	znm_msg.curr3 = curr3;

	uint8_t data[sizeof(zenom_msg_t)] = {0};
	memcpy(data, (void*)&znm_msg, sizeof(zenom_msg_t));
	write(uart_file, data, sizeof(zenom_msg_t));
}
void Sine::send_enc_msg(){
	enc_msg.sync = 0xACEF;
	enc_msg.enc1 = enc1;
	enc_msg.enc2 = enc2;
	enc_msg.enc3 = enc3;

	uint8_t data[sizeof(enc_msg_t)] = {0};
	memcpy(data, (void*)&enc_msg, sizeof(enc_msg_t));
	write(uart_file, data, sizeof(enc_msg_t));
}

void Sine::uart_work(){
	std::cout << "hello from thread!" << std::endl;
	char buff[256];
	int read_size = 0;
	while(1){
		//read from mcu
		read_size = read(uart_file, buff, sizeof(mcu_msg_t));
		uart_buf.append(buff, read_size);
		while(uart_buf.size() >= sizeof(mcu_msg_t)){

			//parse received data
			if((uint8_t)(uart_buf.at(0)) == 0xCD && (uint8_t)(uart_buf.at(1)) == 0xAB &&
					(uint8_t)(uart_buf.at(2)) == 0xAA && (uint8_t)(uart_buf.at(3)) == 0xEF){
				uart_mcu_mutex.lock();
				memcpy((void*)&mcu_msg, uart_buf.data(), sizeof(mcu_msg_t));
				uart_mcu_mutex.unlock();
				std::cout << ".";
				uart_buf.remove(0, sizeof(mcu_msg_t));

			} else {
				std::cout << "*";
				uart_buf.remove(0, 1);

			}
		}
		//----
	}

}

uint16_t Sine::vout_to_dac(float v){
	/*
	 * V+ = 5v * 6.8k/(10k + 6.8k) = 2.024
	 * (Vout - V+)/47k = (V+ - Vdac)/10k
	 * VOut = 5.7*V+ - 4.7Vdac
	 * VDac = (5.7*V+ - VOut)/4.7
	 * 5.7*V+ = 11.54
	 */
	if(v < -11.96) v = -11.96;
	if(v > 11.54) v = 11.54;
	float dac = (11.54 - v)/4.7;
	return (uint16_t)((dac/5.0f)*4095.0f);
}

float Sine::v_to_dac_v(float v){
	if(v < -11.96) v = -11.96;
	if(v > 11.54) v = 11.54;
	return v;
}

/**
 * This function is called when the control program is loaded to zenom.
 * Use this function to register control parameters, to register log variables
 * and to initialize control parameters.
 *
 * @return Return non-zero to indicate an error.
 */
int Sine::initialize()
{
	// ----- Initializes log and control variables -----
	// ----- Register the log variables -----
	registerLogVariable(&tim1, "tim1");
	registerLogVariable(&tim2, "tim2");
	registerLogVariable(&tim3, "tim3");

	registerLogVariable(&target1, "target1");
	registerLogVariable(&target2, "target2");
	registerLogVariable(&target3, "target3");

	registerLogVariable(&v1, "V1");
	registerLogVariable(&v2, "V2");
	registerLogVariable(&v3, "V3");

	registerLogVariable(&t_err1, "t_err1");
	registerLogVariable(&t_err2, "t_err2");
	registerLogVariable(&t_err3, "t_err3");
	// ----- Register the control paramateres -----
	registerControlVariable(&kp1, "kp1");
	registerControlVariable(&kp2, "kp2");
	registerControlVariable(&kp3, "kp3");

	registerControlVariable(&kd1, "kd1");
	registerControlVariable(&kd2, "kd2");
	registerControlVariable(&kd3, "kd3");

	registerControlVariable(&ki1, "ki1");
	registerControlVariable(&ki2, "ki2");
	registerControlVariable(&ki3, "ki3");

	enc1 = 4096-512;
	enc2 = 4096-512;
	enc3 = 4096-512;
	registerControlVariable(&enc1, "enc1");
	registerControlVariable(&enc2, "enc2");
	registerControlVariable(&enc3, "enc3");

	registerControlVariable(&curr1, "curr1");
	registerControlVariable(&curr2, "curr2");
	registerControlVariable(&curr3, "curr3");

	pid1 = {};
	pid2 = {};
	pid3 = {};

	pid1.enc = enc1;
	pid2.enc = enc2;
	pid3.enc = enc3;

	pid1.kp = 7.0f;
	pid1.kd = 1.0f;
	pid1.ki = 20.0f;

	pid2.kp = 7.0f;
	pid2.kd = 1.0f;
	pid2.ki = 20.0f;

	pid3.kp = 7.0f;
	pid3.kd = 1.0f;
	pid3.ki = 20.0f;

	angle = 0;

	curr1 = 3;
	curr2 = 3;
	curr3 = 3;

	state = 0;
	znm_msg.sync = 0xABCD;
	uart_file = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY );
	set_interface_attribs(uart_file, B921600, 0);
	std::cout << uart_file << std::endl;

	uart_thread = std::thread(&Sine::uart_work, this);

	// ----- Prints message in screen -----
	std::cout
		<< "This is a simple control program that generates "
		<< "a sine wave and doesn't access any hardware."
		<< "Use the amplitude control parameter to change "
		<< "the amplitude of the sine wave" << std::endl << std::endl;

	return 0;
}

/**
 * This function is called when the START button is pushed from zenom.
 *
 * @return If you return 0, the control starts and the doloop() function is
 * called periodically. If you return nonzero, the control will not start.
 */
int Sine::start()
{
	znm_msg.stop = 0;
	send_znm_msg();
	send_enc_msg();

	return 0;
}


/**
 * This function is called periodically (as specified by the control frequency).
 * The useful functions that you can call used in doloop() are listed below.
 *
 * frequency()          returns frequency of simulation.
 * period()             returns period of simulation.
 * duration()           returns duration of simulation.
 * simTicks()           returns elapsed simulation ticks.
 * simTimeInNano()      returns elapsed simulation time in nano seconds.
 * simTimeInMiliSec()   returns elapsed simulation time in miliseconds.
 * simTimeInSec()       returns elapsed simulation time in seconds.
 * overruns()           returns the count of overruns.
 *
 * @return If you return 0, the control will continue to execute. If you return
 * nonzero, the control will abort.
 */
int Sine::doloop()
{
	pid1.kp = kp1;
	pid2.kp = kp2;
	pid3.kp = kp3;

	pid1.kd = kd1;
	pid2.kd = kd2;
	pid3.kd = kd3;

	pid1.ki = ki1;
	pid2.ki = ki2;
	pid3.ki = ki3;

	angle += 0.2f;
	angle = fmod(angle, 360.0);

	delta_calcInverse(
			sin(elapsedTime())*8, //x
			cos(elapsedTime())*8, //y
			135+elapsedTime()/5,   //z
			&target1, &target2, &target3);
	target1 = angle_to_radian(-target1);
	target2 = angle_to_radian(-target2);
	target3 = angle_to_radian(-target3);
	//target1 = angle_to_radian(sin(angle_to_radian(angle))*15) + angle_to_radian(18.0f);
	//target2 = angle_to_radian(sin(angle_to_radian(angle))*15) + angle_to_radian(18.0f);
	//target3 = angle_to_radian(sin(angle_to_radian(angle))*15) + angle_to_radian(18.0f);

	uart_mcu_mutex.lock();
	tim1 = pulse_to_radian(enc1 - mcu_msg.tim1, pid1);
	tim2 = pulse_to_radian(enc2 - mcu_msg.tim2, pid2);
	tim3 = pulse_to_radian(enc3 - mcu_msg.tim3, pid3);

	znm_msg.dac1 = -1 * pid(target1, tim1, &pid1, elapsedTime());
	znm_msg.dac2 = -1 * pid(target2, tim2, &pid2, elapsedTime());
	znm_msg.dac3 = -1 * pid(target3, tim3, &pid3, elapsedTime());


	tim1 = radian_to_angle(pulse_to_radian(enc1 - mcu_msg.tim1, pid1));
	tim2 = radian_to_angle(pulse_to_radian(enc2 - mcu_msg.tim2, pid2));
	tim3 = radian_to_angle(pulse_to_radian(enc3 - mcu_msg.tim3, pid3));
	if(tim1 > 100)tim1 = 0;
	if(tim2 > 100)tim2 = 0;
	if(tim3 > 100)tim3 = 0;

	target1 = radian_to_angle(target1);
	target2 = radian_to_angle(target2);
	target3 = radian_to_angle(target3);

	/*
	   v1 = pid1.err;
	   v2 = pid2.err;
	   v3 = pid3.err;
	   */

	v1 = v_to_dac_v(znm_msg.dac1);
	v2 = v_to_dac_v(znm_msg.dac2);
	v3 = v_to_dac_v(znm_msg.dac3);

	uart_mcu_mutex.unlock();

	t_err1 = target1 - tim1;
	if(fabs(t_err1) > fabs(t_err1+360)) t_err1+=360;
	t_err2 = target2 - tim2;
	if(fabs(t_err2) > fabs(t_err2+360)) t_err2+=360;
	t_err3 = target3 - tim3;
	if(fabs(t_err3) > fabs(t_err3+360)) t_err3+=360;

	znm_msg.stop = 0;

	send_znm_msg();

	return 0;
}


/**
 * Called when a timed run ends or the STOP button is pushed from zenom.
 *
 * @return Return non-zero to indicate an error.
 */
int Sine::stop()
{

	znm_msg.stop = 1;
	send_znm_msg();

	return 0;
}


/**
 * This function is called when the control is unloaded. It happens when
 * the user loads a new control program or exits.
 *
 * @return Return non-zero to indicate an error.
 */
int Sine::terminate()
{


	znm_msg.stop = 1;
	send_znm_msg();

	uart_thread.join();
	close(uart_file);
	return 0;
}


/**
 * The main function starts the control program
 */
int main( int argc, char *argv[] )
{
	Sine c;
	c.run( argc, argv );

	return 0;
}


