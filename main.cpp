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

#pragma pack(push, 1)
typedef struct {
	uint16_t sync;

	uint16_t tim1;
	uint16_t tim2;
	uint16_t tim3;

	uint16_t target1;
	uint16_t target2;
	uint16_t target3;

	float err1;
	float err2;
	float err3;
} mcu_msg_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
	uint16_t sync;

	uint16_t kp1;
	uint16_t kp2;
	uint16_t kp3;

	uint16_t kd1;
	uint16_t kd2;
	uint16_t kd3;

	uint16_t ki1;
	uint16_t ki2;
	uint16_t ki3;

	uint16_t stop;
} zenom_msg_t;
#pragma pack(pop)

int set_interface_attribs (int fd, int speed, int parity) {

	struct termios tty;
	if (tcgetattr (fd, &tty) != 0)
	{
		printf ("error %d from tcgetattr", errno);
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
		printf ("error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
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

	private:
		// ----- Log Variables -----
		double sine;
		double tim1;
		double tim2;
		double tim3;

		double target1;
		double target2;
		double target3;

		double err1;
		double err2;
		double err3;



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

		void uart_work();
		void send_znm_msg();

};

void Sine::send_znm_msg(){
	uint8_t data[sizeof(zenom_msg_t)] = {0};
	memcpy(data, (void*)&znm_msg, sizeof(zenom_msg_t));
	write(uart_file, data, sizeof(zenom_msg_t));
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
			if((uint8_t)(uart_buf.at(0)) == 0xCD && (uint8_t)(uart_buf.at(1)) == 0xAB){
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

	registerLogVariable(&err1, "err1");
	registerLogVariable(&err2, "err2");
	registerLogVariable(&err3, "err3");

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

	state = 0;
	znm_msg.sync = 0xABCD;
	uart_file = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY );
	set_interface_attribs(uart_file, B460800, 0);

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
	uart_mcu_mutex.lock();
	tim1 = mcu_msg.tim1;
	tim2 = mcu_msg.tim2;
	tim3 = mcu_msg.tim3;

	target1 = mcu_msg.target1;
	target2 = mcu_msg.target2;
	target3 = mcu_msg.target3;

	err1 = mcu_msg.err1;
	err2 = mcu_msg.err2;
	err3 = mcu_msg.err3;
	uart_mcu_mutex.unlock();


	znm_msg.kp1 = (uint16_t)(kp1*100.0f);
	znm_msg.kp2 = (uint16_t)(kp2*100.0f);
	znm_msg.kp3 = (uint16_t)(kp3*100.0f);

	znm_msg.kd1 = (uint16_t)(kd1*100.0f);
	znm_msg.kd2 = (uint16_t)(kd2*100.0f);
	znm_msg.kd3 = (uint16_t)(kd3*100.0f);

	znm_msg.ki1 = (uint16_t)(ki1*100.0f);
	znm_msg.ki2 = (uint16_t)(ki2*100.0f);
	znm_msg.ki3 = (uint16_t)(ki3*100.0f);

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


