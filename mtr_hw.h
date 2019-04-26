#define termios asmtermios
#include <asm/termios.h>
#undef termios
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <stropts.h>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <thread>
#include <string.h>
#include <mutex>
#include <QByteArray>

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

class mtr_hw
{
	public:
		mtr_hw(int enc1, int enc2, int enc3, std::string tty, int baud);	
		~mtr_hw();
		void start();
		void stop();
		void set(float v1, float v2, float v3,
			 float curr1, float curr2, float curr3);
		void get_enc(int *e1, int *e2, int *e3);
	private:
		QByteArray uart_buf;
		std::mutex uart_mcu_mutex; //lock for mcu_msg;
		std::thread uart_thread; //don't use state, uart, uart_buf outside of thread

		mcu_msg_t mcu_msg;
		enc_msg_t enc_msg;
		zenom_msg_t znm_msg; 
		int uart_file;

		void send_znm_msg();
		void send_enc_msg();
		void uart_work();
		int set_interface_attribs (int fd, int speed);
};
