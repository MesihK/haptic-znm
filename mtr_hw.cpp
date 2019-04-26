#include"mtr_hw.h"

int mtr_hw::set_interface_attribs (int fd, int speed) {
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

void mtr_hw::send_znm_msg(){
	znm_msg.sync = 0xABCD;

	uint8_t data[sizeof(zenom_msg_t)] = {0};
	memcpy(data, (void*)&znm_msg, sizeof(zenom_msg_t));
	write(uart_file, data, sizeof(zenom_msg_t));
}

void mtr_hw::send_enc_msg(){
	enc_msg.sync = 0xACEF;
	uint8_t data[sizeof(enc_msg_t)] = {0};
	memcpy(data, (void*)&enc_msg, sizeof(enc_msg_t));
	write(uart_file, data, sizeof(enc_msg_t));
}

void mtr_hw::uart_work(){
	char buff[256];
	int read_size = 0;
	while(1){
		//read from mcu
		read_size = read(uart_file, buff, sizeof(mcu_msg_t));
		uart_buf.append(buff, read_size);
		while(uart_buf.size() >= sizeof(mcu_msg_t)){
			//parse received data
			if((uint8_t)(uart_buf.at(0)) == 0xCD &&
			   (uint8_t)(uart_buf.at(1)) == 0xAB &&
			   (uint8_t)(uart_buf.at(2)) == 0xAA &&
			   (uint8_t)(uart_buf.at(3)) == 0xEF){
				uart_mcu_mutex.lock();
				memcpy((void*)&mcu_msg,
				       uart_buf.data(), sizeof(mcu_msg_t));
				uart_mcu_mutex.unlock();
				uart_buf.remove(0, sizeof(mcu_msg_t));
			} else {
				uart_buf.remove(0, 1);
			}
		}
	}
}

void mtr_hw::get_enc(int *e1, int *e2, int *e3){
	uart_mcu_mutex.lock();
	*e1 = mcu_msg.tim1; 
	*e2 = mcu_msg.tim2; 
	*e3 = mcu_msg.tim3; 
	uart_mcu_mutex.unlock();
}

mtr_hw::mtr_hw(int enc1, int enc2, int enc3, std::string tty, int baud) 
{
	memset(&mcu_msg, 0, sizeof(mcu_msg_t));
	memset(&znm_msg, 0, sizeof(mcu_msg_t));
	memset(&enc_msg, 0, sizeof(mcu_msg_t));

	enc_msg.enc1 = enc1;
	enc_msg.enc2 = enc2;
	enc_msg.enc3 = enc3;

	uart_file = open(tty.c_str(), O_RDWR | O_NOCTTY );
	set_interface_attribs(uart_file, baud);
	uart_thread = std::thread(&mtr_hw::uart_work, this);
}

mtr_hw::~mtr_hw(){
	//TODO: stop uart_thread
	znm_msg.stop = 1;
	send_znm_msg();
	close(uart_file);
}

void mtr_hw::start(){
	znm_msg.stop = 0;
	send_znm_msg();
	send_enc_msg();
}

void mtr_hw::stop(){
	znm_msg.stop = 1;
	send_znm_msg();
}

void mtr_hw::set(float v1, float v2, float v3,
		 float curr1, float curr2, float curr3){
	znm_msg.dac1 = v1;
	znm_msg.dac2 = v2;
	znm_msg.dac3 = v3;
	znm_msg.curr1 = curr1;
	znm_msg.curr2 = curr2;
	znm_msg.curr3 = curr3;
	send_znm_msg();
}


