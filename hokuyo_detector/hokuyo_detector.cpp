// Detect hokuyo UST lidar and prit its serial number to be used with the udev rule

#include <chrono>
#include <cmath>
#include <string>
#include <iostream>
#include <stdexcept>
#include <cstring>
#include <csignal>

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h> 
#include <unistd.h>

#define HOKUYO_CMD_ID				"#IN0D54\n"

// Serial port name '/dev/blablabla'
std::string serial_port_;
struct termios tty;

// Probing buffer
std::string find_ident;

// Serial port file descriptor
int serial_port = 0;

int rx_bytes = 0;
int rx_accumu_bytes = 0;

int serial_num_begin;
int serial_num_end;

int delay_count = 0;

void hokuyo_writeCmd(std::string hokuyoCMD){
	char *hokuyo_cmd_c = new char[hokuyoCMD.length()];
	strcpy(hokuyo_cmd_c, hokuyoCMD.c_str());
	
	write(
		serial_port,
		hokuyo_cmd_c,
		hokuyoCMD.length()
		);
	
}

int hokuyo_getRxBytes(){
	ioctl(
		serial_port,
		FIONREAD,
		&rx_bytes
		);
	
	return rx_bytes;
}

bool hokuyo_checkRxEqual(int expected_rx){
	return (hokuyo_getRxBytes() >= expected_rx) ? true : false;
}

void hokuyo_readBytes(char *cptr, int clen){
	read(
		serial_port,
		cptr,
		clen
		);
}

void hokuyo_flushSerial(){
	tcflush(
		serial_port,
		TCIOFLUSH
	);
}

void hokuyo_cmdID(){
	hokuyo_writeCmd(HOKUYO_CMD_ID);
}

int main(int argc, char **argv){
	if(argc < 2)
		return -1;
	
	// Get serial port String
	serial_port_ = std::string(argv[1]);
	// std::cout << serial_port_;
	
	char *serial_port_file = new char[serial_port_.length() + 1];
	strcpy(serial_port_file, serial_port_.c_str());
	serial_port = open(serial_port_file, O_RDWR);
	// Can't open serial port
	if(serial_port < -1){
		std::cout << "can't open serial port!";
		std::raise(SIGTERM);
		return -1;
	}
	
	if(tcgetattr(serial_port, &tty) != 0){
		std::cout << "tcgetattr error!";
		std::raise(SIGTERM);
		return -1;			
	}
	
	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
	tty.c_cflag |= CS8; // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

	tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	// Set in/out baud rate to be 115200
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);
	
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
		std::cout << "tcsetattr error!";
		std::raise(SIGTERM);
		return -1;		
	}
		
	tcflush(serial_port,TCIOFLUSH);// Flush serial buffer before start
	
	// Write CMD ID 
	hokuyo_cmdID();
	
	while(rx_accumu_bytes < 943){
		rx_accumu_bytes +=
			hokuyo_getRxBytes();
		delay_count++;
		usleep(10000);// Delay for 10ms
		if(delay_count > 100)// 100ms timeout exceed
			return -1;
	}
	
	char *ident_str = new char[rx_bytes + 1];
	ident_str[rx_bytes] = '\0';
	
	hokuyo_readBytes(
		ident_str,
		rx_bytes
	);
	
	find_ident = std::string(ident_str);
	
	serial_num_begin =
		find_ident.find("serial_number,5=");
	if(serial_num_begin == std::string::npos)
		return 0;
	serial_num_end = 
		find_ident.find(":f", serial_num_begin);
	if(serial_num_end == std::string::npos)
		return 0;
	
	find_ident = 
		find_ident.substr(
			serial_num_begin + 16,
			8
		);
	
	std::cout << find_ident;
	
	return 0;
}
