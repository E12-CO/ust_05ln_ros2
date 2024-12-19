// ROS2 driver for Hokuyo UST-05LN
// By TinLethax at Robot Club KMITL (RB26)

#include <chrono>
#include <cmath>
#include <string>
#include <iostream>
#include <stdexcept>

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h> 

struct termios tty;

// ROS2 library
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#define SERIAL_TIME_MIL	10 // around 80Hz to ping pong fifo the scan data
#define PUB_TIME_MIL	25 // around 40Hz

#define HOKUYO_CMD_ID				"#IN0D54\n"
#define HOKUYO_CMD_PD				"#PD15F5\n"
#define HOKUYO_CMD_START_SCAN		"#GT15466\n"
#define HOKUYO_CMD_STOP_SCAN		"#ST5297\n"

#define HOKUYO_SCAN_LEN			4359

#define SERIAL_MAX_LEN		4095

class ust_05ln_if : public rclcpp::Node{
	
	public:
	
	// LiDAR publisher
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr 	pubLaserScan;
	// LiDAR message buffer
	sensor_msgs::msg::LaserScan			LaserMsg;
	
	// Used in wall timer callback
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr serialLoopTimer_;
	
	// Serial port name '/dev/blablabla'
	std::string serial_port_;
	
	// Scanner frame id
	std::string laser_frame_id;
	
	// Scanner topic name
	std::string laser_topic;
	
	// Laser angle offset (in radiant)
	float angle_offset;

	// Serial port file descriptor
	int serial_port = 0;
	
	// Laser FSM
	uint8_t laser_fsm = 0;
	
	// Receiver counter
	int rx_bytes = 0;
	int rx_accumu = 0;
	int laserData_accumu 	= 0;
	int laserData_offset	= 0;
	int laserData_remains	= 0;
	
	// Scan buffer
	bool inSync = false;
	
	char *scan_buffer;
	std::string scan_buffer_str;
	std::string laserscan_buffer;
	std::size_t laserLastHeader;

	// Laser data in CPP string
	std::string laser_dataOut;
	std::string laser_dataOld;
	std::string sub_range;
	std::string sub_intens;
	
	ust_05ln_if() : Node("urg_node"){
		RCLCPP_INFO(
			this->get_logger(), 
			"Robot Club KMITL : Starting UST-05LN LiDAR node..."
			);
		
		declare_parameter("serial_port", "/dev/hokuyo");
		get_parameter("serial_port", serial_port_);
		
		declare_parameter("laser_frame_id", "laser_frame");
		get_parameter("laser_frame_id", laser_frame_id);
		
		declare_parameter("laser_topic", "/scan");
		get_parameter("laser_topic", laser_topic);
		
		declare_parameter("angle_offset", -1.22173f);
		get_parameter("angle_offset", angle_offset);
		
		
		char *serial_port_file = new char[serial_port_.length() + 1];
		strcpy(serial_port_file, serial_port_.c_str());
		serial_port = open(serial_port_file, O_RDWR);
		// Can't open serial port
		if(serial_port < -1){
			RCLCPP_ERROR(
				this->get_logger(), 
				"Error openning Serial %s", 
				serial_port_.c_str()
				);
			std::raise(SIGTERM);
			return;
		}
		
		if(tcgetattr(serial_port, &tty) != 0){
			RCLCPP_ERROR(
				this->get_logger(), 
				"Error %i from tcgetattr: %s\n", 
				errno, 
				strerror(errno)
				);
			std::raise(SIGTERM);
			return;			
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
			RCLCPP_ERROR(
				this->get_logger(),
				"Error %i from tcsetattr: %s\n", 
				errno, 
				strerror(errno));
			std::raise(SIGTERM);
			return;
		}
		
		tcflush(serial_port,TCIOFLUSH);// Flush serial buffer before start
		
		// Laser Publisher 
		pubLaserScan =
			create_publisher<sensor_msgs::msg::LaserScan>(
				laser_topic,
				rclcpp::QoS(rclcpp::SensorDataQoS())
			);
		
		// Set up laser message parameter
		// Allocate buffer extra byte for null character
		scan_buffer = new char [SERIAL_MAX_LEN+1];
		scan_buffer[SERIAL_MAX_LEN] = '\0';
		
		LaserMsg.header.frame_id 	= laser_frame_id;
		LaserMsg.angle_min 			= -1.178097245f + angle_offset;
		LaserMsg.angle_max			= 1.178097245f + angle_offset;
		LaserMsg.angle_increment	= 0.008726646f; // 4.712389rad / (541 -1)
		LaserMsg.scan_time			= (1 / 40.0f);
		LaserMsg.time_increment		= (1 / 40.0f) / 541.0f;
		LaserMsg.ranges.resize(541);
		LaserMsg.intensities.resize(541);
		
		LaserMsg.range_min			= 0.0f;
		LaserMsg.range_max			= 5.0f;
		
		// Timer callback
		timer_ = 
			this->create_wall_timer(
				std::chrono::milliseconds(PUB_TIME_MIL),
				std::bind(
					&ust_05ln_if::hokuyo_fsm, 
					this)
			);
		
		// Serial loop timer
		serialLoopTimer_ =
			this->create_wall_timer(
				std::chrono::milliseconds(SERIAL_TIME_MIL),
				std::bind(
					&ust_05ln_if::hokuyo_serialRunner, 
					this)
			);
	}
	
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
		
		RCLCPP_DEBUG(
			this->get_logger(), 
			"getRxBytes : %d", rx_bytes
			);
		
		return rx_bytes;
	}
	
	bool hokuyo_checkRxEqual(int expected_rx){
		return (hokuyo_getRxBytes() >= expected_rx) ? true : false;
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
	
	void hokuyo_cmdPD(){
		hokuyo_writeCmd(HOKUYO_CMD_PD);
	}
	
	void hokuyo_cmdStopScan(){
		hokuyo_writeCmd(HOKUYO_CMD_STOP_SCAN);
	}
	
	// Fast circular serial fifo
	void hokuyo_serialRunner(){
		// Only run fast FIFO when scanner is started
		if(laser_fsm != 4)
			return;
		
		ioctl(
			serial_port,
			FIONREAD,
			&rx_bytes
			);
			
		read(
			serial_port,
			scan_buffer,
			rx_bytes
			);

		RCLCPP_DEBUG(
			this->get_logger(), 
			"Received %d bytes", 
			rx_bytes
			);
		
		scan_buffer_str = std::string(scan_buffer);
		
		// Trying to sync with the first scan data
		if(inSync == false){
			laserLastHeader = scan_buffer_str.find("#G");
			if(laserLastHeader != std::string::npos){
				inSync = true;
				laserData_offset 	= laserLastHeader;
				laserData_remains 	= rx_bytes - laserLastHeader;
				laserData_accumu 	+= laserData_remains;
			}
		}else{
			laserData_offset 	= 0;
			laserData_remains 	= HOKUYO_SCAN_LEN - laserData_accumu;
			
			// Capping if remains data is larger than serial FIFO size
			if(laserData_remains > SERIAL_MAX_LEN)
				laserData_remains = SERIAL_MAX_LEN;
			
			laserData_accumu 	+= laserData_remains;
		}
		
		// remaining of sencond round -> rx_byte 
		if(inSync == true){
			
			laserscan_buffer += 
				scan_buffer_str.substr(
					laserData_offset,
					laserData_remains
				);
				
			if(laserData_accumu >= HOKUYO_SCAN_LEN){	
				
				// Save current scan data
				laser_dataOut = 
					laserscan_buffer.substr(
						0, 
						HOKUYO_SCAN_LEN
					);
					
				RCLCPP_DEBUG(
					this->get_logger(), 
					"Received %ld bytes long complete scan data\n%s<-end",
					laser_dataOut.length(),
					laser_dataOut.c_str()
				);	
				
				// look for next scan data (if any)
				if(laserscan_buffer.length() > HOKUYO_SCAN_LEN){
					// In the case of having next scan data in the buffer
					// Just cut the next scan data and override the old scan data
					laserscan_buffer = laserscan_buffer.substr(
						HOKUYO_SCAN_LEN
						);
						
					laserData_accumu = laserscan_buffer.length();
					// RCLCPP_DEBUG(
						// this->get_logger(),
						// "Next scan data %s",
						// laserscan_buffer.c_str()
					// );
				}else{
					// In the case of having complete single scan data,
					// just resync the header to get next bytes
					laserData_accumu = 0;// reset data count accumulator for next scan data
					inSync = false;
				}
				
				// Publish the currently completed scan data
				hokuyo_publisher();
			}	
			
		}
	}
	
	void hokuyo_publisher(){
		// If data is less than the HOKUYO_SCAN_LEN, wait for more
		if(laser_dataOut.length() < (HOKUYO_SCAN_LEN - 1)){
			RCLCPP_DEBUG(
				this->get_logger(),
				"laser data size too small %ld",
				laser_dataOut.size()
			);
			return;
		}
		
		laser_dataOut = laser_dataOut.substr(
						26
					);
		
		if(
			(laser_dataOut.find('#') != std::string::npos) ||
			(laser_dataOut.find('G') != std::string::npos) ||
			(laser_dataOut.find('T') != std::string::npos) ||
			(laser_dataOut.find(':') != std::string::npos)
		)
			laser_dataOut = laser_dataOld;
		// RCLCPP_DEBUG(
			// this->get_logger(),
			// "laser data %s",
			// laser_dataOut.c_str()
		// );			

		for(uint16_t i = 0; i < 541; i++){
			sub_range 	= laser_dataOut.substr(i*8, 4);// Get 4 chars from string
			sub_intens 	= laser_dataOut.substr((i*8)+4, 4);
			
			try{
			LaserMsg.ranges[i]		=  
				std::stoi(sub_range, nullptr, 16) * 0.001f;
			LaserMsg.intensities[i]	=  
				std::stoi(sub_intens, nullptr, 16) * 0.00001f;
		
			
			} catch(...){
				RCLCPP_WARN(
					this->get_logger(),
					"stoi Exception! : non number character detected %s,%s at %d",
					sub_range.c_str(), sub_intens.c_str(), i
				);
				
				RCLCPP_WARN(
					this->get_logger(),
					"In this data %s",
					laser_dataOut.c_str()
				);
				return;
			}
		}
		
		laser_dataOld = laser_dataOut;
		
		LaserMsg.header.stamp = this->get_clock()->now();
		pubLaserScan->publish(LaserMsg);
		
	}
	
	void hokuyo_fsm(){
		
		switch(laser_fsm){
			case 0:// Stop sensor
			{
				RCLCPP_INFO(
					this->get_logger(), 
					"Stopping scanner..."
					);
					
				hokuyo_cmdStopScan();
				laser_fsm = 1;
			}
			break;
			
			case 1:// ID query
			{
				if(!hokuyo_checkRxEqual(10))// return of stop scan
					return;
				
				hokuyo_flushSerial();
				
				RCLCPP_INFO(
					this->get_logger(), 
					"Scanner ID query..."
					);
				
				hokuyo_cmdID();
				laser_fsm = 2;
			}
			break;
			
			case 2:// PD query
			{
				if(!hokuyo_checkRxEqual(994))// return of ID query
					return;
				
				hokuyo_flushSerial();
				
				RCLCPP_INFO(
					this->get_logger(), 
					"Scanner PD query..."
					);
				
				hokuyo_cmdPD();
				laser_fsm = 3;
			}
			break;
			
			case 3:// Start scan command
			{
				if(!hokuyo_checkRxEqual(10))// return of PD query "#PD0070FA\n"
					return;

				RCLCPP_INFO(this->get_logger(), "Hokuyo started!");		
				hokuyo_flushSerial();
				
				hokuyo_writeCmd(HOKUYO_CMD_START_SCAN);
				laser_fsm = 4;
			}
			break;
			
			case 4:// Get Scan AB
			{
				
			}
			break;
			
		}
	}
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	auto ust_if {std::make_shared<ust_05ln_if>()};
	rclcpp::spin(ust_if);
	ust_if->hokuyo_cmdStopScan();// Stop sensor before exit
	close(ust_if->serial_port);
	rclcpp::shutdown();
}