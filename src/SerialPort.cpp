//!
//! @file 			SerialPort.cpp
//! @author 		Vulcan YJX <vulcanai@163.com> 
//! @created		2022-06-07
//! @last-modified 	2022-06-07
//! @brief			The main serial port class.

// System includes
#include <iostream>
#include <sstream>
#include <stdio.h>   	// Standard input/output definitions
#include <string.h>  	// String function definitions
#include <unistd.h>  	// UNIX standard function definitions
#include <fcntl.h>   	// File control definitions
#include <errno.h>   	// Error number definitions

#include <system_error>	// For throwing std::system_error
#include <sys/ioctl.h> // Used for TCGETS2, which is required for custom baud rates
#include <cassert>

#include <asm/termbits.h>
#include <algorithm>
#include <iterator>

// User includes
#include "VulcanSerial/Exception.hpp"
#include "VulcanSerial/SerialPort.hpp"



namespace VulcanSerial {

	SerialPort::SerialPort() {
        echo_ = false;
        timeout_ms_ = defaultTimeout_ms_;
        baudRateStandard_ = defaultBaudRate_;
        readBufferSize_B_ = defaultReadBufferSize_B_;
        readBuffer_.reserve(readBufferSize_B_);
		state_ = State::CLOSED;
	}

	SerialPort::SerialPort(const std::string& device, speed_t baudRate) :
            SerialPort() {
		device_ = device;
        baudRateStandard_ = baudRate;
	}


	SerialPort::SerialPort(const std::string& device, speed_t baudRate, NumDataBits numDataBits, Parity parity, NumStopBits numStopBits) :
            SerialPort() {
		device_ = device;
        baudRateStandard_ = baudRate;
		numDataBits_ = numDataBits;
		parity_ = parity;
		numStopBits_ = numStopBits;
	}

	SerialPort::~SerialPort() {
        try {
            Close();
        } catch(...) {
            // We can't do anything about this!
            // But we don't want to throw within destructor, so swallow
        }
	}

	void SerialPort::SetDevice(const std::string& device) {
		device_ = device;
        if(state_ == State::OPEN)
        	ConfigureTermios();
	}


	void SerialPort::SetBaudRate(speed_t baudRate)	{		
		baudRateStandard_ = baudRate;
        if(state_ == State::OPEN)
            ConfigureTermios();
	}

	void SerialPort::SetNumDataBits(NumDataBits numDataBits) {
		numDataBits_ = numDataBits;
		if(state_ == State::OPEN)
            ConfigureTermios();
	}

	void SerialPort::SetParity(Parity parity) {
		parity_ = parity;
		if(state_ == State::OPEN)
            ConfigureTermios();
	}

	void SerialPort::SetNumStopBits(NumStopBits numStopBits) {
		numStopBits_ = numStopBits;
		if(state_ == State::OPEN)
            ConfigureTermios();
	}

	void SerialPort::Open()
	{

		if(device_.empty()) {
			THROW_EXCEPT("Attempted to open file when file path has not been assigned to.");
		}

		// O_RDONLY for read-only, O_WRONLY for write only, O_RDWR for both read/write access
		// 3rd, optional parameter is mode_t mode
		fileDesc_ = open(device_.c_str(), O_RDWR);

		// Check status
		if(fileDesc_ == -1) {
		    THROW_EXCEPT("Could not open device " + device_ + ". Is the device name correct and do you have read/write permission?");
		}

        ConfigureTermios();

		// std::cout << "COM port opened successfully." << std::endl;
        state_ = State::OPEN;
	}

	void SerialPort::SetEcho(bool value) {
        echo_ = value;
        ConfigureTermios();
	}

	void SerialPort::ConfigureTermios()
	{
		// std::cout << "Configuring COM port \"" << device_ << "\"." << std::endl;

		//================== CONFIGURE ==================//

		// termios tty = GetTermios();
		termios2 tty = GetTermios2();

		//================= (.c_cflag) ===============//

		// Set num. data bits
		// See https://man7.org/linux/man-pages/man3/tcflush.3.html
		tty.c_cflag     &=  ~CSIZE;			// CSIZE is a mask for the number of bits per character
		switch(numDataBits_) {
			case NumDataBits::FIVE:
				tty.c_cflag     |=  CS5;
				break;
			case NumDataBits::SIX:
				tty.c_cflag     |=  CS6;
				break;
			case NumDataBits::SEVEN:
				tty.c_cflag     |=  CS7;
				break;
			case NumDataBits::EIGHT:
				tty.c_cflag     |=  CS8;
				break;
			default:
				THROW_EXCEPT("numDataBits_ value not supported!");
		}
		
		// Set parity
		// See https://man7.org/linux/man-pages/man3/tcflush.3.html
		switch(parity_) {
			case Parity::NONE:
				tty.c_cflag     &=  ~PARENB;
				break;
			case Parity::EVEN:	
				tty.c_cflag 	|=   PARENB;
				tty.c_cflag		&=	 ~PARODD; // Clearing PARODD makes the parity even
				break;
			case Parity::ODD:
				tty.c_cflag     |=   PARENB;
				tty.c_cflag		|=	 PARODD;
				break;
			default:
				THROW_EXCEPT("parity_ value not supported!");

		}

		// Set num. stop bits
		switch(numStopBits_) {
			case NumStopBits::ONE:
				tty.c_cflag     &=  ~CSTOPB;
				break;
			case NumStopBits::TWO:
				tty.c_cflag     |=  CSTOPB;
				break;
			default:
				THROW_EXCEPT("numStopBits_ value not supported!");
		}

		tty.c_cflag     &=  ~CRTSCTS;       // Disable hadrware flow control (RTS/CTS)
		tty.c_cflag     |=  CREAD | CLOCAL;     				// Turn on READ & ignore ctrl lines (CLOCAL = 1)


        //===================== BAUD RATE =================//

        tty.c_ispeed = baudRateStandard_;
        tty.c_ospeed = baudRateStandard_;

		//===================== (.c_oflag) =================//

		tty.c_oflag     =   0;              // No remapping, no delays
		tty.c_oflag     &=  ~OPOST;			// Make raw

		//================= CONTROL CHARACTERS (.c_cc[]) ==================//

        if(timeout_ms_ == -1) {
            tty.c_cc[VTIME] = 0;
            tty.c_cc[VMIN] = 1;
        } else if(timeout_ms_ == 0) {
            // Setting both to 0 will give a non-blocking read
            tty.c_cc[VTIME] = 0;
            tty.c_cc[VMIN] = 0;
        } else if(timeout_ms_ > 0) {
            tty.c_cc[VTIME] = (cc_t)(timeout_ms_/100);    // 0.5 seconds read timeout
            tty.c_cc[VMIN] = 0;
        }

		//======================== (.c_iflag) ====================//

		tty.c_iflag     &= ~(IXON | IXOFF | IXANY);			// Turn off s/w flow ctrl
		tty.c_iflag 	&= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

		//=========================== LOCAL MODES (c_lflag) =======================//

		// Canonical input is when read waits for EOL or EOF characters before returning. In non-canonical mode, the rate at which
		// read() returns is instead controlled by c_cc[VMIN] and c_cc[VTIME]
		tty.c_lflag		&= ~ICANON;								// Turn off canonical input, which is suitable for pass-through
		// Configure echo depending on echo_ boolean
        if(echo_) {
			tty.c_lflag |= ECHO;
		} else {
			tty.c_lflag &= ~(ECHO);
		}
		tty.c_lflag		&= ~ECHOE;								// Turn off echo erase (echo erase only relevant if canonical input is active)
		tty.c_lflag		&= ~ECHONL;								//
		tty.c_lflag		&= ~ISIG;								// Disables recognition of INTR (interrupt), QUIT and SUSP (suspend) characters

		this->SetTermios2(tty);

	}

	void SerialPort::Write(const std::string& data) {

        if(state_ != State::OPEN)
            THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but state != OPEN. Please call Open() first.");

		if(fileDesc_ < 0) {
			THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but file descriptor < 0, indicating file has not been opened.");
		}

		int writeResult = write(fileDesc_, data.c_str(), data.size());

		// Check status
		if (writeResult == -1) {
			throw std::system_error(EFAULT, std::system_category());
		}
	}

	
    int SerialPort::Write(const std::string& data, uint16_t size) {

        if(state_ != State::OPEN)
            THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but state != OPEN. Please call Open() first.");

		if(fileDesc_ < 0) {
			THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but file descriptor < 0, indicating file has not been opened.");
		}

		int writeResult = write(fileDesc_, data.c_str(), size);

        return writeResult;
	}
    
    void SerialPort::WriteChar (const unsigned char c){
        if(state_ != State::OPEN)
            THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but state != OPEN. Please call Open() first.");

		if(fileDesc_ < 0) {
			THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but file descriptor < 0, indicating file has not been opened.");
		}
        int writeResult  = write (fileDesc_, &c, 1) ;
        		// Check status
		if (writeResult == -1) {
			throw std::system_error(EFAULT, std::system_category());
		}
    }

    void SerialPort::WriteBinary(const std::vector<uint8_t>& data) {

        if(state_ != State::OPEN)
            THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but state != OPEN. Please call Open() first.");

		if(fileDesc_ < 0) {
			THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but file descriptor < 0, indicating file has not been opened.");
		}

		int writeResult = write(fileDesc_, data.data(), data.size());

		// Check status
		if (writeResult == -1) {
			throw std::system_error(EFAULT, std::system_category());
		}
	}

	void SerialPort::Read(std::string& data)
	{
        data.clear();

		if(fileDesc_ == 0) {
			THROW_EXCEPT("Read() was called but file descriptor (fileDesc) was 0, indicating file has not been opened.");
		}

		ssize_t n = read(fileDesc_, &readBuffer_[0], readBufferSize_B_);

		// Error Handling
		if(n < 0) {
			// Read was unsuccessful
			throw std::system_error(EFAULT, std::system_category());
		}

		if(n > 0) {

            data = std::string(&readBuffer_[0], n);
			//std::cout << *str << " and size of string =" << str->size() << "\r\n";
		}

		// If code reaches here, read must of been successful
	}
    
    
    int SerialPort::ReadChar()
    {
        uint8_t charBuf ;

        if (read(fileDesc_, &charBuf, 1) != 1){
            return -1 ;
        }
        
        return ((int)charBuf) & 0xFF ;
    }
        
	void SerialPort::ReadBinary(std::vector<uint8_t>& data)
	{
        data.clear();

		if(fileDesc_ == 0) {
        
			THROW_EXCEPT("Read() was called but file descriptor (fileDesc) was 0, indicating file has not been opened.");
		}

		// Read from file
        // We provide the underlying raw array from the readBuffer_ vector to this C api.
        // This will work because we do not delete/resize the vector while this method
        // is called
		ssize_t n = read(fileDesc_, &readBuffer_[0], readBufferSize_B_);

		// Error Handling
		if(n < 0) {
			// Read was unsuccessful
			throw std::system_error(EFAULT, std::system_category());
		}

		if(n > 0) {
            copy(readBuffer_.begin(), readBuffer_.begin() + n, back_inserter(data));
		}

		// If code reaches here, read must of been successful
	}


	termios2 SerialPort::GetTermios2()
	{
		struct termios2 term2;

        ioctl(fileDesc_, TCGETS2, &term2);

		return term2;
	}

	void SerialPort::SetTermios2(termios2 tty)
	{
		ioctl(fileDesc_, TCSETS2, &tty);
	}

    void SerialPort::Close() {
        if(fileDesc_ != -1) {
            auto retVal = close(fileDesc_);
            if(retVal != 0)
                THROW_EXCEPT("Tried to close serial port " + device_ + ", but close() failed.");

            fileDesc_ = -1;
        }

        state_ = State::CLOSED;
    }

    void SerialPort::SetTimeout(int32_t timeout_ms) {
        if(timeout_ms < -1)
            THROW_EXCEPT(std::string() + "timeout_ms provided to " + __PRETTY_FUNCTION__ + " was < -1, which is invalid.");
        if(timeout_ms > 25500)
            THROW_EXCEPT(std::string() + "timeout_ms provided to " + __PRETTY_FUNCTION__ + " was > 25500, which is invalid.");
        if(state_ == State::OPEN)
            THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called while state == OPEN.");
        timeout_ms_ = timeout_ms;
    }
    
    int32_t SerialPort::Available() {
		if(state_ != State::OPEN)
            THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but state != OPEN. Please call Open() first.");
        int32_t ret = 0;
        ioctl(fileDesc_, FIONREAD, &ret);
        return ret;
        
    }
    
    State SerialPort::GetState() {
      return state_;
    }

} // namespace VulcanSerial

