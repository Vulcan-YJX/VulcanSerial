// fileName: main.cpp

#include "VulcanSerial/SerialPort.hpp"
#include <iostream>

using namespace VulcanSerial;

int main() {

	SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
	serialPort.Open(timeout_ms = 0); 

	while(1){
		while(serialPort.Available() > 0){
		    // Write some ASCII data
		    serialPort.Write("Vulcan Serial");

		    std::string readData;
		    serialPort.Read(readData);
		    std::cout << readData << std::endl;
		}

	}
	// Close the serial port
	serialPort.Close();
}
