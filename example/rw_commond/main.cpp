#include "VulcanSerial/SerialPort.hpp"

using namespace VulcanSerial;


int main() {
	// Create serial port object and open serial port at 57600 buad, 8 data bits, no parity bit, and one stop bit (8n1)
	SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_460800, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
	// Use SerialPort serialPort("/dev/ttyACM0", 13000); instead if you want to provide a custom baud rate
	serialPort.SetTimeout(-1); // Block when reading until any data is received
	serialPort.Open();
    
	// Write some ASCII data
	serialPort.Write("Hello");

	// Read some data back (will block until at least 1 byte is received due to the SetTimeout(-1) call above)
	std::string readData;
	serialPort.Read(readData);

	// Close the serial port
	serialPort.Close();
}

