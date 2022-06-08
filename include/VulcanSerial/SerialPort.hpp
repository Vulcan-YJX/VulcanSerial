///
//! @file 			SerialPort.hpp
//! @author 		Vulcan YJX <vulcanai@163.com> 
//! @created		2022-06-07
//! @last-modified 	2022-06-07
//! @brief			The main serial port class.

// Header guard
#ifndef SERIAL_PORT_SERIAL_PORT_H
#define SERIAL_PORT_SERIAL_PORT_H

// System headers
#include <string>
#include <fstream> // For file I/O (reading/writing to COM port)
#include <sstream>

#include <vector>
#include <asm/ioctls.h>
#include <asm/termbits.h>

// User headers
#include "Exception.hpp"


namespace VulcanSerial {

        /// \brief      Enumeration of all the valid num. of data bits. Must align with the options 
        ///                 provided in termbits.h, i.e. CS5, CS6, CS7 and CS8.
        enum class NumDataBits {
            FIVE,
            SIX,
            SEVEN,
            EIGHT,
        };

        enum class Parity {
            NONE,
            EVEN,
            ODD,
        };

        enum class NumStopBits {
            ONE,
            TWO,
        };

        /// \brief      Represents the state of the serial port.
        enum class State {
            CLOSED,
            OPEN,
        };

/// \brief		SerialPort object is used to perform rx/tx serial communication.
        class SerialPort {

        public:
            /// \brief		Default constructor. You must specify at least the device before calling Open().
            SerialPort();

            /// \brief		Constructor that sets up serial port with the basic (required) parameters.
            SerialPort(const std::string &device, speed_t baudRate);

            /// \brief		Constructor that sets up serial port and allows the user to specify all the common parameters.
            SerialPort(const std::string &device, speed_t baudRate, NumDataBits numDataBits, Parity parity, NumStopBits numStopBits);

            /// \brief		Destructor. Closes serial port if still open.
            virtual ~SerialPort();

            /// \brief		Sets the device to use for serial port communications.
            /// \details    Method can be called when serial port is in any state.
            void SetDevice(const std::string &device);

            /// \brief      Call this to set a standard baud rate.
            void SetBaudRate(speed_t baudRate);

            /// \brief      Call this to set the num. of data bits.
            void SetNumDataBits(NumDataBits numDataBits);

            /// \brief      Call this to set the parity.
            void SetParity(Parity parity);

            void SetNumStopBits(NumStopBits numStopBits);

            /// \brief      Sets the read timeout (in milliseconds)/blocking mode.
            /// \details    Only call when state != OPEN. This method manupulates VMIN and VTIME.
            /// \param      timeout_ms  Set to -1 to infinite timeout, 0 to return immediately with any data (non
            ///             blocking, or >0 to wait for data for a specified number of milliseconds). Timeout will
            ///             be rounded to the nearest 100ms (a Linux API restriction). Maximum value limited to
            ///             25500ms (another Linux API restriction).
            void SetTimeout(int32_t timeout_ms);
            

            /// \brief		Enables/disables echo.
            /// \param		value		Pass in true to enable echo, false to disable echo.
            void SetEcho(bool value);

            /// \brief		Opens the COM port for use.
            /// \throws		CppLinuxSerial::Exception if device cannot be opened.
            /// \note		Must call this before you can configure the COM port.
            void Open();

            /// \brief		Closes the COM port.
            void Close();

            /// \brief		Sends a text message over the com port.
            /// \param		data		The data that will be written to the COM port.
            /// \throws		CppLinuxSerial::Exception if state != OPEN.
            void Write(const std::string& data);
            
            /// \brief		Sends a text message over the com port.
            /// \param		data		The data that will be written to the COM port.
            /// \throws		CppLinuxSerial::Exception if state != OPEN.
            /// \return     Read error return -1.
            int Write(const std::string& data, uint16_t size);
            
            /// \brief		Sends a text message over the com port.
            /// \throws		CppLinuxSerial::Exception if state != OPEN.
            void WriteChar (const unsigned char c);
            
            /// \brief		Sends a binary message over the com port.
            /// \param		data		The data that will be written to the COM port.
            /// \throws		CppLinuxSerial::Exception if state != OPEN.
            void WriteBinary(const std::vector<uint8_t>& data);

            /// \brief		Use to read text from the COM port.
            /// \param		data		The object the read characters from the COM port will be saved to.
            /// \param      wait_ms     The amount of time to wait for data. Set to 0 for non-blocking mode. Set to -1
            ///                 to wait indefinitely for new data.
            /// \throws		CppLinuxSerial::Exception if state != OPEN.
            void Read(std::string& data);

            /// \brief		Use to read char from the COM port.
            int ReadChar();
            
            /// \brief		Use to read binary data from the COM port.
            /// \param		data		The object the read uint8_t bytes from the COM port will be saved to.
            /// \param      wait_ms     The amount of time to wait for data. Set to 0 for non-blocking mode. Set to -1
            ///                 to wait indefinitely for new data.
            /// \throws		CppLinuxSerial::Exception if state != OPEN.
            void ReadBinary(std::vector<uint8_t>& data);

			/// \brief		Use to get number of bytes available in receive buffer.
            /// \returns    The number of bytes available in the receive buffer (ready to be read).
            /// \throws		CppLinuxSerial::Exception if state != OPEN.
            int32_t Available();

            /// \brief          Use to get the state of the serial port
            /// \returns        The state of the serial port
            State GetState();
            
            void Flush();

        private:

            /// \brief		Configures the tty device as a serial port.
            /// \warning    Device must be open (valid file descriptor) when this is called.
            void ConfigureTermios();

            // void SetTermios(termios myTermios);

            /// \brief		Returns a populated termios2 structure for the serial port pointed to by the file descriptor.
            termios2 GetTermios2();

            /// \brief      Assigns the provided tty settings to the serial port pointed to by the file descriptor.
            void SetTermios2(termios2 tty);

            /// \brief      Keeps track of the serial port's state.
            State state_;

            /// \brief      The file path to the serial port device (e.g. "/dev/ttyUSB0").
            std::string device_;

            /// \brief      The current baud rate if baudRateType_ == STANDARD.
            speed_t baudRateStandard_;

            /// \brief      The num. of data bits. Defaults to 8 (most common).
            NumDataBits numDataBits_ = NumDataBits::EIGHT;

            /// \brief      The parity. Defaults to none (most common).
            Parity parity_ = Parity::NONE;

            /// \brief      The num. of stop bits. Defaults to 1 (most common).
            NumStopBits numStopBits_ = NumStopBits::ONE;

            /// \brief		The file descriptor for the open file. This gets written to when Open() is called.
            int fileDesc_;

            bool echo_;

            int32_t timeout_ms_;

            std::vector<char> readBuffer_;
            unsigned char readBufferSize_B_;

            static constexpr uint32_t defaultBaudRate_ = 115200;
            static constexpr int32_t defaultTimeout_ms_ = -1;
            static constexpr unsigned char defaultReadBufferSize_B_ = 255;


        };

} // namespace VulcanSerial


#endif // #ifndef SERIAL_PORT_SERIAL_PORT_H
