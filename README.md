<p align="center"><strong>Vulcan Serial</strong></p>
<p align="center"><a href="https://github.com/Vulcan-YJX/VulcanSerial/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-MIT-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>


# VulcanSerial
​	Linux serial port library written in C++.

## Description 项目描述

Library for communicating with COM ports on a Linux system.

* Simple API
* Supports custom baud rates
* `cmake` based build system.

# Quick Start 快速开始

```bash
#国内可能git可以使用代理ghproxy.com进行下载
#$ git clone https://ghproxy.com/https://github.com/Vulcan-YJX/VulcanSerial.git
$ git clone https://github.com/Vulcan-YJX/VulcanSerial.git
$ cd VulcanSerial
$ mkdir build && cd build
$ cmake ..
$ make 
$ sudo make install
```

## Examples 测试案例

```c++
// fileName: main.cpp

#include "CppLinuxSerial/SerialPort.hpp"
#include <iostream>

using namespace VulcanSerial;

int main() {
	// Create serial port object and open serial port at 57600 buad, 8 data bits, no parity bit, and one stop bit (8n1)
	SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
	serialPort.Open();
    while(1){
        while(Available() > 0){
            // Write some ASCII data
            serialPort.Write("Vulcan Serial");

            // Read some data back (will block until at least 1 byte is received due to the SetTimeout(-1) call above)
            std::string readData;
            serialPort.Read(readData);
            std::cout << readData << std::endl;
        }
    
    }
	// Close the serial port
	serialPort.Close();
}
```

Make sure you have install `VulcanSerial` lib.

```bash
g++ main.cpp -lCppLinuxSerial
```

