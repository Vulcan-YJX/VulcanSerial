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

#include "VulcanSerial/SerialPort.hpp"
#include <iostream>

using namespace VulcanSerial;

int main() {

	SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
	serialPort.Open(0); 	//set timeout 0, Open file as Non blocking, >0 as block
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
```

Make sure you have install `VulcanSerial` lib.

```bash
g++ main.cpp -lVulcanSerial
```

