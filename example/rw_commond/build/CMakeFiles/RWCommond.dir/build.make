# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vulcan/projects/VulcanSerial/example/rw_commond

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vulcan/projects/VulcanSerial/example/rw_commond/build

# Include any dependencies generated for this target.
include CMakeFiles/RWCommond.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/RWCommond.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RWCommond.dir/flags.make

CMakeFiles/RWCommond.dir/main.cpp.o: CMakeFiles/RWCommond.dir/flags.make
CMakeFiles/RWCommond.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vulcan/projects/VulcanSerial/example/rw_commond/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/RWCommond.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RWCommond.dir/main.cpp.o -c /home/vulcan/projects/VulcanSerial/example/rw_commond/main.cpp

CMakeFiles/RWCommond.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RWCommond.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vulcan/projects/VulcanSerial/example/rw_commond/main.cpp > CMakeFiles/RWCommond.dir/main.cpp.i

CMakeFiles/RWCommond.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RWCommond.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vulcan/projects/VulcanSerial/example/rw_commond/main.cpp -o CMakeFiles/RWCommond.dir/main.cpp.s

CMakeFiles/RWCommond.dir/home/vulcan/projects/VulcanSerial/src/SerialPort.cpp.o: CMakeFiles/RWCommond.dir/flags.make
CMakeFiles/RWCommond.dir/home/vulcan/projects/VulcanSerial/src/SerialPort.cpp.o: /home/vulcan/projects/VulcanSerial/src/SerialPort.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vulcan/projects/VulcanSerial/example/rw_commond/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/RWCommond.dir/home/vulcan/projects/VulcanSerial/src/SerialPort.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RWCommond.dir/home/vulcan/projects/VulcanSerial/src/SerialPort.cpp.o -c /home/vulcan/projects/VulcanSerial/src/SerialPort.cpp

CMakeFiles/RWCommond.dir/home/vulcan/projects/VulcanSerial/src/SerialPort.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RWCommond.dir/home/vulcan/projects/VulcanSerial/src/SerialPort.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vulcan/projects/VulcanSerial/src/SerialPort.cpp > CMakeFiles/RWCommond.dir/home/vulcan/projects/VulcanSerial/src/SerialPort.cpp.i

CMakeFiles/RWCommond.dir/home/vulcan/projects/VulcanSerial/src/SerialPort.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RWCommond.dir/home/vulcan/projects/VulcanSerial/src/SerialPort.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vulcan/projects/VulcanSerial/src/SerialPort.cpp -o CMakeFiles/RWCommond.dir/home/vulcan/projects/VulcanSerial/src/SerialPort.cpp.s

# Object files for target RWCommond
RWCommond_OBJECTS = \
"CMakeFiles/RWCommond.dir/main.cpp.o" \
"CMakeFiles/RWCommond.dir/home/vulcan/projects/VulcanSerial/src/SerialPort.cpp.o"

# External object files for target RWCommond
RWCommond_EXTERNAL_OBJECTS =

RWCommond: CMakeFiles/RWCommond.dir/main.cpp.o
RWCommond: CMakeFiles/RWCommond.dir/home/vulcan/projects/VulcanSerial/src/SerialPort.cpp.o
RWCommond: CMakeFiles/RWCommond.dir/build.make
RWCommond: CMakeFiles/RWCommond.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vulcan/projects/VulcanSerial/example/rw_commond/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable RWCommond"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RWCommond.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RWCommond.dir/build: RWCommond

.PHONY : CMakeFiles/RWCommond.dir/build

CMakeFiles/RWCommond.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RWCommond.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RWCommond.dir/clean

CMakeFiles/RWCommond.dir/depend:
	cd /home/vulcan/projects/VulcanSerial/example/rw_commond/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vulcan/projects/VulcanSerial/example/rw_commond /home/vulcan/projects/VulcanSerial/example/rw_commond /home/vulcan/projects/VulcanSerial/example/rw_commond/build /home/vulcan/projects/VulcanSerial/example/rw_commond/build /home/vulcan/projects/VulcanSerial/example/rw_commond/build/CMakeFiles/RWCommond.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RWCommond.dir/depend

