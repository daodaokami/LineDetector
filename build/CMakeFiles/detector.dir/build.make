# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/lut/Desktop/ubuntu_desk_C++/lineDetector

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lut/Desktop/ubuntu_desk_C++/lineDetector/build

# Include any dependencies generated for this target.
include CMakeFiles/detector.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/detector.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/detector.dir/flags.make

CMakeFiles/detector.dir/detect/LineDetector.cpp.o: CMakeFiles/detector.dir/flags.make
CMakeFiles/detector.dir/detect/LineDetector.cpp.o: ../detect/LineDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lut/Desktop/ubuntu_desk_C++/lineDetector/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/detector.dir/detect/LineDetector.cpp.o"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/detector.dir/detect/LineDetector.cpp.o -c /home/lut/Desktop/ubuntu_desk_C++/lineDetector/detect/LineDetector.cpp

CMakeFiles/detector.dir/detect/LineDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detector.dir/detect/LineDetector.cpp.i"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lut/Desktop/ubuntu_desk_C++/lineDetector/detect/LineDetector.cpp > CMakeFiles/detector.dir/detect/LineDetector.cpp.i

CMakeFiles/detector.dir/detect/LineDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detector.dir/detect/LineDetector.cpp.s"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lut/Desktop/ubuntu_desk_C++/lineDetector/detect/LineDetector.cpp -o CMakeFiles/detector.dir/detect/LineDetector.cpp.s

CMakeFiles/detector.dir/detect/LineDetector.cpp.o.requires:

.PHONY : CMakeFiles/detector.dir/detect/LineDetector.cpp.o.requires

CMakeFiles/detector.dir/detect/LineDetector.cpp.o.provides: CMakeFiles/detector.dir/detect/LineDetector.cpp.o.requires
	$(MAKE) -f CMakeFiles/detector.dir/build.make CMakeFiles/detector.dir/detect/LineDetector.cpp.o.provides.build
.PHONY : CMakeFiles/detector.dir/detect/LineDetector.cpp.o.provides

CMakeFiles/detector.dir/detect/LineDetector.cpp.o.provides.build: CMakeFiles/detector.dir/detect/LineDetector.cpp.o


# Object files for target detector
detector_OBJECTS = \
"CMakeFiles/detector.dir/detect/LineDetector.cpp.o"

# External object files for target detector
detector_EXTERNAL_OBJECTS =

libdetector.so: CMakeFiles/detector.dir/detect/LineDetector.cpp.o
libdetector.so: CMakeFiles/detector.dir/build.make
libdetector.so: CMakeFiles/detector.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lut/Desktop/ubuntu_desk_C++/lineDetector/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libdetector.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/detector.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/detector.dir/build: libdetector.so

.PHONY : CMakeFiles/detector.dir/build

CMakeFiles/detector.dir/requires: CMakeFiles/detector.dir/detect/LineDetector.cpp.o.requires

.PHONY : CMakeFiles/detector.dir/requires

CMakeFiles/detector.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/detector.dir/cmake_clean.cmake
.PHONY : CMakeFiles/detector.dir/clean

CMakeFiles/detector.dir/depend:
	cd /home/lut/Desktop/ubuntu_desk_C++/lineDetector/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lut/Desktop/ubuntu_desk_C++/lineDetector /home/lut/Desktop/ubuntu_desk_C++/lineDetector /home/lut/Desktop/ubuntu_desk_C++/lineDetector/build /home/lut/Desktop/ubuntu_desk_C++/lineDetector/build /home/lut/Desktop/ubuntu_desk_C++/lineDetector/build/CMakeFiles/detector.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/detector.dir/depend

