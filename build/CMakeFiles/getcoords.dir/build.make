# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_COMMAND = /opt/local/bin/cmake

# The command to remove a file.
RM = /opt/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/ceezeh/Desktop/sam/robotarm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/ceezeh/Desktop/sam/robotarm/build

# Include any dependencies generated for this target.
include CMakeFiles/getcoords.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/getcoords.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/getcoords.dir/flags.make

CMakeFiles/getcoords.dir/src/getcoords.cpp.o: CMakeFiles/getcoords.dir/flags.make
CMakeFiles/getcoords.dir/src/getcoords.cpp.o: ../src/getcoords.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/ceezeh/Desktop/sam/robotarm/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/getcoords.dir/src/getcoords.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/getcoords.dir/src/getcoords.cpp.o -c /Users/ceezeh/Desktop/sam/robotarm/src/getcoords.cpp

CMakeFiles/getcoords.dir/src/getcoords.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/getcoords.dir/src/getcoords.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/ceezeh/Desktop/sam/robotarm/src/getcoords.cpp > CMakeFiles/getcoords.dir/src/getcoords.cpp.i

CMakeFiles/getcoords.dir/src/getcoords.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/getcoords.dir/src/getcoords.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/ceezeh/Desktop/sam/robotarm/src/getcoords.cpp -o CMakeFiles/getcoords.dir/src/getcoords.cpp.s

CMakeFiles/getcoords.dir/src/getcoords.cpp.o.requires:
.PHONY : CMakeFiles/getcoords.dir/src/getcoords.cpp.o.requires

CMakeFiles/getcoords.dir/src/getcoords.cpp.o.provides: CMakeFiles/getcoords.dir/src/getcoords.cpp.o.requires
	$(MAKE) -f CMakeFiles/getcoords.dir/build.make CMakeFiles/getcoords.dir/src/getcoords.cpp.o.provides.build
.PHONY : CMakeFiles/getcoords.dir/src/getcoords.cpp.o.provides

CMakeFiles/getcoords.dir/src/getcoords.cpp.o.provides.build: CMakeFiles/getcoords.dir/src/getcoords.cpp.o

# Object files for target getcoords
getcoords_OBJECTS = \
"CMakeFiles/getcoords.dir/src/getcoords.cpp.o"

# External object files for target getcoords
getcoords_EXTERNAL_OBJECTS =

getcoords: CMakeFiles/getcoords.dir/src/getcoords.cpp.o
getcoords: CMakeFiles/getcoords.dir/build.make
getcoords: /usr/local/lib/libopencv_core.a
getcoords: /usr/local/lib/libopencv_imgproc.a
getcoords: /usr/local/lib/libopencv_highgui.a
getcoords: /usr/lib/libARToolKitPlus.dylib
getcoords: /usr/local/lib/libopencv_imgproc.a
getcoords: /usr/local/lib/libopencv_core.a
getcoords: /usr/local/share/OpenCV/3rdparty/lib/liblibjpeg.a
getcoords: /usr/local/share/OpenCV/3rdparty/lib/liblibpng.a
getcoords: /usr/local/share/OpenCV/3rdparty/lib/liblibtiff.a
getcoords: /usr/local/share/OpenCV/3rdparty/lib/liblibjasper.a
getcoords: /usr/local/share/OpenCV/3rdparty/lib/libIlmImf.a
getcoords: /usr/local/share/OpenCV/3rdparty/lib/libzlib.a
getcoords: /usr/lib/libbz2.dylib
getcoords: CMakeFiles/getcoords.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable getcoords"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/getcoords.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/getcoords.dir/build: getcoords
.PHONY : CMakeFiles/getcoords.dir/build

CMakeFiles/getcoords.dir/requires: CMakeFiles/getcoords.dir/src/getcoords.cpp.o.requires
.PHONY : CMakeFiles/getcoords.dir/requires

CMakeFiles/getcoords.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/getcoords.dir/cmake_clean.cmake
.PHONY : CMakeFiles/getcoords.dir/clean

CMakeFiles/getcoords.dir/depend:
	cd /Users/ceezeh/Desktop/sam/robotarm/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/ceezeh/Desktop/sam/robotarm /Users/ceezeh/Desktop/sam/robotarm /Users/ceezeh/Desktop/sam/robotarm/build /Users/ceezeh/Desktop/sam/robotarm/build /Users/ceezeh/Desktop/sam/robotarm/build/CMakeFiles/getcoords.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/getcoords.dir/depend

