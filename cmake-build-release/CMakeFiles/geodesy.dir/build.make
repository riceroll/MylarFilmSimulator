# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/Roll/Desktop/mylarSimulator

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/Roll/Desktop/mylarSimulator/cmake-build-release

# Include any dependencies generated for this target.
include CMakeFiles/geodesy.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/geodesy.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/geodesy.dir/flags.make

CMakeFiles/geodesy.dir/main.cpp.o: CMakeFiles/geodesy.dir/flags.make
CMakeFiles/geodesy.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Roll/Desktop/mylarSimulator/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/geodesy.dir/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/geodesy.dir/main.cpp.o -c /Users/Roll/Desktop/mylarSimulator/main.cpp

CMakeFiles/geodesy.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/geodesy.dir/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/Roll/Desktop/mylarSimulator/main.cpp > CMakeFiles/geodesy.dir/main.cpp.i

CMakeFiles/geodesy.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/geodesy.dir/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/Roll/Desktop/mylarSimulator/main.cpp -o CMakeFiles/geodesy.dir/main.cpp.s

# Object files for target geodesy
geodesy_OBJECTS = \
"CMakeFiles/geodesy.dir/main.cpp.o"

# External object files for target geodesy
geodesy_EXTERNAL_OBJECTS =

geodesy: CMakeFiles/geodesy.dir/main.cpp.o
geodesy: CMakeFiles/geodesy.dir/build.make
geodesy: lib/libShapeOp/libShapeOp.0.1.0.dylib
geodesy: libmodel.dylib
geodesy: /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX11.1.sdk/System/Library/Frameworks/OpenGL.framework/OpenGL.tbd
geodesy: imgui/libimgui.a
geodesy: glfw/src/libglfw3.a
geodesy: glad/libglad.a
geodesy: CMakeFiles/geodesy.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/Roll/Desktop/mylarSimulator/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable geodesy"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/geodesy.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/geodesy.dir/build: geodesy

.PHONY : CMakeFiles/geodesy.dir/build

CMakeFiles/geodesy.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/geodesy.dir/cmake_clean.cmake
.PHONY : CMakeFiles/geodesy.dir/clean

CMakeFiles/geodesy.dir/depend:
	cd /Users/Roll/Desktop/mylarSimulator/cmake-build-release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/Roll/Desktop/mylarSimulator /Users/Roll/Desktop/mylarSimulator /Users/Roll/Desktop/mylarSimulator/cmake-build-release /Users/Roll/Desktop/mylarSimulator/cmake-build-release /Users/Roll/Desktop/mylarSimulator/cmake-build-release/CMakeFiles/geodesy.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/geodesy.dir/depend

