# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.12.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.12.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/apurvagb/argos3-examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/apurvagb/argos3-examples/build

# Include any dependencies generated for this target.
include controllers/thesis_sample/CMakeFiles/thesis_sample.dir/depend.make

# Include the progress variables for this target.
include controllers/thesis_sample/CMakeFiles/thesis_sample.dir/progress.make

# Include the compile flags for this target's objects.
include controllers/thesis_sample/CMakeFiles/thesis_sample.dir/flags.make

controllers/thesis_sample/CMakeFiles/thesis_sample.dir/thesis.cpp.o: controllers/thesis_sample/CMakeFiles/thesis_sample.dir/flags.make
controllers/thesis_sample/CMakeFiles/thesis_sample.dir/thesis.cpp.o: ../controllers/thesis_sample/thesis.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/apurvagb/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object controllers/thesis_sample/CMakeFiles/thesis_sample.dir/thesis.cpp.o"
	cd /Users/apurvagb/argos3-examples/build/controllers/thesis_sample && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/thesis_sample.dir/thesis.cpp.o -c /Users/apurvagb/argos3-examples/controllers/thesis_sample/thesis.cpp

controllers/thesis_sample/CMakeFiles/thesis_sample.dir/thesis.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/thesis_sample.dir/thesis.cpp.i"
	cd /Users/apurvagb/argos3-examples/build/controllers/thesis_sample && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/apurvagb/argos3-examples/controllers/thesis_sample/thesis.cpp > CMakeFiles/thesis_sample.dir/thesis.cpp.i

controllers/thesis_sample/CMakeFiles/thesis_sample.dir/thesis.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/thesis_sample.dir/thesis.cpp.s"
	cd /Users/apurvagb/argos3-examples/build/controllers/thesis_sample && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/apurvagb/argos3-examples/controllers/thesis_sample/thesis.cpp -o CMakeFiles/thesis_sample.dir/thesis.cpp.s

controllers/thesis_sample/CMakeFiles/thesis_sample.dir/thesis_sample_autogen/mocs_compilation.cpp.o: controllers/thesis_sample/CMakeFiles/thesis_sample.dir/flags.make
controllers/thesis_sample/CMakeFiles/thesis_sample.dir/thesis_sample_autogen/mocs_compilation.cpp.o: controllers/thesis_sample/thesis_sample_autogen/mocs_compilation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/apurvagb/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object controllers/thesis_sample/CMakeFiles/thesis_sample.dir/thesis_sample_autogen/mocs_compilation.cpp.o"
	cd /Users/apurvagb/argos3-examples/build/controllers/thesis_sample && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/thesis_sample.dir/thesis_sample_autogen/mocs_compilation.cpp.o -c /Users/apurvagb/argos3-examples/build/controllers/thesis_sample/thesis_sample_autogen/mocs_compilation.cpp

controllers/thesis_sample/CMakeFiles/thesis_sample.dir/thesis_sample_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/thesis_sample.dir/thesis_sample_autogen/mocs_compilation.cpp.i"
	cd /Users/apurvagb/argos3-examples/build/controllers/thesis_sample && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/apurvagb/argos3-examples/build/controllers/thesis_sample/thesis_sample_autogen/mocs_compilation.cpp > CMakeFiles/thesis_sample.dir/thesis_sample_autogen/mocs_compilation.cpp.i

controllers/thesis_sample/CMakeFiles/thesis_sample.dir/thesis_sample_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/thesis_sample.dir/thesis_sample_autogen/mocs_compilation.cpp.s"
	cd /Users/apurvagb/argos3-examples/build/controllers/thesis_sample && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/apurvagb/argos3-examples/build/controllers/thesis_sample/thesis_sample_autogen/mocs_compilation.cpp -o CMakeFiles/thesis_sample.dir/thesis_sample_autogen/mocs_compilation.cpp.s

# Object files for target thesis_sample
thesis_sample_OBJECTS = \
"CMakeFiles/thesis_sample.dir/thesis.cpp.o" \
"CMakeFiles/thesis_sample.dir/thesis_sample_autogen/mocs_compilation.cpp.o"

# External object files for target thesis_sample
thesis_sample_EXTERNAL_OBJECTS =

controllers/thesis_sample/libthesis_sample.dylib: controllers/thesis_sample/CMakeFiles/thesis_sample.dir/thesis.cpp.o
controllers/thesis_sample/libthesis_sample.dylib: controllers/thesis_sample/CMakeFiles/thesis_sample.dir/thesis_sample_autogen/mocs_compilation.cpp.o
controllers/thesis_sample/libthesis_sample.dylib: controllers/thesis_sample/CMakeFiles/thesis_sample.dir/build.make
controllers/thesis_sample/libthesis_sample.dylib: controllers/thesis_sample/CMakeFiles/thesis_sample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/apurvagb/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library libthesis_sample.dylib"
	cd /Users/apurvagb/argos3-examples/build/controllers/thesis_sample && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/thesis_sample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
controllers/thesis_sample/CMakeFiles/thesis_sample.dir/build: controllers/thesis_sample/libthesis_sample.dylib

.PHONY : controllers/thesis_sample/CMakeFiles/thesis_sample.dir/build

controllers/thesis_sample/CMakeFiles/thesis_sample.dir/clean:
	cd /Users/apurvagb/argos3-examples/build/controllers/thesis_sample && $(CMAKE_COMMAND) -P CMakeFiles/thesis_sample.dir/cmake_clean.cmake
.PHONY : controllers/thesis_sample/CMakeFiles/thesis_sample.dir/clean

controllers/thesis_sample/CMakeFiles/thesis_sample.dir/depend:
	cd /Users/apurvagb/argos3-examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/apurvagb/argos3-examples /Users/apurvagb/argos3-examples/controllers/thesis_sample /Users/apurvagb/argos3-examples/build /Users/apurvagb/argos3-examples/build/controllers/thesis_sample /Users/apurvagb/argos3-examples/build/controllers/thesis_sample/CMakeFiles/thesis_sample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : controllers/thesis_sample/CMakeFiles/thesis_sample.dir/depend

