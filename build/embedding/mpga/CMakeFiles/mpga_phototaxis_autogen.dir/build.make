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

# Utility rule file for mpga_phototaxis_autogen.

# Include the progress variables for this target.
include embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/progress.make

embedding/mpga/CMakeFiles/mpga_phototaxis_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/apurvagb/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC for target mpga_phototaxis"
	cd /Users/apurvagb/argos3-examples/build/embedding/mpga && /usr/local/Cellar/cmake/3.12.2/bin/cmake -E cmake_autogen /Users/apurvagb/argos3-examples/build/embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/AutogenInfo.cmake ""

mpga_phototaxis_autogen: embedding/mpga/CMakeFiles/mpga_phototaxis_autogen
mpga_phototaxis_autogen: embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/build.make

.PHONY : mpga_phototaxis_autogen

# Rule to build all files generated by this target.
embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/build: mpga_phototaxis_autogen

.PHONY : embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/build

embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/clean:
	cd /Users/apurvagb/argos3-examples/build/embedding/mpga && $(CMAKE_COMMAND) -P CMakeFiles/mpga_phototaxis_autogen.dir/cmake_clean.cmake
.PHONY : embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/clean

embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/depend:
	cd /Users/apurvagb/argos3-examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/apurvagb/argos3-examples /Users/apurvagb/argos3-examples/embedding/mpga /Users/apurvagb/argos3-examples/build /Users/apurvagb/argos3-examples/build/embedding/mpga /Users/apurvagb/argos3-examples/build/embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/depend

