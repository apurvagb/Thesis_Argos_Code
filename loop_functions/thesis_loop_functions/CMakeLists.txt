link_directories(${CMAKE_BINARY_DIR}/controllers/thesis_sample)
set(thesis_loop_functions_SOURCES thesis_loop_functions.cpp)

if(ARGOS_COMPILE_QTOPENGL)
  set(thesis_loop_functions_SOURCES
    ${thesis_loop_functions_SOURCES}
    thesis_qtuser_functions.cpp)
endif(ARGOS_COMPILE_QTOPENGL)

add_library(thesis_loop_functions MODULE ${thesis_loop_functions_SOURCES})
target_link_libraries(thesis_loop_functions
  thesis_sample
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(thesis_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_COMPILE_QTOPENGL)
