add_library(thesis_loop_functions MODULE 
  thesis_loop_functions.h
  thesis_qtuser_functions.h
  thesis_loop_functions.cpp
  thesis_qtuser_functions.cpp)

target_link_libraries(thesis_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
