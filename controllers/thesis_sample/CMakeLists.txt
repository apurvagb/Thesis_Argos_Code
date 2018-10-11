add_library(thesis_sample SHARED thesis.h thesis.cpp)
target_link_libraries(thesis_sample
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
