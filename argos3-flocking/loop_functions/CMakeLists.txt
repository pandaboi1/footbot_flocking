include_directories(${CMAKE_SOURCE_DIR})

# Descend into the flocking_loop_functions directory
add_subdirectory(flocking_loop_functions)


# If Qt+OpenGL dependencies were found, descend into these directories
if(ARGOS_QTOPENGL_FOUND)
  add_subdirectory(manualcontrol_loop_functions)
endif(ARGOS_QTOPENGL_FOUND)
