cmake_minimum_required(VERSION 3.0)

project(HelicopterSimulation)

################################################################################
# Configuration section                                                        #
################################################################################

# Compile library only (WITH_SIM = 0) or the simulator also (WITH_SIM = 1)
set(WITH_SIM 1)

# Compile the required dependencies "glut" (COMPILE_GLUT = 1) or use a system one (COMPILE_GLUT = 0)
# If using a system one the line
#   target_link_libraries(HelicopterSimulationSim glut)
# must be updated accordingly (if the glut.lib name is different from "glut")
# Further, a copying a glut.dll file to the binary folder *may* be needed (if not installed in system path)
set(COMPILE_GLUT 1)

# glut folder if COMPILE_GLUT is set to 1
set(GLUT_FOLDER ${CMAKE_SOURCE_DIR}/3rdparty/freeglut/)

################################################################################
# Do NOT edit below this line (unless you know what you're doing)              #
################################################################################

# Project output:
# * HelicopterSimulationLib: simulation static library
# * HelicopterSimulationSim: included test simulator executable
# The HelicopterSimulationSim is available only if WITH_SIM is set to 1

# Set the library source files
set(LIB_SRC
  src/matlib.c
  src/model.c
  src/simlib.c
)

# Create the static library
add_library(HelicopterSimulationLib STATIC ${LIB_SRC})

# If simulator is required
if(WITH_SIM)

  # Set the simulator source files
  set(SIM_SRC
    src/graphics.c
    src/helisim.c
    src/instruments.c
  )

  # If glut compilation is required
  if(COMPILE_GLUT)
    # Check glut folder exists, since the user should manually download it
    if(NOT EXISTS ${GLUT_FOLDER})
      message(FATAL_ERROR
      "Missing glut library."
      "Please download glut (http://freeglut.sourceforge.net/) and extract it in '${GLUT_FOLDER}' without any intermediate folder (the '${GLUT_FOLDER}' folder must contain a CMakeLists.txt file)")
    endif()

    # Add glut project
    add_subdirectory(${GLUT_FOLDER})

    # Add glut to include directories
    include_directories(${GLUT_FOLDER}/include)
  endif(COMPILE_GLUT)

  # Create the simulator executable...
  add_executable(HelicopterSimulationSim ${SIM_SRC})
  # ... and link it:
  #  1) with the simulation library
  target_link_libraries(HelicopterSimulationSim HelicopterSimulationLib)
  
  if(COMPILE_GLUT)
    #  2a) with compiled glut
    target_link_libraries(HelicopterSimulationSim freeglut)

    # Deploy glut dll file to bin directory
    add_custom_command(TARGET HelicopterSimulationSim POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "$<TARGET_FILE:freeglut>"
      "$<TARGET_FILE_DIR:HelicopterSimulationSim>")
  else(COMPILE_GLUT)
    #  2b) with system glut
    target_link_libraries(HelicopterSimulationSim glut)
  endif(COMPILE_GLUT)

endif(WITH_SIM)
