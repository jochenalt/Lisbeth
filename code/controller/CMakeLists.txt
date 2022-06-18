cmake_minimum_required(VERSION 3.1)

set (CMAKE_CXX_STANDARD 11)

# Project properties
set(PROJECT_NAME controller-core)
set(PROJECT_DESCRIPTION "control for lisbeth")
set(PROJECT_URL https://github.com/jochenalt/lisbeth)

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python binding" ON)
option(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python bindings" OFF)
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif()

# JRL-cmakemodule setup
include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/python.cmake)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
set(BOOST_COMPONENTS unit_test_framework)
ADD_PROJECT_DEPENDENCY(Eigen3 REQUIRED)

set(BUILD_PYTHON_INTERFACE TRUE)
if(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  ADD_PROJECT_DEPENDENCY(eigenpy REQUIRED)
  string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  set(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})
  set(BOOST_COMPONENTS ${BOOST_COMPONENTS} python)
endif()

SEARCH_FOR_BOOST()

# Main Library
set(${PROJECT_NAME}_HEADERS
  include/MPC.hpp
  include/Gait.hpp
  include/Utils.hpp
  include/Kinematics.hpp
  include/Estimator.hpp
  include/FootTrajectoryGenerator.hpp
  include/FootstepPlanner.hpp
  include/StatePlanner.hpp
  include/Types.h
  include/InvKin.hpp
  include/QPWBC.hpp
  include/Params.hpp
  include/st_to_cc.hpp
  include/ComplementaryFilter.hpp
  include/Filter.hpp
#  include/WbcWrapper.hpp

  )

set(${PROJECT_NAME}_SOURCES
  src/st_to_cc.cpp
  src/MPC.cpp
  src/Gait.cpp
  src/Utils.cpp
  src/Kinematics.cpp
  src/Estimator.cpp
  src/FootTrajectoryGenerator.cpp
  src/FootstepPlanner.cpp
  src/StatePlanner.cpp
  src/InvKin.cpp
  src/QPWBC.cpp
  src/Params.cpp
  src/ComplementaryFilter.cpp
  src/Filter.cpp
  src/WbcWrapper.cpp
  src/MpcWrapper.cpp

  )

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)

# Include Eigen3 directories
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM INTERFACE ${EIGEN3_INCLUDE_DIR})

# Find pinocchio library
add_project_dependency(pinocchio REQUIRED)

# Link pinocchio library
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC pinocchio::pinocchio)

# Find parametric curves library and headers
# add_project_dependency(curves REQUIRED)

# Link parametric curves library
# target_link_libraries(${PROJECT_NAME} PUBLIC curves::curves)

# Find eiquadprog library and headers
add_project_dependency(eiquadprog REQUIRED)

# Link eiquadprog library
target_link_libraries(${PROJECT_NAME} PUBLIC eiquadprog::eiquadprog)

# Find OSQP library and headers
find_package(osqp REQUIRED)

# Link the OSQP shared library
target_link_libraries(${PROJECT_NAME} PUBLIC osqp::osqp)

#find_package(yaml-cpp REQUIRED)
#target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp::yaml-cpp)
add_project_dependency(yaml-cpp CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC ${YAML_CPP_LIBRARIES})

if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif()

# target_compile_options(${PROJECT_NAME} PUBLIC PRINTING)
# target_compile_definitions(${PROJECT_NAME} PUBLIC PRINTING)

target_compile_options(${PROJECT_NAME} PUBLIC -DNDEBUG -O3)

target_compile_definitions(${PROJECT_NAME} PUBLIC CONFIG_SOLO12_YAML="${PROJECT_SOURCE_DIR}/config/config_solo12.yaml")
target_compile_definitions(${PROJECT_NAME} PUBLIC WALK_PARAMETERS_YAML="${PROJECT_SOURCE_DIR}/config/config.yaml")


# Main Executable
add_executable(${PROJECT_NAMESPACE}-${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAMESPACE}-${PROJECT_NAME} ${PROJECT_NAME})

# Python Bindings
if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif()


# Installation
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
  install(TARGETS ${PROJECT_NAMESPACE}-${PROJECT_NAME} RUNTIME DESTINATION bin)
endif()
