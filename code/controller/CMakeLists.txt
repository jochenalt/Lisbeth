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
  include/MPCSolver.hpp
  include/GaitPlanner.hpp
  include/Utils.hpp
  include/Estimator.hpp
  include/FootTrajectoryGenerator.hpp
  include/FootstepPlanner.hpp
  include/BodyPlanner.hpp
  include/Types.h
  include/InvKin.hpp
  include/WBCSolver.hpp
  include/Params.hpp
  include/st_to_cc.hpp
  include/ComplementaryFilter.hpp
  include/Filter.hpp
  include/WBCController.hpp
  include/MPCController.hpp
  include/Controller.hpp
  include/KeyboardListener.hpp
  )

set(${PROJECT_NAME}_SOURCES
  src/WBCController.cpp
  src/InvKin.cpp
  src/Controller.cpp
  src/Estimator.cpp
  src/FootstepPlanner.cpp
  src/MPCController.cpp
  src/WBCSolver.cpp
  src/MPCSolver.cpp
  src/st_to_cc.cpp
  src/GaitPlanner.cpp
  src/Utils.cpp
  src/FootTrajectoryGenerator.cpp
  src/BodyPlanner.cpp
  src/Params.cpp
  src/ComplementaryFilter.cpp
  src/Filter.cpp
  src/KeyboardListener.cpp
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

# Find OSQP library and headers
find_package(osqp REQUIRED)

# Link the OSQP shared library
target_link_libraries(${PROJECT_NAME} PUBLIC osqp::osqp)

add_project_dependency(yaml-cpp CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC ${YAML_CPP_LIBRARIES})

if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif()

#target_compile_options(${PROJECT_NAME} PUBLIC -DNDEBUG -O3 -ffast-math)
target_compile_options(${PROJECT_NAME} PUBLIC -O3 -ffast-math)

target_compile_definitions(${PROJECT_NAME} PUBLIC CONFIG_SOLO12_YAML="${PROJECT_SOURCE_DIR}/config/config_solo12.yaml")
target_compile_definitions(${PROJECT_NAME} PUBLIC WALK_PARAMETERS_YAML="${PROJECT_SOURCE_DIR}/config/config.yaml")
target_compile_definitions(${PROJECT_NAME} PUBLIC URDF_MODEL="${PROJECT_SOURCE_DIR}/../../description/solo12.urdf")
target_compile_definitions(${PROJECT_NAME} PUBLIC SRDF_MODEL="${PROJECT_SOURCE_DIR}/../../description/solo.srdf")

# path where pinccocio is supposed to look for meshes etc.
target_compile_definitions(${PROJECT_NAME} PUBLIC DESCRIPTION_PATH="${PROJECT_SOURCE_DIR}/../../description")


# Main Executable
add_executable(${PROJECT_NAMESPACE}-${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAMESPACE}-${PROJECT_NAME} ${PROJECT_NAME})

# Python Bindings
if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(stubs)
endif()


# Installation
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
  install(TARGETS ${PROJECT_NAMESPACE}-${PROJECT_NAME} RUNTIME DESTINATION bin)
endif()
