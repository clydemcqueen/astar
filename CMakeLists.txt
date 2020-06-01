cmake_minimum_required(VERSION 3.5)
project(astar)

#=============
# Setup
#=============

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find packages
find_package(ament_cmake REQUIRED)

# Local includes
include_directories(
  include
)

#=============
# Library
#=============

add_library(
  astar SHARED
  src/astar.cpp
)
ament_export_libraries(astar)

#=============
# Test
#=============

add_executable(
  astar_test
  src/astar_test.cpp
  src/astar.cpp
)

# This will load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#=============
# Install
#=============

# Export include directory
ament_export_include_directories(include)

# Install include directory
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

## Install library
install(
  TARGETS astar
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executable
install(
  TARGETS astar_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
