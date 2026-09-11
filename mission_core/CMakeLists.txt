cmake_minimum_required(VERSION 3.16)
project(mission_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MISSION_CORE_BUILD_TESTS "Build mission_core tests" ON)

find_package(tinyxml2 REQUIRED)

add_library(mission_core
  src/types.cpp
  src/executor.cpp
  src/safety.cpp
  src/mission_tree.cpp)
target_include_directories(mission_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(mission_core PUBLIC tinyxml2::tinyxml2)
target_compile_features(mission_core PUBLIC cxx_std_17)
target_compile_options(mission_core PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS mission_core
  EXPORT mission_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT mission_coreTargets
  FILE mission_coreTargets.cmake
  NAMESPACE mission_core::
  DESTINATION lib/cmake/mission_core)

if(MISSION_CORE_BUILD_TESTS)
  include(CTest)
  enable_testing()
  add_executable(mission_core_test test/test_core.cpp)
  target_link_libraries(mission_core_test PRIVATE mission_core)
  target_compile_options(mission_core_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME mission_core_test COMMAND mission_core_test)
endif()
