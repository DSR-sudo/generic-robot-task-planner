cmake_minimum_required(VERSION 3.16)
project(mission_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(MISSION_CORE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
if(NOT TARGET mission_core)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../mission_core
    ${CMAKE_CURRENT_BINARY_DIR}/mission_core)
endif()

add_library(mission_sim
  src/sim_executor.cpp)
target_include_directories(mission_sim
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(mission_sim PUBLIC mission_core)
target_compile_features(mission_sim PUBLIC cxx_std_17)
target_compile_options(mission_sim PRIVATE -Wall -Wextra -Wpedantic)

add_executable(mission_sim_demo src/main.cpp)
target_link_libraries(mission_sim_demo PRIVATE mission_sim mission_core)
target_compile_options(mission_sim_demo PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS mission_sim mission_sim_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/mission_sim)

include(CTest)
enable_testing()
add_executable(mission_sim_test test/test_sim.cpp)
target_link_libraries(mission_sim_test PRIVATE mission_sim mission_core)
target_compile_options(mission_sim_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME mission_sim_test COMMAND mission_sim_test)
