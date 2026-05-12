cmake_minimum_required(VERSION 3.15)
project(osrm_wrapper LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED COMPONENTS iostreams filesystem thread)
find_package(Threads REQUIRED)

find_path(OSRM_INCLUDE_DIR NAMES osrm/osrm.hpp
  PATHS /usr/local/include /usr/include
)
find_library(OSRM_LIBRARY NAMES osrm
  PATHS /usr/local/lib /usr/lib
)

if(NOT OSRM_INCLUDE_DIR OR NOT OSRM_LIBRARY)
  message(FATAL_ERROR "Could not find OSRM. Run the prepare script first.")
endif()

message(STATUS "Found OSRM: ${OSRM_LIBRARY}")
message(STATUS "OSRM include: ${OSRM_INCLUDE_DIR}")

add_library(osrm_wrapper SHARED
  osrm_wrapper.cpp
  parsers.cpp
)

target_include_directories(osrm_wrapper PRIVATE
  ${OSRM_INCLUDE_DIR}
  ${OSRM_INCLUDE_DIR}/osrm
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(osrm_wrapper PRIVATE
  ${OSRM_LIBRARY}
  Boost::iostreams
  Boost::filesystem
  Boost::thread
  Threads::Threads
)