cmake_minimum_required(VERSION 3.15)
project(osrm_wrapper LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# OSRM dependencies
find_package(Boost REQUIRED COMPONENTS iostreams filesystem thread)
find_package(Threads REQUIRED)

# Fetch OSRM backend and build it as a dependency
include(FetchContent)
FetchContent_Declare(
  osrm_backend
  GIT_REPOSITORY https://github.com/Project-OSRM/osrm-backend.git
  GIT_TAG 362b388d7e0582291662105d7bfc004a3a44a393
)

FetchContent_MakeAvailable(osrm_backend)


add_library(osrm_wrapper SHARED
  osrm_wrapper.cpp
  parsers.cpp
)

target_include_directories(osrm_wrapper PRIVATE
    ${osrm_backend_SOURCE_DIR}/include
    ${osrm_backend_SOURCE_DIR}/include/osrm
    ${osrm_backend_SOURCE_DIR}/third_party/variant/include
    ${osrm_backend_SOURCE_DIR}/third_party/stxxl/include
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(osrm_wrapper PRIVATE
    osrm
    ${Boost_LIBRARIES}
    Threads::Threads
)

if(WIN32)
  set_target_properties(osrm_wrapper PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()
