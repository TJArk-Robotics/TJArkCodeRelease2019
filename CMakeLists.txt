cmake_minimum_required(VERSION 3.5)

project(tjark)

# Change default bin file location
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Bin)
# Change default .so file location
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Lib})
# Change default .a file location
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Lib)

add_subdirectory(msgpack)

add_compile_options(-O3 -Wall -march=core2 -mtune=core2 -mfpmath=sse -msse4 -ggdb)
add_definitions(-D TARGET_ROBOT)
# add_definitions(-D NDEBUG)

# Packages
find_package(Boost COMPONENTS system program_options filesystem REQUIRED)
find_package(OpenCV)

include_directories(
    Src
    Src/Nao
    Src/Aruco/src
    Src/gtest/include
    ${OpenCV_INCLUDE_DIRS}
)
link_directories(
    Lib
    ctc/gtest/lib
)

# Build lola_connector
add_executable(lola_connector
    Src/LoLAConnector/leds.cpp
    Src/LoLAConnector/lola_connector.cpp
    Src/LoLAConnector/lola_frame.cpp
    Src/LoLAConnector/UdpComm.cpp
)
target_link_libraries(lola_connector Boost::system msgpack -lpthread -lrt)
target_include_directories(lola_connector PUBLIC .)
set_target_properties(lola_connector PROPERTIES CXX_STANDARD 14)

# Build Aruco libraries

file( GLOB_RECURSE ARUCO_SOURCES Src/Aruco/src/aruco/[^.]*.cpp )
file( GLOB_RECURSE ARUCO_HEADERS Src/Aruco/src/aruco/[^.]*.h )

add_library(aruco STATIC ${ARUCO_SOURCES} ${ARUCO_HEADERS})
set_target_properties(aruco 
    PROPERTIES          # create *nix style library versions + symbolic links
    CXX_STANDARD 14
)
target_link_libraries(aruco ${OpenCV_LIBS} Boost::system -lpthread -lrt)

# Build gtest libraries
add_library(gtest STATIC Src/gtest/src/gtest-all.cc)
target_include_directories(gtest
    INTERFACE
    Src/gtest
    Src/gtest/include
)

# Build Nao
file( GLOB_RECURSE NAO_SOURCES Src/Nao/[^.]*.cpp )
file( GLOB_RECURSE NAO_HEADERS Src/Nao/[^.]*.h )

set(ARUCO_SOURCES
    Src/Aruco/tools.cpp
)

add_executable(nao
    ${NAO_SOURCES}
    ${NAO_HEADERS}
    ${ARUCO_SOURCES}
)
target_link_libraries(nao ${OpenCV_LIBS} Boost::system -lpthread -lrt -laruco -lgtest)
set_target_properties(nao PROPERTIES CXX_STANDARD 14)

# Read battery volume
include_directories(test)
add_executable(battery test/battery.cpp)
target_link_libraries(battery Boost::system -lpthread -lrt)
set_target_properties(battery PROPERTIES CXX_STANDARD 14)

# Check sensor data
add_executable(sensor_check test/sensor_check.cpp)
target_link_libraries(sensor_check Boost::system -lpthread -lrt)
set_target_properties(sensor_check PROPERTIES CXX_STANDARD 14)

# Test code
add_executable(test_code test/test.cpp)
set_target_properties(test_code PROPERTIES CXX_STANDARD 14)
target_link_libraries(test_code ${OpenCV_LIBS})
