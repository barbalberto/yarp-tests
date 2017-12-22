#  YARP middleware test units ( using Robot Testing Framework)
#  Copyright: (C) 2017 iCub Facility, Istituto Italiano di Tecnologia
#  Authors: Alberto Cardellino <alberto.cardellino@iit.it>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

cmake_minimum_required(VERSION 2.8.9)
project(yarp-tests)
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin )

set (CMAKE_CXX_STANDARD 11)

find_package(YARP REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}
                    ${RTF_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${YARP_HELPERS_INCLUDE_DIR})

# find a version of RTF higher then 1.0.1
# and with yarp support enabled
find_package(RTF 1.3.3 COMPONENTS DLL REQUIRED)

# set the output plugin directory to collect all the shared libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
set(CMAKE_SHARED_MODULE_PREFIX "")


# options
option(YARP_TESTS_USES_ROS   "Turn on to compile the tests that rely on 'ROS' external dependency" OFF)
option(ENABLE_BROKEN_TESTS   "Turn on to compile known broken tests" OFF)


# Build main tests
add_subdirectory(src)
add_subdirectory(noRTF)

# Build examples: skeleton of tipical YARP test to be used when writing new tests
# add_subdirectory(example/cpp)

# Build optional tests that rely on external dependencies
# add_subdirectory(external)



