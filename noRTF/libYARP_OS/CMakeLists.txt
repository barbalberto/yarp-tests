#  Copyright: (C) 2017 iCub Facility, Istituto Italiano di Tecnologia
#  Authors: Alberto Cardellino <alberto.cardellino@iit.it>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

#
# Add here YARP_OS tests
#

project(noRTF_libYARP_OS_test)

include_directories(${CMAKE_SOURCE_DIR}
                    ${RTF_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${YARP_HELPERS_INCLUDE_DIR})

add_executable       (envelopePublisher   envelopePublisher.cpp)
add_executable       (envelopeReader      envelopeReader.cpp)

target_link_libraries(envelopePublisher   ${YARP_LIBRARIES})
target_link_libraries(envelopeReader      ${YARP_LIBRARIES})


