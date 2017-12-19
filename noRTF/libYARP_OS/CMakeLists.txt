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

add_executable       (envelopeProducer    envelopeProducer.cpp)
add_executable       (envelopeConsumer    envelopeConsumer.cpp)

target_link_libraries(envelopeProducer    ${YARP_LIBRARIES})
target_link_libraries(envelopeConsumer    ${YARP_LIBRARIES})


