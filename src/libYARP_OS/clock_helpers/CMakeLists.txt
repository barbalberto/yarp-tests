#  Copyright: (C) 2017 iCub Facility, Istituto Italiano di Tecnologia
#  Authors: Alberto Cardellino <alberto.cardellino@iit.it>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

#
# Add here YARP_OS tests
#

project(libYARP_OS_test)

include_directories(${CMAKE_SOURCE_DIR}
                    ${RTF_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${YARP_HELPERS_INCLUDE_DIR})


add_executable       (yarpClock_Producer   yarpTestClock_Producer.cpp)
target_link_libraries(yarpClock_Producer   ${YARP_LIBRARIES})

add_executable       (yarpClock_HWConsumer yarpTestClock_HWConsumer.cpp)
target_link_libraries(yarpClock_HWConsumer ${YARP_LIBRARIES})

add_executable       (yarpClock_SWConsumer yarpTestClock_SWConsumer.cpp)
target_link_libraries(yarpClock_SWConsumer ${YARP_LIBRARIES})

set(execs yarpClock_Producer yarpClock_HWConsumer yarpClock_SWConsumer)

install(TARGETS ${execs}
        EXPORT  ${PROJECT_NAME}
        COMPONENT runtime
        DESTINATION bin
        LIBRARY DESTINATION lib)
