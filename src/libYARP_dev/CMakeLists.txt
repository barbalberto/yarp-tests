#  Copyright: (C) 2017 iCub Facility, Istituto Italiano di Tecnologia
#  Authors: Alberto Cardellino <alberto.cardellino@iit.it>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

#
# Add here YARP_OS tests
#

message(STATUS " No libYARP_OS tests yet ... please populate this category!")



include_directories(${CMAKE_SOURCE_DIR}
                    ${RTF_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${YARP_HELPERS_INCLUDE_DIR})

link_libraries(${RTF_LIBRARIES} ${YARP_LIBRARIES})

#add_library(${PROJECTNAME} MODULE ControlModes.h ControlModes.cpp)

#install(TARGETS ${PROJECTNAME}
#        EXPORT ${PROJECTNAME}
#        COMPONENT runtime
#        LIBRARY DESTINATION lib)
