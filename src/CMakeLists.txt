#  Copyright: (C) 2017 iCub Facility, Istituto Italiano di Tecnologia
#  Authors: Alberto Cardellino <alberto.cardellino@iit.it>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

#
# Add all subdirectory for different test cases
#

if(RTF_FOUND)
    add_subdirectory(libYARP_dev)
    add_subdirectory(libYARP_OS)
    add_subdirectory(devices)
    add_subdirectory(integration)

    if(ENABLE_BROKEN_TESTS)
        add_subdirectory(broken)
    endif(ENABLE_BROKEN_TESTS)

else()
    message("RTF not found, disabling RTF tests")

endif()