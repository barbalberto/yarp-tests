CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(synchTest)

FIND_PACKAGE(YARP REQUIRED)
LIST(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

SET(sender   synch_sender.cpp)
SET(receiver synch_recv.cpp)
SOURCE_GROUP("Source Files" FILES ${sourcea})


include_directories(${YARP_INCLUDE_DIRS})
add_executable(sender ${sender})
add_executable(receiver ${receiver})

target_link_libraries(sender ${YARP_LIBRARIES})
target_link_libraries(receiver ${YARP_LIBRARIES})

