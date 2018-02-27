# add_subdirectory(testMotionControlInterfaces)

# add_subdirectory(synchTest)

# add_subdirectory(testMotionControlInterfaces)
# add_subdirectory(clock)
# add_subdirectory(eredita)
# add_subdirectory(Issue_1303)
add_subdirectory(libYARP_OS)
add_subdirectory(libYARP_dev)

if(PCL_FOUND)
  add_subdirectory(libYARP_sig)
endif()
