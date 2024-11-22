#find_library(optoforce_LIBRARY
#            NAMES optoforce
#            #PATHS "${optoforce_DIR}/../../../lib/"
#            PATHS "${optoforce_DIR}/../../../../build/optoforce/"
#            NO_DEFAULT_PATH)

find_library(optoforce_LIBRARY
            NAMES optoforce
            #PATHS "${optoforce_DIR}/../../../lib/"
            PATHS "${optoforce_DIR}/../../../../build/optoforce/")

message ("Test here we are")
message ("Checking for optoforce in: ${optoforce_DIR}/../../../../build/optoforce/")
if(optoforce_LIBRARY)
  # Multiple CMake projects case (i.e. 'catkin build'):
  # - The target has already been built when its dependencies require it
  # - Specify full path to found library
  message ("catkin build case") 
  list(APPEND optoforce_LIBRARIES ${optoforce_LIBRARY})
else()
  # Single CMake project case (i.e. 'catkin_make'):
  # - The target has not been built when its dependencies require it
  # - Specify target name only
  message ("catkin_make case") 
  list(APPEND optoforce_LIBRARIES optoforce)
endif()
