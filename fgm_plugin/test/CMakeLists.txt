## Test CMakeLists.txt for fgm_plugin package

# Find testing dependencies
find_package(rostest REQUIRED)
find_package(GTest REQUIRED)

# Include directories
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
    ../include
)

# PID Controller Unit Tests
catkin_add_gtest(test_pid_controller
    test_pid_controller.cpp
    ../src/PID.cpp
)
target_link_libraries(test_pid_controller
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    pthread
)

# Data Validation Tests (requires roscore)
add_rostest_gtest(test_data_validation
    test_data_validation.test
    test_data_validation.cpp
)
target_link_libraries(test_data_validation
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    pthread
)
