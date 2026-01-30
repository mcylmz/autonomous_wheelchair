## Test CMakeLists.txt for wheelchair_navigation package

# Find testing dependencies
find_package(rostest REQUIRED)
find_package(GTest REQUIRED)

# Include directories
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
    ../include
)

# C++ Unit Tests
catkin_add_gtest(test_chair_controller
    test_chair_controller.cpp
    ../src/ChairController.cpp
    ../src/RoboteqDevice.cpp
    ../src/RoboteqExtended.cpp
)
target_link_libraries(test_chair_controller
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    pthread
)

# Integration Tests (requires roscore)
add_rostest_gtest(test_integration
    test_integration.test
    test_integration.cpp
    ../src/ChairController.cpp
    ../src/RoboteqDevice.cpp
    ../src/RoboteqExtended.cpp
)
target_link_libraries(test_integration
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    pthread
)

# Python Tests
catkin_add_nosetests(test_velocity_mapping.py)

# Test data and configurations
file(COPY
    ${CMAKE_CURRENT_SOURCE_DIR}/../config/
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test/config/
)
