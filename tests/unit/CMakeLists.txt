add_executable(openrover_unit_tests
    test_differential_drive_kinematics.cpp
)

target_link_libraries(openrover_unit_tests
    PRIVATE
        openrover_math
        GTest::gtest_main
)

# Registers the tests with CTest, so running "ctest" finds and runs
# every TEST(...) defined in this executable.
include(GoogleTest)
gtest_discover_tests(openrover_unit_tests)
