add_executable(measurement_unit_tests
    unit/CoreCoordinateTests.cpp
    unit/InstrumentTests.cpp
    unit/CpuDrrTests.cpp
    unit/ProjectManifestTests.cpp
)

target_link_libraries(measurement_unit_tests
    PRIVATE
        measurement::core
        measurement::planning
        measurement::drr
        measurement::persistence
        GTest::gtest_main
)

measurement_apply_common_options(measurement_unit_tests)

include(GoogleTest)
gtest_discover_tests(measurement_unit_tests)
