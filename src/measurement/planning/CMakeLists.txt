add_library(measurement_planning STATIC
    src/InstrumentGeometry.cpp
)

add_library(measurement::planning ALIAS measurement_planning)

target_include_directories(measurement_planning
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(measurement_planning PUBLIC measurement::core)
measurement_apply_common_options(measurement_planning)
