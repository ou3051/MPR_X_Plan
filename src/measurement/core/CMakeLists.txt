add_library(measurement_core STATIC
    src/Geometry.cpp
    src/Instrument.cpp
    src/MeasurementAnnotation.cpp
    src/MeasurementStateMachine.cpp
    src/MeasurementStore.cpp
    src/MeasurementVisibility.cpp
    src/Volume.cpp
)

add_library(measurement::core ALIAS measurement_core)

target_include_directories(measurement_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

measurement_apply_common_options(measurement_core)
