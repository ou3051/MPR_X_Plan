add_library(measurement_persistence STATIC
    src/ProjectManifest.cpp
)

add_library(measurement::persistence ALIAS measurement_persistence)

target_include_directories(measurement_persistence
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(measurement_persistence PUBLIC measurement::core)
measurement_apply_common_options(measurement_persistence)
