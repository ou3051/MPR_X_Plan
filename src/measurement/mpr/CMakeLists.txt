add_library(measurement_mpr STATIC
    src/MprResliceEngine.cpp
)

add_library(measurement::mpr ALIAS measurement_mpr)

target_include_directories(measurement_mpr
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(measurement_mpr PUBLIC measurement::core)
measurement_apply_common_options(measurement_mpr)
