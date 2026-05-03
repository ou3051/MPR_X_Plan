add_library(measurement_drr STATIC
    src/CpuDrrEngine.cpp
    src/CudaDrrEngine.cpp
)

add_library(measurement::drr ALIAS measurement_drr)

target_include_directories(measurement_drr
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(measurement_drr PUBLIC measurement::core)

if(MEASUREMENT_HAVE_CUDA)
    target_compile_definitions(measurement_drr PRIVATE MEASUREMENT_HAVE_CUDA=1)
else()
    target_compile_definitions(measurement_drr PRIVATE MEASUREMENT_HAVE_CUDA=0)
endif()

measurement_apply_common_options(measurement_drr)
