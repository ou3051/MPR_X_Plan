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
    target_sources(measurement_drr PRIVATE src/CudaDrrKernel.cu)
    target_link_libraries(measurement_drr PRIVATE CUDA::cudart)
    target_compile_options(measurement_drr PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Wno-deprecated-gpu-targets>)
    target_compile_definitions(measurement_drr PRIVATE MEASUREMENT_HAVE_CUDA=1)
    set_property(TARGET measurement_drr PROPERTY CUDA_STANDARD 17)
    set_property(TARGET measurement_drr PROPERTY CUDA_STANDARD_REQUIRED ON)
    set_property(TARGET measurement_drr PROPERTY CUDA_RUNTIME_LIBRARY Shared)
else()
    target_compile_definitions(measurement_drr PRIVATE MEASUREMENT_HAVE_CUDA=0)
endif()

measurement_apply_common_options(measurement_drr)
