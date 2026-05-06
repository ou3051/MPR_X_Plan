cmake_minimum_required(VERSION 3.26)

project(MeasurementMPR
    VERSION 0.1.0
    DESCRIPTION "Orthopedic MPR planning and X-ray simulation"
    LANGUAGES CXX
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(CompilerOptions)
include(Dependencies)

option(MPR_BUILD_APP "Build the Qt desktop application when Qt/VTK are available" ON)
option(MPR_BUILD_TESTS "Build unit and validation tests" ON)
option(MPR_USE_SYSTEM_DCMTK "Prefer system DCMTK package" ON)
option(MPR_USE_SYSTEM_GTEST "Prefer system GoogleTest package" ON)
option(MPR_USE_SYSTEM_VTK "Prefer system VTK package" ON)
option(MPR_ENABLE_CUDA_DRR "Enable CUDA-backed DRR implementation" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if(MSVC)
    add_compile_options(
        $<$<COMPILE_LANGUAGE:CXX>:/utf-8>
        $<$<COMPILE_LANGUAGE:CXX>:/permissive->
        $<$<COMPILE_LANGUAGE:CXX>:/Zc:__cplusplus>
    )
endif()

measurement_find_dependencies()

if(MEASUREMENT_HAVE_CUDA)
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    set(CMAKE_CUDA_EXTENSIONS OFF)
endif()

add_subdirectory(src/measurement/core)
add_subdirectory(src/measurement/dicom)
add_subdirectory(src/measurement/mpr)
add_subdirectory(src/measurement/planning)
add_subdirectory(src/measurement/drr)
add_subdirectory(src/measurement/vtk_adapter)
add_subdirectory(src/measurement/persistence)

if(MPR_BUILD_APP)
    if(MEASUREMENT_HAVE_QT AND MEASUREMENT_HAVE_VTK)
        add_subdirectory(src/app)
    else()
        message(WARNING "MPR_BUILD_APP is ON, but Qt6 and/or VTK were not found. The app target is disabled; core libraries still build.")
    endif()
endif()

if(MPR_BUILD_TESTS)
    enable_testing()
    measurement_configure_gtest()
    add_subdirectory(tests)
endif()
