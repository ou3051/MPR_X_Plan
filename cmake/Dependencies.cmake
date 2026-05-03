function(measurement_find_dependencies)
    set(MEASUREMENT_HAVE_QT OFF CACHE BOOL "Qt6 was found" FORCE)
    set(MEASUREMENT_HAVE_VTK OFF CACHE BOOL "VTK was found" FORCE)
    set(MEASUREMENT_HAVE_DCMTK OFF CACHE BOOL "DCMTK was found" FORCE)
    set(MEASUREMENT_HAVE_CUDA OFF CACHE BOOL "CUDA toolkit was found" FORCE)

    find_package(Qt6 QUIET COMPONENTS Widgets)
    if(Qt6_FOUND)
        set(MEASUREMENT_HAVE_QT ON CACHE BOOL "Qt6 was found" FORCE)
    endif()

    if(MPR_USE_SYSTEM_VTK)
        find_package(VTK QUIET COMPONENTS
            CommonCore
            CommonDataModel
            ImagingCore
            ImagingGeneral
            RenderingCore
            RenderingOpenGL2
        )
        if(VTK_FOUND)
            set(MEASUREMENT_HAVE_VTK ON CACHE BOOL "VTK was found" FORCE)
        endif()
    endif()

    if(MPR_USE_SYSTEM_DCMTK)
        find_package(DCMTK QUIET)
        if(DCMTK_FOUND)
            set(MEASUREMENT_HAVE_DCMTK ON CACHE BOOL "DCMTK was found" FORCE)
        endif()
    endif()

    if(MPR_ENABLE_CUDA_DRR)
        find_package(CUDAToolkit QUIET)
        if(CUDAToolkit_FOUND)
            set(MEASUREMENT_HAVE_CUDA ON CACHE BOOL "CUDA toolkit was found" FORCE)
        endif()
    endif()
endfunction()

function(measurement_configure_gtest)
    if(MPR_USE_SYSTEM_GTEST)
        find_package(GTest QUIET CONFIG)
    endif()

    if(NOT TARGET GTest::gtest_main)
        set(_gtest_dir "${CMAKE_CURRENT_SOURCE_DIR}/third_party/googletest")
        if(EXISTS "${_gtest_dir}/CMakeLists.txt")
            set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
            add_subdirectory("${_gtest_dir}" "${CMAKE_CURRENT_BINARY_DIR}/third_party/googletest" EXCLUDE_FROM_ALL)
        endif()
    endif()

    if(NOT TARGET GTest::gtest_main)
        message(FATAL_ERROR
            "GoogleTest was not found. Install GTest or run scripts/fetch_dependencies.ps1 -Dependency googletest. "
            "CMake does not download dependencies implicitly.")
    endif()
endfunction()
