function(measurement_find_dependencies)
    set(MEASUREMENT_HAVE_QT OFF CACHE BOOL "Qt6 was found" FORCE)
    set(MEASUREMENT_HAVE_VTK OFF CACHE BOOL "VTK was found" FORCE)
    set(MEASUREMENT_HAVE_DCMTK OFF CACHE BOOL "DCMTK was found" FORCE)
    set(MEASUREMENT_HAVE_DCMTK_RUNTIME OFF CACHE BOOL "DCMTK command-line runtime was found" FORCE)
    set(MEASUREMENT_HAVE_CUDA OFF CACHE BOOL "CUDA toolkit was found" FORCE)
    set(MPR_DCMTK_ROOT "" CACHE PATH "Optional root path for a local DCMTK installation")

    find_package(Qt6 QUIET COMPONENTS Widgets OpenGL OpenGLWidgets)
    if(Qt6_FOUND)
        set(MEASUREMENT_HAVE_QT ON CACHE BOOL "Qt6 was found" FORCE)
    endif()

    if(MPR_USE_SYSTEM_VTK)
        find_package(VTK QUIET COMPONENTS
            CommonCore
            CommonDataModel
            ImagingCore
            ImagingGeneral
            FiltersSources
            InteractionStyle
            RenderingCore
            RenderingOpenGL2
            RenderingVolume
            RenderingVolumeOpenGL2
            GUISupportQt
        )
        if(VTK_FOUND)
            set(MEASUREMENT_HAVE_VTK ON CACHE BOOL "VTK was found" FORCE)
        endif()
    endif()

    if(MPR_USE_SYSTEM_DCMTK)
        set(_dcmtk_roots)
        if(MPR_DCMTK_ROOT)
            if(IS_DIRECTORY "${MPR_DCMTK_ROOT}")
                file(GLOB _dcmtk_build_roots LIST_DIRECTORIES true "${MPR_DCMTK_ROOT}/dcmtk-*-build")
                file(GLOB _dcmtk_install_roots LIST_DIRECTORIES true "${MPR_DCMTK_ROOT}/dcmtk-*-install")
                list(APPEND _dcmtk_roots ${_dcmtk_build_roots} ${_dcmtk_install_roots})
            endif()
            list(APPEND _dcmtk_roots "${MPR_DCMTK_ROOT}")
        endif()

        foreach(_dcmtk_root IN LISTS _dcmtk_roots)
            list(APPEND CMAKE_PREFIX_PATH "${_dcmtk_root}")
            list(APPEND CMAKE_PREFIX_PATH "${_dcmtk_root}/cmake")
            list(APPEND CMAKE_PREFIX_PATH "${_dcmtk_root}/lib/cmake/dcmtk")
        endforeach()

        if(_dcmtk_roots)
            list(GET _dcmtk_roots 0 _dcmtk_first_root)
        else()
            set(_dcmtk_first_root "${MPR_DCMTK_ROOT}")
        endif()

        find_package(DCMTK QUIET CONFIG)
        if(NOT DCMTK_FOUND)
            find_package(DCMTK QUIET)
        endif()
        if(DCMTK_FOUND)
            set(MEASUREMENT_HAVE_DCMTK ON CACHE BOOL "DCMTK was found" FORCE)
            if(TARGET DCMTK::dcmdata)
                set(MEASUREMENT_DCMTK_LIBRARIES "DCMTK::dcmdata" CACHE STRING "DCMTK link libraries" FORCE)
            else()
                set(MEASUREMENT_DCMTK_LIBRARIES "${DCMTK_LIBRARIES}" CACHE STRING "DCMTK link libraries" FORCE)
            endif()
        else()
            find_path(DCMTK_INCLUDE_DIR
                NAMES dcmtk/dcmdata/dctk.h
                HINTS ${_dcmtk_roots}
                PATH_SUFFIXES include
            )
            find_library(DCMTK_DCMDATA_LIBRARY
                NAMES dcmdata
                HINTS ${_dcmtk_roots}
                PATH_SUFFIXES lib lib64 lib/Debug lib/Release
            )
            find_library(DCMTK_OFSTD_LIBRARY
                NAMES ofstd
                HINTS ${_dcmtk_roots}
                PATH_SUFFIXES lib lib64 lib/Debug lib/Release
            )
            find_library(DCMTK_OFLOG_LIBRARY
                NAMES oflog
                HINTS ${_dcmtk_roots}
                PATH_SUFFIXES lib lib64 lib/Debug lib/Release
            )
            if(DCMTK_INCLUDE_DIR AND DCMTK_DCMDATA_LIBRARY AND DCMTK_OFSTD_LIBRARY)
                set(DCMTK_INCLUDE_DIRS "${DCMTK_INCLUDE_DIR}" CACHE PATH "DCMTK include directories" FORCE)
                set(DCMTK_LIBRARIES "${DCMTK_DCMDATA_LIBRARY};${DCMTK_OFSTD_LIBRARY};${DCMTK_OFLOG_LIBRARY}" CACHE STRING "DCMTK libraries" FORCE)
                set(MEASUREMENT_DCMTK_LIBRARIES "${DCMTK_LIBRARIES}" CACHE STRING "DCMTK link libraries" FORCE)
                set(MEASUREMENT_HAVE_DCMTK ON CACHE BOOL "DCMTK was found" FORCE)
            endif()
        endif()

        find_program(DCMTK_DCMDUMP_EXECUTABLE
            NAMES dcmdump
            HINTS ${_dcmtk_roots}
            PATH_SUFFIXES bin
        )
        if(DCMTK_DCMDUMP_EXECUTABLE)
            set(MEASUREMENT_HAVE_DCMTK_RUNTIME ON CACHE BOOL "DCMTK command-line runtime was found" FORCE)
        endif()

        if(MEASUREMENT_HAVE_DCMTK)
            message(STATUS "DCMTK development files found.")
        elseif(MEASUREMENT_HAVE_DCMTK_RUNTIME)
            message(WARNING "DCMTK runtime tools were found at '${_dcmtk_first_root}', but headers/libraries or DCMTKConfig.cmake were not found. DICOM import remains disabled for this build.")
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
