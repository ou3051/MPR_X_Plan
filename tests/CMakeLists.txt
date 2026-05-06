add_executable(measurement_unit_tests
    unit/CoreCoordinateTests.cpp
    unit/InstrumentTests.cpp
    unit/CpuDrrTests.cpp
    unit/DrrInteractionGeometryTests.cpp
    unit/InstrumentRenderModelTests.cpp
    unit/DicomVolumeLoaderTests.cpp
    unit/MprResliceTests.cpp
    unit/ProjectManifestTests.cpp
)

target_link_libraries(measurement_unit_tests
    PRIVATE
        measurement::core
        measurement::dicom
        measurement::mpr
        measurement::planning
        measurement::drr
        measurement::vtk_adapter
        measurement::persistence
        measurement::app_support
        GTest::gtest_main
)

if(MEASUREMENT_HAVE_DCMTK)
    target_compile_definitions(measurement_unit_tests PRIVATE MEASUREMENT_HAVE_DCMTK=1)
    if(DEFINED DCMTK_DCMDUMP_EXECUTABLE AND DCMTK_DCMDUMP_EXECUTABLE)
        get_filename_component(_measurement_dcmtk_bin_dir "${DCMTK_DCMDUMP_EXECUTABLE}" DIRECTORY)
        set(_measurement_dump2dcm_executable "${_measurement_dcmtk_bin_dir}/dump2dcm.exe")
        file(GLOB _measurement_dcmtk_runtime_dlls "${_measurement_dcmtk_bin_dir}/*.dll")
        target_compile_definitions(
            measurement_unit_tests
            PRIVATE
                MEASUREMENT_DCMTK_DUMP2DCM_PATH="${_measurement_dump2dcm_executable}")
        if(_measurement_dcmtk_runtime_dlls)
            add_custom_command(
                TARGET measurement_unit_tests POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${_measurement_dcmtk_runtime_dlls}
                    $<TARGET_FILE_DIR:measurement_unit_tests>
                COMMAND_EXPAND_LISTS)
        endif()
    endif()
else()
    target_compile_definitions(measurement_unit_tests PRIVATE MEASUREMENT_HAVE_DCMTK=0)
endif()

if(MEASUREMENT_HAVE_VTK)
    target_compile_definitions(measurement_unit_tests PRIVATE MEASUREMENT_HAVE_VTK=1)
    target_include_directories(measurement_unit_tests PRIVATE ${VTK_INCLUDE_DIRS})
    target_link_libraries(measurement_unit_tests PRIVATE ${VTK_LIBRARIES})
else()
    target_compile_definitions(measurement_unit_tests PRIVATE MEASUREMENT_HAVE_VTK=0)
endif()

if(MEASUREMENT_HAVE_CUDA AND CUDAToolkit_BIN_DIR)
    file(GLOB _measurement_cuda_runtime_dlls "${CUDAToolkit_BIN_DIR}/cudart64*.dll")
    if(_measurement_cuda_runtime_dlls)
        add_custom_command(
            TARGET measurement_unit_tests POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${_measurement_cuda_runtime_dlls}
                $<TARGET_FILE_DIR:measurement_unit_tests>
            COMMAND_EXPAND_LISTS)
    endif()
endif()

measurement_apply_common_options(measurement_unit_tests)

include(GoogleTest)
gtest_discover_tests(measurement_unit_tests)
