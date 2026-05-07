add_library(measurement_app_support STATIC
    DrrInteractionGeometry.cpp
    DrrInteractionGeometry.h
    DrrRenderRequest.cpp
    DrrRenderRequest.h
    InstrumentRenderModel.cpp
    InstrumentRenderModel.h
)

target_link_libraries(measurement_app_support
    PUBLIC
        measurement::core
        measurement::planning
)

target_include_directories(measurement_app_support
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(measurement::app_support ALIAS measurement_app_support)

measurement_apply_common_options(measurement_app_support)

add_executable(measurement_app
    MprSliceWidget.cpp
    MprPlanVerificationWindow.cpp
    MprPlanVerificationWindow.h
    MprPlanVerificationWindowMeasurements.cpp
    MprPlanVerificationWindowUi.cpp
    PlanSceneWidget.cpp
    XrayDisplayWidget.cpp
    XrayDisplayWidget.h
    main.cpp
)

target_link_libraries(measurement_app
    PRIVATE
        measurement::core
        measurement::dicom
        measurement::mpr
        measurement::planning
        measurement::drr
        measurement::vtk_adapter
        measurement::persistence
        measurement::app_support
        Qt6::Widgets
        Qt6::OpenGL
        Qt6::OpenGLWidgets
        ${VTK_LIBRARIES}
)

set(_measurement_app_vtk_modules ${VTK_LIBRARIES})
foreach(_measurement_app_vtk_target IN ITEMS
    VTK::GUISupportQt
    VTK::InteractionStyle
    VTK::FiltersSources
    VTK::RenderingVolume
    VTK::RenderingVolumeOpenGL2
)
    if(TARGET ${_measurement_app_vtk_target})
        target_link_libraries(measurement_app PRIVATE ${_measurement_app_vtk_target})
        list(APPEND _measurement_app_vtk_modules ${_measurement_app_vtk_target})
    endif()
endforeach()

if(COMMAND vtk_module_autoinit)
    list(REMOVE_DUPLICATES _measurement_app_vtk_modules)
    vtk_module_autoinit(
        TARGETS measurement_app
        MODULES ${_measurement_app_vtk_modules})
endif()

if(MEASUREMENT_HAVE_VTK)
    if(NOT VTK_INCLUDE_DIRS AND VTK_DIR)
        get_filename_component(_measurement_app_vtk_cmake_dir "${VTK_DIR}" DIRECTORY)
        get_filename_component(_measurement_app_vtk_lib_dir "${_measurement_app_vtk_cmake_dir}" DIRECTORY)
        get_filename_component(_measurement_app_vtk_root "${_measurement_app_vtk_lib_dir}" DIRECTORY)
        file(GLOB _measurement_app_vtk_include_dirs LIST_DIRECTORIES true "${_measurement_app_vtk_root}/include/vtk-*")
        list(APPEND VTK_INCLUDE_DIRS ${_measurement_app_vtk_include_dirs})
    endif()
    target_include_directories(measurement_app PRIVATE ${VTK_INCLUDE_DIRS})
endif()

measurement_apply_common_options(measurement_app)

if(WIN32)
    add_custom_command(
        TARGET measurement_app POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:measurement_app>
            $<TARGET_FILE_DIR:measurement_app>
        COMMAND_EXPAND_LISTS)

    foreach(_measurement_app_qt_runtime_target IN ITEMS Qt6::Core Qt6::Gui Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets)
        if(TARGET ${_measurement_app_qt_runtime_target})
            add_custom_command(
                TARGET measurement_app POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:${_measurement_app_qt_runtime_target}>
                    $<TARGET_FILE_DIR:measurement_app>
                COMMAND_EXPAND_LISTS)
        endif()
    endforeach()

    if(TARGET Qt6::QWindowsIntegrationPlugin)
        add_custom_command(
            TARGET measurement_app POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory
                $<TARGET_FILE_DIR:measurement_app>/platforms
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:Qt6::QWindowsIntegrationPlugin>
                $<TARGET_FILE_DIR:measurement_app>/platforms
            COMMAND_EXPAND_LISTS)
    endif()
endif()

if(MEASUREMENT_HAVE_CUDA AND CUDAToolkit_BIN_DIR)
    file(GLOB _measurement_app_cuda_runtime_dlls "${CUDAToolkit_BIN_DIR}/cudart64*.dll")
    if(_measurement_app_cuda_runtime_dlls)
        add_custom_command(
            TARGET measurement_app POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${_measurement_app_cuda_runtime_dlls}
                $<TARGET_FILE_DIR:measurement_app>
            COMMAND_EXPAND_LISTS)
    endif()
endif()

if(MEASUREMENT_HAVE_DCMTK AND DEFINED DCMTK_DCMDUMP_EXECUTABLE AND DCMTK_DCMDUMP_EXECUTABLE)
    get_filename_component(_measurement_app_dcmtk_bin_dir "${DCMTK_DCMDUMP_EXECUTABLE}" DIRECTORY)
    file(GLOB _measurement_app_dcmtk_runtime_dlls "${_measurement_app_dcmtk_bin_dir}/*.dll")
    if(_measurement_app_dcmtk_runtime_dlls)
        add_custom_command(
            TARGET measurement_app POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${_measurement_app_dcmtk_runtime_dlls}
                $<TARGET_FILE_DIR:measurement_app>
            COMMAND_EXPAND_LISTS)
    endif()
endif()
