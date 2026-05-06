add_library(measurement_vtk_adapter STATIC
    src/VtkMprResliceAdapter.cpp
    src/VtkPhysicsAdapter.cpp
)

add_library(measurement::vtk_adapter ALIAS measurement_vtk_adapter)

target_include_directories(measurement_vtk_adapter
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(measurement_vtk_adapter PUBLIC measurement::core measurement::mpr)

if(MEASUREMENT_HAVE_VTK)
    set(_measurement_vtk_libraries ${VTK_LIBRARIES})
    foreach(_measurement_vtk_target IN ITEMS
        VTK::CommonCore
        VTK::CommonDataModel
        VTK::ImagingCore
        VTK::ImagingGeneral
        VTK::RenderingCore
        VTK::RenderingOpenGL2
    )
        if(TARGET ${_measurement_vtk_target})
            list(APPEND _measurement_vtk_libraries ${_measurement_vtk_target})
        endif()
    endforeach()
    if(_measurement_vtk_libraries)
        list(REMOVE_DUPLICATES _measurement_vtk_libraries)
    endif()
    if(NOT VTK_INCLUDE_DIRS AND VTK_DIR)
        get_filename_component(_measurement_vtk_cmake_dir "${VTK_DIR}" DIRECTORY)
        get_filename_component(_measurement_vtk_lib_dir "${_measurement_vtk_cmake_dir}" DIRECTORY)
        get_filename_component(_measurement_vtk_root "${_measurement_vtk_lib_dir}" DIRECTORY)
        file(GLOB _measurement_vtk_include_dirs LIST_DIRECTORIES true "${_measurement_vtk_root}/include/vtk-*")
        list(APPEND VTK_INCLUDE_DIRS ${_measurement_vtk_include_dirs})
    endif()

    target_compile_definitions(measurement_vtk_adapter PRIVATE MEASUREMENT_HAVE_VTK=1)
    target_include_directories(measurement_vtk_adapter PRIVATE ${VTK_INCLUDE_DIRS})
    target_link_libraries(measurement_vtk_adapter PRIVATE ${_measurement_vtk_libraries})
else()
    target_compile_definitions(measurement_vtk_adapter PRIVATE MEASUREMENT_HAVE_VTK=0)
endif()

measurement_apply_common_options(measurement_vtk_adapter)
