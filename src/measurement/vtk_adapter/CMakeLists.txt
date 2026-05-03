add_library(measurement_vtk_adapter STATIC
    src/VtkPhysicsAdapter.cpp
)

add_library(measurement::vtk_adapter ALIAS measurement_vtk_adapter)

target_include_directories(measurement_vtk_adapter
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(measurement_vtk_adapter PUBLIC measurement::core)

if(MEASUREMENT_HAVE_VTK)
    target_compile_definitions(measurement_vtk_adapter PRIVATE MEASUREMENT_HAVE_VTK=1)
    target_link_libraries(measurement_vtk_adapter PRIVATE ${VTK_LIBRARIES})
else()
    target_compile_definitions(measurement_vtk_adapter PRIVATE MEASUREMENT_HAVE_VTK=0)
endif()

measurement_apply_common_options(measurement_vtk_adapter)
