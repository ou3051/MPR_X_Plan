add_executable(measurement_app
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
        Qt6::Widgets
        ${VTK_LIBRARIES}
)

measurement_apply_common_options(measurement_app)
