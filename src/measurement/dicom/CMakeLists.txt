add_library(measurement_dicom STATIC
    src/DicomVolumeLoader.cpp
)

add_library(measurement::dicom ALIAS measurement_dicom)

target_include_directories(measurement_dicom
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(measurement_dicom PUBLIC measurement::core)

if(MEASUREMENT_HAVE_DCMTK)
    target_compile_definitions(measurement_dicom PRIVATE MEASUREMENT_HAVE_DCMTK=1)
    if(NOT MEASUREMENT_DCMTK_LIBRARIES)
        message(FATAL_ERROR "MEASUREMENT_HAVE_DCMTK is ON but no DCMTK link libraries were configured.")
    endif()
    target_include_directories(measurement_dicom PRIVATE ${DCMTK_INCLUDE_DIRS})
    target_link_libraries(measurement_dicom PRIVATE ${MEASUREMENT_DCMTK_LIBRARIES})
else()
    target_compile_definitions(measurement_dicom PRIVATE MEASUREMENT_HAVE_DCMTK=0)
endif()

measurement_apply_common_options(measurement_dicom)
