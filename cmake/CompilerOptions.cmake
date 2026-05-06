function(measurement_apply_common_options target_name)
    target_compile_features(${target_name} PUBLIC cxx_std_20)

    if(MSVC)
        target_compile_options(${target_name} PRIVATE
            $<$<COMPILE_LANGUAGE:CXX>:/W4>
            $<$<COMPILE_LANGUAGE:CXX>:/WX>
            $<$<COMPILE_LANGUAGE:CXX>:/permissive->
            $<$<COMPILE_LANGUAGE:CXX>:/Zc:preprocessor>
        )
    else()
        target_compile_options(${target_name} PRIVATE
            $<$<COMPILE_LANGUAGE:CXX>:-Wall>
            $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
            $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
            $<$<COMPILE_LANGUAGE:CXX>:-Werror>
        )
    endif()
endfunction()
