include_guard(GLOBAL)

function(silk_debug_warnings target)
    set(_silk_debug_flags_gnu_clang
        -Wall
        -Wextra
        -Wpedantic
        -fdiagnostics-color
        -Wno-sign-compare # libigl emits signed/unsigned comparisons
        -Wno-ctor-dtor-privacy 
    )

    set(_silk_debug_flags_msvc
        /W4
        /permissive-
        /wd4244 # signed/unsigned conversions
    )

    set(_silk_debug_flags_nvcc
      -diag-suppress 2361 # signed unsiged narrowing
    )

    target_compile_options(
        ${target}
        PRIVATE
            $<$<AND:$<CONFIG:Debug>,$<COMPILE_LANG_AND_ID:CXX,Clang,GNU>>:${_silk_debug_flags_gnu_clang}>
            $<$<AND:$<CONFIG:Debug>,$<COMPILE_LANG_AND_ID:CXX,MSVC>>:${_silk_debug_flags_msvc}>
    )

    unset(_silk_debug_flags_gnu_clang)
    unset(_silk_debug_flags_msvc)
    unset(_silk_debug_flags_nvcc)
endfunction()
