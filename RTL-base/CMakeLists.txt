set(private_sources)
set(private_includes)
set(private_definitions)
set(private_compile_options)

ameba_list_append(private_sources
    app_example.c
    monitor.c
    inject.c
)

ameba_list_append(private_definitions
    USE_DWT_CYCCNT
)

ameba_add_internal_library(ameba_test
    p_SOURCES        ${private_sources}
    p_INCLUDES       ${private_includes}
    p_DEFINITIONS    ${private_definitions}
    p_COMPILE_OPTIONS ${private_compile_options}
)
