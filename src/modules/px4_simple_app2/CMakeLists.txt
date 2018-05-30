px4_add_module(
    MODULE modules__px4_simple_app2
    MAIN px4_simple_app2
    STACK_MAIN 2000
    SRCS
        px4_simple_app2.c
    DEPENDS
        platforms__common
    )