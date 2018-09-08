px4_add_module(
    MODULE modules__simplex
    MAIN simplex
    STACK_MAIN 2000
    SRCS
        simplex.c
    DEPENDS
        platforms__common
    )