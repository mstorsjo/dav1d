include_directories(.)

set(dav1d_src
    dav1d.c
    dav1d_cli_parse.c
    input/input.c
    input/annexb.c
    input/ivf.c
    input/section5.c
    output/md5.c
    output/null.c
    output/output.c
    output/y4m2.c
    output/yuv.c)

if (WIN32)
  list(APPEND dav1d_src
      compat/getopt.c)
endif()

add_executable(dav1d_cli ${dav1d_src})

target_link_libraries(dav1d_cli PRIVATE dav1d)

install(TARGETS dav1d)
