set(checkasm_src
    checkasm.c
    msac.c
    pal.c
    refmvs.c)

set(checkasm_tmpl_src
    cdef.c
    filmgrain.c
    ipred.c
    itx.c
    loopfilter.c
    looprestoration.c
    mc.c)

if (ARCH_AARCH64)
  list(APPEND checkasm_src
      arm/checkasm_64.S)
elseif (ARCH_ARM)
  list(APPEND checkasm_src
      arm/checkasm_32.S)
elseif (ARCH_I386 OR ARCH_X86_64)
  if (CMAKE_ASM_NASM_COMPILER)
    set(x86_nasm_sources
        x86/checkasm.asm)
    list(APPEND checkasm_src
        ${x86_nasm_sources})
    set_source_files_properties(${x86_nasm_sources} PROPERTIES LANGUAGE ASM_NASM)
  endif()
endif()

foreach(bitdepth 8 16)
  add_library(checkasm_bitdepth_${bitdepth} OBJECT ${checkasm_tmpl_src})
  target_compile_definitions(checkasm_bitdepth_${bitdepth} PRIVATE -DBITDEPTH=${bitdepth})
  list(APPEND bitdepth_libraries checkasm_bitdepth_${bitdepth})
endforeach()

add_executable(checkasm ${checkasm_src})
target_link_libraries(checkasm LINK_PRIVATE ${bitdepth_libraries})
target_link_libraries(checkasm PRIVATE dav1d)
