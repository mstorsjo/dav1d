set(dav1d_src
    cdf.c
    cpu.c
    ctx.c
    data.c
    decode.c
    dequant_tables.c
    getbits.c
    intra_edge.c
    itx_1d.c
    lf_mask.c
    lib.c
    log.c
    mem.c
    msac.c
    obu.c
    pal.c
    picture.c
    qm.c
    ref.c
    refmvs.c
    scan.c
    tables.c
    thread_task.c
    warpmv.c
    wedge.c)

if (WIN32)
  list(APPEND dav1d_src
      win32/thread.c)
endif()

set(dav1d_tmpl_src
    cdef_apply_tmpl.c
    cdef_tmpl.c
    fg_apply_tmpl.c
    filmgrain_tmpl.c
    ipred_prepare_tmpl.c
    ipred_tmpl.c
    itx_tmpl.c
    lf_apply_tmpl.c
    loopfilter_tmpl.c
    looprestoration_tmpl.c
    lr_apply_tmpl.c
    mc_tmpl.c
    recon_tmpl.c)

if (ARCH_AARCH64)
  list(APPEND dav1d_src
      arm/cpu.c
      arm/64/itx.S
      arm/64/looprestoration_common.S
      arm/64/msac.S
      arm/64/refmvs.S
      arm/64/cdef.S
      arm/64/filmgrain.S
      arm/64/ipred.S
      arm/64/loopfilter.S
      arm/64/looprestoration.S
      arm/64/mc.S
      arm/64/mc_dotprod.S
      arm/64/cdef16.S
      arm/64/filmgrain16.S
      arm/64/ipred16.S
      arm/64/itx16.S
      arm/64/loopfilter16.S
      arm/64/looprestoration16.S
      arm/64/mc16.S
      arm/64/mc16_sve.S)
elseif (ARCH_ARM)
  list(APPEND dav1d_src
      arm/cpu.c
      arm/32/itx.S
      arm/32/looprestoration_common.S
      arm/32/msac.S
      arm/32/refmvs.S
      arm/32/cdef.S
      arm/32/filmgrain.S
      arm/32/ipred.S
      arm/32/loopfilter.S
      arm/32/looprestoration.S
      arm/32/mc.S
      arm/32/cdef16.S
      arm/32/filmgrain16.S
      arm/32/ipred16.S
      arm/32/itx16.S
      arm/32/loopfilter16.S
      arm/32/looprestoration16.S
      arm/32/mc16.S)
elseif (ARCH_I386 OR ARCH_X86_64)
  list(APPEND dav1d_src
      x86/cpu.c)
  if (CMAKE_ASM_NASM_COMPILER)
    set(x86_nasm_sources
        x86/cpuid.asm
        x86/msac.asm
        x86/pal.asm
        x86/refmvs.asm
        x86/itx_avx512.asm
        x86/cdef_avx2.asm
        x86/itx_avx2.asm
        x86/cdef_sse.asm
        x86/itx_sse.asm
        x86/cdef_avx512.asm
        x86/filmgrain_avx512.asm
        x86/ipred_avx512.asm
        x86/loopfilter_avx512.asm
        x86/looprestoration_avx512.asm
        x86/mc_avx512.asm
        x86/filmgrain_avx2.asm
        x86/ipred_avx2.asm
        x86/loopfilter_avx2.asm
        x86/looprestoration_avx2.asm
        x86/mc_avx2.asm
        x86/filmgrain_sse.asm
        x86/ipred_sse.asm
        x86/loopfilter_sse.asm
        x86/looprestoration_sse.asm
        x86/mc_sse.asm
        x86/cdef16_avx512.asm
        x86/filmgrain16_avx512.asm
        x86/ipred16_avx512.asm
        x86/itx16_avx512.asm
        x86/loopfilter16_avx512.asm
        x86/looprestoration16_avx512.asm
        x86/mc16_avx512.asm
        x86/cdef16_avx2.asm
        x86/filmgrain16_avx2.asm
        x86/ipred16_avx2.asm
        x86/itx16_avx2.asm
        x86/loopfilter16_avx2.asm
        x86/looprestoration16_avx2.asm
        x86/mc16_avx2.asm
        x86/cdef16_sse.asm
        x86/filmgrain16_sse.asm
        x86/ipred16_sse.asm
        x86/itx16_sse.asm
        x86/loopfilter16_sse.asm
        x86/looprestoration16_sse.asm
        x86/mc16_sse.asm)
    list(APPEND dav1d_src
        ${x86_nasm_sources})
    set_source_files_properties(${x86_nasm_sources} PROPERTIES LANGUAGE ASM_NASM)
  endif()
endif()

foreach(bitdepth 8 16)
  add_library(dav1d_bitdepth_${bitdepth} OBJECT ${dav1d_tmpl_src})
  target_compile_definitions(dav1d_bitdepth_${bitdepth} PRIVATE -DBITDEPTH=${bitdepth})
  list(APPEND bitdepth_libraries dav1d_bitdepth_${bitdepth})
endforeach()

add_library(dav1d ${dav1d_src})
target_link_libraries(dav1d LINK_PRIVATE ${bitdepth_libraries})
