/*
 * Copyright © 2024, VideoLAN and dav1d authors
 * Copyright © 2024, Janne Grunau
 * Copyright © 2024, Martin Storsjo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "src/arm/asm.S"
#include "util.S"


#if HAVE_DOTPROD
ENABLE_DOTPROD

// No spaces in these expressions, due to gas-preprocessor. It is translated by
// -1 to save the negative offset at getting the address of `mc_subpel_filters`.
#define REGULAR1        (((0*15-1)<<7)|(3*15-1))
#define SMOOTH1         (((1*15-1)<<7)|(4*15-1))
#define SHARP1          (((2*15-1)<<7)|(3*15-1))

#define FUNC_ALIGN      2
#define JUMP_ALIGN      2
#define LOOP_ALIGN      2


        .text
// Lookup table used to help conversion of shifted 32-bit values to 8-bit.
        .align 4
L(hv_tbl_neon_dotprod):
        .byte  1,  2,  5,  6,   9, 10, 13, 14,  17, 18, 21, 22,  25, 26, 29, 30

// Shuffle indices to permute horizontal samples in preparation for input to
// SDOT instructions. The 8-tap horizontal convolution uses sample indices in the
// interval of [-3, 4] relative to the current sample position. We load samples
// from index value -4 to keep loads word aligned, so the shuffle bytes are
// translated by 1 to handle this.
        .align 4
L(h_tbl_neon_dotprod):
        .byte  1,  2,  3,  4,   2,  3,  4,  5,   3,  4,  5,  6,   4,  5,  6,  7
        .byte  5,  6,  7,  8,   6,  7,  8,  9,   7,  8,  9, 10,   8,  9, 10, 11
        .byte  9, 10, 11, 12,  10, 11, 12, 13,  11, 12, 13, 14,  12, 13, 14, 15
        .byte 13, 14, 15, 16,  14, 15, 16, 17,  15, 16, 17, 18,  16, 17, 18, 19

// Vertical convolutions are also using SDOT instructions, where a 128-bit
// register contains a transposed 4x4 matrix of values. Subsequent iterations of
// the vertical convolution can reuse the 3x4 sub-matrix from the previous loop
// iteration. These shuffle indices shift and merge this 4x4 matrix with the
// values of a new line.
        .align 4
L(v_tbl_neon_dotprod):
        .byte  1,  2,  3, 16,   5,  6,  7, 20,   9, 10, 11, 24,  13, 14, 15, 28
        .byte  1,  2,  3, 16,   5,  6,  7, 17,   9, 10, 11, 18,  13, 14, 15, 19
        .byte  1,  2,  3, 20,   5,  6,  7, 21,   9, 10, 11, 22,  13, 14, 15, 23
        .byte  1,  2,  3, 24,   5,  6,  7, 25,   9, 10, 11, 26,  13, 14, 15, 27
        .byte  1,  2,  3, 28,   5,  6,  7, 29,   9, 10, 11, 30,  13, 14, 15, 31


// tmp(x0), src(x1), src_stride(x2), w(w3), h(w4), mx(w5), my(w6)
function prep_8tap_sharp_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, SHARP1
        mov             x9, SHARP1
        b               prep_8tap_neon_dotprod
endfunc

function prep_8tap_sharp_smooth_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, SHARP1
        mov             x9, SMOOTH1
        b               prep_8tap_neon_dotprod
endfunc

function prep_8tap_sharp_regular_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, SHARP1
        mov             x9, REGULAR1
        b               prep_8tap_neon_dotprod
endfunc

function prep_8tap_smooth_sharp_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, SMOOTH1
        mov             x9, SHARP1
        b               prep_8tap_neon_dotprod
endfunc

function prep_8tap_smooth_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, SMOOTH1
        mov             x9, SMOOTH1
        b               prep_8tap_neon_dotprod
endfunc

function prep_8tap_smooth_regular_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, SMOOTH1
        mov             x9, REGULAR1
        b               prep_8tap_neon_dotprod
endfunc

function prep_8tap_regular_sharp_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, REGULAR1
        mov             x9, SHARP1
        b               prep_8tap_neon_dotprod
endfunc

function prep_8tap_regular_smooth_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, REGULAR1
        mov             x9, SMOOTH1
        b               prep_8tap_neon_dotprod
endfunc

function prep_8tap_regular_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, REGULAR1
        mov             x9, REGULAR1
        // b            prep_8tap_neon_dotprod  // fallthrough
endfunc


// tmp(x0), src(x1), src_stride(x2), w(w3), h(w4), mx(w5), my(w6)
function prep_8tap_neon_dotprod, align=FUNC_ALIGN
        clz             w12, w3
        mov             w10,  #0x4081   // (1 << 14) | (1 << 7) | (1 << 0)
        sub             w12, w12, #24   // for jump tables
        movrel          x11, X(mc_subpel_filters)
        cbnz            w5, 1f          // prep HV or H
        cbz             w6, L(prep_neon_dotprod)

        // prep V cases
        madd            w6, w6, w10, w9
        mov             w12, 0x2002             // FILTER_WEIGHT * 128 + rounding
        sub             x1, x1, x2
        dup             v4.4s, w12
        ldr             q6, L(v_tbl_neon_dotprod)
        ubfx            w10, w6, #7, #7
        and             w6, w6, #0x7F
        ldr             q28, L(v_tbl_neon_dotprod + 16)
        cmp             w4, #4
        csel            w6, w6, w10, le
        sub             x1, x1, x2, lsl #1      // src - src_stride * 3
        ldr             q29, L(v_tbl_neon_dotprod + 32)
        add             x6, x11, x6, lsl #3     // subpel V filter address
        movi            v5.16b, #128
        ldr             d7, [x6]
        cmp             w3, #8
        b.eq            80f
        b.lt            40f

        // .align JUMP_ALIGN    // fallthrough
160:    // prep V - 16xN+
        ldr             q30, L(v_tbl_neon_dotprod + 48)
        ldr             q31, L(v_tbl_neon_dotprod + 64)
        lsl             w8, w3, #1

        .align LOOP_ALIGN
161:
        mov             x6, x1
        mov             x5, x0
        mov             x7, x4

        // 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
        // 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f
        // 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f
        // 30 31 32 33 34 35 36 37 38 39 3a 3b 3c 3d 3e 3f
        ldr             q16, [x6]
        ldr             q17, [x6, x2]
        add             x6, x6, x2, lsl #1
        ldr             q18, [x6]
        ldr             q19, [x6, x2]
        add             x6, x6, x2, lsl #1

        // 00 10 01 11 02 12 03 13 04 14 05 15 06 16 07 17
        // 08 18 09 19 0a 1a 0b 1b 0c 1c 0d 1d 0e 1e 0f 1f
        // 20 30 21 31 22 32 23 33 24 34 25 35 26 36 27 37
        // 28 38 29 39 2a 3a 2b 3b 2c 3c 2d 3d 2e 3e 2f 3f
        zip1            v0.16b, v16.16b, v17.16b
        zip2            v1.16b, v16.16b, v17.16b
        zip1            v2.16b, v18.16b, v19.16b
        zip2            v3.16b, v18.16b, v19.16b

        // 40 41 42 43 44 45 46 47 48 49 4a 4b 4c 4d 4e 4f
        // 50 51 52 53 54 55 56 57 58 59 5a 5b 5c 5d 5e 5f
        // 60 61 62 63 64 65 66 67 68 69 6a 6b 6c 6d 6e 6f
        // 70 71 72 73 74 75 76 77 78 79 7a 7b 7c 7d 7e 7f
        ldr             q20, [x6]
        ldr             q21, [x6, x2]
        add             x6, x6, x2, lsl #1
        ldr             q22, [x6]
        ldr             q23, [x6, x2]
        add             x6, x6, x2, lsl #1

        // 40 50 41 51 42 52 43 53 44 54 45 55 46 56 47 57
        // 48 58 49 59 4a 5a 4b 5b 4c 5c 4d 5d 4e 5e 4f 5f
        // 60 70 61 71 62 72 63 73 64 74 65 75 66 76 67 77
        // 68 78 69 79 6a 7a 6b 7b 6c 7c 6d 7d 6e 7e 6f 7f
        zip1            v18.16b, v20.16b, v21.16b
        zip2            v21.16b, v20.16b, v21.16b
        zip1            v24.16b, v22.16b, v23.16b
        zip2            v27.16b, v22.16b, v23.16b

        // 00 10 20 30  01 11 21 31  02 12 22 32  03 13 23 33
        // 04 14 24 34  05 15 25 35  06 16 26 36  07 17 27 37
        // 08 18 28 38  09 19 29 39  0a 1a 2a 3a  0b 1b 2b 3b
        // 0c 1c 2c 3c  0d 1d 2d 3d  0e 1e 2e 3e  0f 1f 2f 3f
        zip1            v16.8h, v0.8h, v2.8h
        zip2            v19.8h, v0.8h, v2.8h
        zip1            v22.8h, v1.8h, v3.8h
        zip2            v25.8h, v1.8h, v3.8h

        // 40 50 60 70  41 51 61 71  42 52 62 72  43 53 63 73
        // 44 54 64 74  45 55 65 75  46 56 66 76  47 57 67 77
        // 48 58 68 78  49 59 69 79  4a 5a 6a 7a  4b 5b 6b 7b
        // 4c 5c 6c 7c  4d 5d 6d 7d  4e 5e 6e 7e  4f 5f 6f 7f
        zip1            v17.8h, v18.8h, v24.8h
        zip2            v20.8h, v18.8h, v24.8h
        zip1            v23.8h, v21.8h, v27.8h
        zip2            v26.8h, v21.8h, v27.8h

        sub             v16.16b, v16.16b, v5.16b
        sub             v19.16b, v19.16b, v5.16b
        sub             v22.16b, v22.16b, v5.16b
        sub             v25.16b, v25.16b, v5.16b

        sub             v17.16b, v17.16b, v5.16b
        sub             v20.16b, v20.16b, v5.16b
        sub             v23.16b, v23.16b, v5.16b
        sub             v26.16b, v26.16b, v5.16b

        .align LOOP_ALIGN
16:
        ld1             {v27.16b}, [x6], x2

        mov             v0.16b, v4.16b
        mov             v1.16b, v4.16b
        mov             v2.16b, v4.16b
        mov             v3.16b, v4.16b

        sub             v18.16b, v27.16b, v5.16b
        sub             v21.16b, v27.16b, v5.16b
        sub             v24.16b, v27.16b, v5.16b
        sub             v27.16b, v27.16b, v5.16b

        sdot            v0.4s, v16.16b, v7.4b[0]
        sdot            v1.4s, v19.16b, v7.4b[0]
        sdot            v2.4s, v22.16b, v7.4b[0]
        sdot            v3.4s, v25.16b, v7.4b[0]

        tbl             v16.16b, {v16.16b, v17.16b}, v6.16b
        tbl             v19.16b, {v19.16b, v20.16b}, v6.16b
        tbl             v22.16b, {v22.16b, v23.16b}, v6.16b
        tbl             v25.16b, {v25.16b, v26.16b}, v6.16b

        sdot            v0.4s, v17.16b, v7.4b[1]
        sdot            v1.4s, v20.16b, v7.4b[1]
        sdot            v2.4s, v23.16b, v7.4b[1]
        sdot            v3.4s, v26.16b, v7.4b[1]

        tbl             v17.16b, {v17.16b, v18.16b}, v28.16b
        tbl             v20.16b, {v20.16b, v21.16b}, v29.16b
        tbl             v23.16b, {v23.16b, v24.16b}, v30.16b
        tbl             v26.16b, {v26.16b, v27.16b}, v31.16b

        subs            w7, w7, #1
        uzp1            v0.8h, v0.8h, v1.8h
        uzp1            v1.8h, v2.8h, v3.8h
        sshr            v0.8h, v0.8h, #2
        sshr            v1.8h, v1.8h, #2

        st1             {v0.8h, v1.8h}, [x5], x8
        b.gt            16b

        add             x0, x0, #32
        add             x1, x1, #16
        subs            w3, w3, #16
        b.gt            161b
        ret

        .align JUMP_ALIGN
80:     // prep V - 8xN
        // 00 01 02 03 04 05 06 07
        // 10 11 12 13 14 15 16 17
        // 20 21 22 23 24 25 26 27
        // 30 31 32 33 34 35 36 37
        ldr             d16, [x1]
        ldr             d17, [x1, x2]
        add             x1, x1, x2, lsl #1
        ldr             d18, [x1]
        ldr             d19, [x1, x2]
        add             x1, x1, x2, lsl #1

        // 40 41 42 43 44 45 46 47
        // 50 51 52 53 54 55 56 57
        // 60 61 62 63 64 65 66 67
        // 70 71 72 73 74 75 76 77
        ldr             d20, [x1]
        ldr             d21, [x1, x2]
        add             x1, x1, x2, lsl #1
        ldr             d22, [x1]
        ldr             d23, [x1, x2]
        add             x1, x1, x2, lsl #1

        // 00 10 01 11 02 12 03 13 04 14 05 15 06 16 07 17
        // 20 30 21 31 22 32 23 33 24 34 25 35 26 36 27 37
        // 40 50 41 51 42 52 43 53 44 54 45 55 46 56 47 57
        // 60 70 61 71 62 72 63 73 64 74 65 75 66 76 67 77
        zip1            v0.16b, v16.16b, v17.16b
        zip1            v2.16b, v18.16b, v19.16b
        zip1            v18.16b, v20.16b, v21.16b
        zip1            v24.16b, v22.16b, v23.16b

        // 00 10 20 30  01 11 21 31  02 12 22 32  03 13 23 33
        // 04 14 24 34  05 15 25 35  06 16 26 36  07 17 27 37
        // 40 50 60 70  41 51 61 71  42 52 62 72  43 53 63 73
        // 44 54 64 74  45 55 65 75  46 56 66 76  47 57 67 77
        zip1            v16.8h, v0.8h, v2.8h
        zip2            v19.8h, v0.8h, v2.8h
        zip1            v17.8h, v18.8h, v24.8h
        zip2            v20.8h, v18.8h, v24.8h

        sub             v16.16b, v16.16b, v5.16b
        sub             v19.16b, v19.16b, v5.16b
        sub             v17.16b, v17.16b, v5.16b
        sub             v20.16b, v20.16b, v5.16b

        .align LOOP_ALIGN
8:
        ldr             d21, [x1]
        ldr             d27, [x1, x2]
        add             x1, x1, x2, lsl #1

        mov             v0.16b, v4.16b
        mov             v1.16b, v4.16b
        mov             v2.16b, v4.16b
        mov             v3.16b, v4.16b

        sub             v18.16b, v21.16b, v5.16b
        sub             v21.16b, v21.16b, v5.16b
        sub             v24.16b, v27.16b, v5.16b
        sub             v27.16b, v27.16b, v5.16b

        tbl             v22.16b, {v16.16b, v17.16b}, v6.16b
        tbl             v25.16b, {v19.16b, v20.16b}, v6.16b
        tbl             v23.16b, {v17.16b, v18.16b}, v28.16b
        tbl             v26.16b, {v20.16b, v21.16b}, v29.16b

        sdot            v0.4s, v16.16b, v7.4b[0]
        sdot            v0.4s, v17.16b, v7.4b[1]
        sdot            v1.4s, v19.16b, v7.4b[0]
        sdot            v1.4s, v20.16b, v7.4b[1]

        tbl             v16.16b, {v22.16b, v23.16b}, v6.16b
        tbl             v19.16b, {v25.16b, v26.16b}, v6.16b
        tbl             v17.16b, {v23.16b, v24.16b}, v28.16b
        tbl             v20.16b, {v26.16b, v27.16b}, v29.16b

        sdot            v2.4s, v22.16b, v7.4b[0]
        sdot            v2.4s, v23.16b, v7.4b[1]
        sdot            v3.4s, v25.16b, v7.4b[0]
        sdot            v3.4s, v26.16b, v7.4b[1]

        subs            w4, w4, #2
        uzp1            v0.8h, v0.8h, v1.8h
        uzp1            v2.8h, v2.8h, v3.8h
        sshr            v0.8h, v0.8h, #2
        sshr            v1.8h, v2.8h, #2

        stp             q0, q1, [x0], #32
        b.gt            8b
        ret

        .align JUMP_ALIGN
40:     // prep V - 4xN
        // 00 01 02 03
        // 10 11 12 13
        // 20 21 22 23
        // 30 31 32 33
        ldr             s16, [x1]
        ldr             s17, [x1, x2]
        add             x1, x1, x2, lsl #1
        ldr             s18, [x1]
        ldr             s19, [x1, x2]
        add             x1, x1, x2, lsl #1

        // 40 41 42 43
        // 50 51 52 53
        // 60 61 62 63
        // 70 71 72 73
        ldr             s20, [x1]
        ldr             s21, [x1, x2]
        add             x1, x1, x2, lsl #1
        ldr             s22, [x1]
        ldr             s23, [x1, x2]
        add             x1, x1, x2, lsl #1

        // 00 10 01 11 02 12 03 13
        // 20 30 21 31 22 32 23 33
        // 40 50 41 51 42 52 43 53
        // 60 70 61 71 62 72 63 73
        zip1            v0.8b, v16.8b, v17.8b
        zip1            v2.8b, v18.8b, v19.8b
        zip1            v18.8b, v20.8b, v21.8b
        zip1            v24.8b, v22.8b, v23.8b

        // 00 10 20 30  01 11 21 31  02 12 22 32  03 13 23 33
        // 40 50 60 70  41 51 61 71  42 52 62 72  43 53 63 73
        zip1            v16.8h, v0.8h, v2.8h
        zip1            v17.8h, v18.8h, v24.8h

        sub             v16.16b, v16.16b, v5.16b
        sub             v17.16b, v17.16b, v5.16b

        .align LOOP_ALIGN
4:
        ldr             s18, [x1]
        ldr             s21, [x1, x2]
        add             x1, x1, x2, lsl #1

        mov             v0.16b, v4.16b
        mov             v1.16b, v4.16b

        sub             v18.16b, v18.16b, v5.16b
        sub             v21.16b, v21.16b, v5.16b

        tbl             v19.16b, {v16.16b, v17.16b}, v6.16b
        tbl             v20.16b, {v17.16b, v18.16b}, v28.16b

        sdot            v0.4s, v16.16b, v7.4b[0]
        sdot            v0.4s, v17.16b, v7.4b[1]

        tbl             v16.16b, {v19.16b, v20.16b}, v6.16b
        tbl             v17.16b, {v20.16b, v21.16b}, v28.16b

        sdot            v1.4s, v19.16b, v7.4b[0]
        sdot            v1.4s, v20.16b, v7.4b[1]

        subs            w4, w4, #2
        shrn            v0.4h, v0.4s, #2
        shrn2           v0.8h, v1.4s, #2

        str             q0, [x0], #16
        b.gt            4b
        ret


        .align JUMP_ALIGN
1:      // prep H or HV cases
        madd            w5, w5, w10, w8
        madd            w7, w6, w10, w9         // for HV
        ldr             q28, L(h_tbl_neon_dotprod)
        mov             w13, 0x2002             // FILTER_WEIGHT * 128 + rounding
        sub             x1, x1, #4              // src - 4
        dup             v27.4s, w13
        ubfx            w8, w5, #7, #7
        and             w5, w5, #0x7F
        ubfx            w10, w7, #7, #7         // for HV
        and             w7, w7, #0x7F           // for HV
        cmp             w3, #4
        csel            w5, w5, w8, le
        add             x5, x11, x5, lsl #3     // subpel H filter address
        movi            v24.16b, #128
        cbz             w6, L(prep_8tap_h_neon_dotprod)

        // prep HV cases
        cmp             w4, #4
        csel            w7, w7, w10, le
        sub             x1, x1, x2, lsl #1      // src - src_stride * 2 - 4
        add             x6, x11, x7, lsl #3     // subpel V filter address
        mov             x15, x30
        ldr             d7, [x6]
        sxtl            v7.8h, v7.8b
        cmp             w9, SHARP1
        b.ne            L(prep_6tap_hv_neon_dotprod)    // if vertical != SHARP

        // prep HV 8-tap cases
        sub             x1, x1, x2              // src - src_stride * 3 - 4
        cmp             w3, #4
        b.eq            40f

        // .align JUMP_ALIGN    // fallthrough
80:     // prep HV8 - 8xN+
        ldr             q29, L(h_tbl_neon_dotprod + 16)
        ldr             q30, L(h_tbl_neon_dotprod + 32)
        ldr             d26, [x5]
        lsl             w8, w3, #1

        .align LOOP_ALIGN
81:
        mov             x6, x1
        mov             x5, x0
        mov             x7, x4

        bl              L(prep_hv_filter8_neon_dotprod)
        mov             v16.16b, v22.16b
        bl              L(prep_hv_filter8_neon_dotprod)
        mov             v17.16b, v22.16b
        bl              L(prep_hv_filter8_neon_dotprod)
        mov             v18.16b, v22.16b
        bl              L(prep_hv_filter8_neon_dotprod)
        mov             v19.16b, v22.16b
        bl              L(prep_hv_filter8_neon_dotprod)
        mov             v20.16b, v22.16b
        bl              L(prep_hv_filter8_neon_dotprod)
        mov             v21.16b, v22.16b
        bl              L(prep_hv_filter8_neon_dotprod)

        .align LOOP_ALIGN
8:
        ldr             q23, [x6]
        add             x6, x6, x2

        smull           v0.4s, v16.4h, v7.h[0]
        smull2          v1.4s, v16.8h, v7.h[0]
        mov             v16.16b, v17.16b

        sub             v23.16b, v23.16b, v24.16b

        mov             v5.16b, v27.16b
        mov             v6.16b, v27.16b

        smlal           v0.4s, v17.4h, v7.h[1]
        smlal2          v1.4s, v17.8h, v7.h[1]
        mov             v17.16b, v18.16b

        tbl             v2.16b, {v23.16b}, v28.16b
        tbl             v3.16b, {v23.16b}, v29.16b
        tbl             v4.16b, {v23.16b}, v30.16b

        smlal           v0.4s, v18.4h, v7.h[2]
        smlal2          v1.4s, v18.8h, v7.h[2]
        mov             v18.16b, v19.16b

        sdot            v5.4s, v2.16b, v26.4b[0]
        sdot            v6.4s, v3.16b, v26.4b[0]

        smlal           v0.4s, v19.4h, v7.h[3]
        smlal2          v1.4s, v19.8h, v7.h[3]
        mov             v19.16b, v20.16b

        sdot            v5.4s, v3.16b, v26.4b[1]
        sdot            v6.4s, v4.16b, v26.4b[1]

        smlal           v0.4s, v20.4h, v7.h[4]
        smlal2          v1.4s, v20.8h, v7.h[4]
        mov             v20.16b, v21.16b

        smlal           v0.4s, v21.4h, v7.h[5]
        smlal2          v1.4s, v21.8h, v7.h[5]
        uzp1            v23.8h, v5.8h, v6.8h
        mov             v21.16b, v22.16b

        smlal           v0.4s, v22.4h, v7.h[6]
        smlal2          v1.4s, v22.8h, v7.h[6]
        sshr            v22.8h, v23.8h, #2

        smlal           v0.4s, v22.4h, v7.h[7]
        smlal2          v1.4s, v22.8h, v7.h[7]

        rshrn           v0.4h, v0.4s, #6
        rshrn2          v0.8h, v1.4s, #6

        subs            w7, w7, #1
        st1             {v0.8h}, [x5], x8
        b.gt            8b

        add             x0, x0, #16
        add             x1, x1, #8
        subs            w3, w3, #8
        b.gt            81b
        ret             x15

        .align JUMP_ALIGN
40:     // prep HV8 - 4xN
        ldr             s25, [x5, #2]
        add             x1, x1, #2

        bl              L(prep_hv_filter4_neon_dotprod)
        mov             v16.16b, v22.16b
        bl              L(prep_hv_filter4_neon_dotprod)
        mov             v17.16b, v22.16b
        bl              L(prep_hv_filter4_neon_dotprod)
        mov             v18.16b, v22.16b
        bl              L(prep_hv_filter4_neon_dotprod)
        mov             v19.16b, v22.16b
        bl              L(prep_hv_filter4_neon_dotprod)
        mov             v20.16b, v22.16b
        bl              L(prep_hv_filter4_neon_dotprod)
        mov             v21.16b, v22.16b
        bl              L(prep_hv_filter4_neon_dotprod)

        .align LOOP_ALIGN
4:
        ld1             {v4.8b}, [x1], x2

        smull           v0.4s, v16.4h, v7.h[0]
        smlal           v0.4s, v17.4h, v7.h[1]
        mov             v16.16b, v17.16b
        mov             v17.16b, v18.16b
        sub             v4.16b, v4.16b, v24.16b

        smlal           v0.4s, v18.4h, v7.h[2]
        smlal           v0.4s, v19.4h, v7.h[3]
        tbl             v2.16b, {v4.16b}, v28.16b
        mov             v5.16b, v27.16b

        mov             v18.16b, v19.16b
        mov             v19.16b, v20.16b

        smlal           v0.4s, v20.4h, v7.h[4]
        smlal           v0.4s, v21.4h, v7.h[5]

        sdot            v5.4s, v2.16b, v25.4b[0]
        mov             v20.16b, v21.16b
        mov             v21.16b, v22.16b

        smlal           v0.4s, v22.4h, v7.h[6]
        shrn            v22.4h, v5.4s, #2

        smlal           v0.4s, v22.4h, v7.h[7]

        rshrn           v0.4h, v0.4s, #6

        str             d0, [x0], #8
        subs            w4, w4, #1
        b.gt            4b
        ret             x15


// tmp(x0) src-4-2*src_stride(x1) src_stride(x2) w(w3) h(w4) fh(x5) fv(x6)
        .align JUMP_ALIGN
L(prep_6tap_hv_neon_dotprod):
        cmp             w3, #4
        b.eq            40f

        // .align JUMP_ALIGN    // fallthrough
80:     // prep HV6 - 8xN+
        ldr             q29, L(h_tbl_neon_dotprod + 16)
        ldr             q30, L(h_tbl_neon_dotprod + 32)
        ldr             d26, [x5]
        lsl             w8, w3, #1

        .align LOOP_ALIGN
81:
        mov             x6, x1
        mov             x5, x0
        mov             x7, x4

        bl              L(prep_hv_filter8_neon_dotprod)
        mov             v16.16b, v22.16b
        bl              L(prep_hv_filter8_neon_dotprod)
        mov             v17.16b, v22.16b
        bl              L(prep_hv_filter8_neon_dotprod)
        mov             v18.16b, v22.16b
        bl              L(prep_hv_filter8_neon_dotprod)
        mov             v19.16b, v22.16b
        bl              L(prep_hv_filter8_neon_dotprod)
        mov             v20.16b, v22.16b

        .align LOOP_ALIGN
8:
        ldr             q23, [x6]
        add             x6, x6, x2

        smull           v0.4s, v16.4h, v7.h[1]
        smull2          v1.4s, v16.8h, v7.h[1]
        sub             v23.16b, v23.16b, v24.16b
        mov             v16.16b, v17.16b

        mov             v5.16b, v27.16b
        mov             v6.16b, v27.16b

        tbl             v2.16b, {v23.16b}, v28.16b
        tbl             v3.16b, {v23.16b}, v29.16b

        smlal           v0.4s, v17.4h, v7.h[2]
        smlal2          v1.4s, v17.8h, v7.h[2]
        tbl             v4.16b, {v23.16b}, v30.16b
        mov             v17.16b, v18.16b

        sdot            v5.4s, v2.16b, v26.4b[0]
        sdot            v6.4s, v3.16b, v26.4b[0]
        smlal           v0.4s, v18.4h, v7.h[3]
        smlal2          v1.4s, v18.8h, v7.h[3]
        mov             v18.16b, v19.16b

        sdot            v5.4s, v3.16b, v26.4b[1]
        sdot            v6.4s, v4.16b, v26.4b[1]
        smlal           v0.4s, v19.4h, v7.h[4]
        smlal2          v1.4s, v19.8h, v7.h[4]
        mov             v19.16b, v20.16b
        uzp1            v23.8h, v5.8h, v6.8h

        smlal           v0.4s, v20.4h, v7.h[5]
        smlal2          v1.4s, v20.8h, v7.h[5]
        sshr            v20.8h, v23.8h, #2

        smlal           v0.4s, v20.4h, v7.h[6]
        smlal2          v1.4s, v20.8h, v7.h[6]

        rshrn           v0.4h, v0.4s, #6
        rshrn2          v0.8h, v1.4s, #6

        st1             {v0.8h}, [x5], x8
        subs            w7, w7, #1
        b.gt            8b

        add             x0, x0, #16
        add             x1, x1, #8
        subs            w3, w3, #8
        b.gt            81b
        ret             x15

        .align FUNC_ALIGN
L(prep_hv_filter8_neon_dotprod):
        ldr             q4, [x6]
        add             x6, x6, x2
        sub             v4.16b, v4.16b, v24.16b
        mov             v22.16b, v27.16b
        mov             v23.16b, v27.16b
        tbl             v2.16b, {v4.16b}, v28.16b
        tbl             v3.16b, {v4.16b}, v29.16b
        tbl             v4.16b, {v4.16b}, v30.16b
        sdot            v22.4s, v2.16b, v26.4b[0]
        sdot            v22.4s, v3.16b, v26.4b[1]
        sdot            v23.4s, v3.16b, v26.4b[0]
        sdot            v23.4s, v4.16b, v26.4b[1]
        shrn            v22.4h, v22.4s, #2
        shrn2           v22.8h, v23.4s, #2
        ret

        .align FUNC_ALIGN
L(prep_hv_filter4_neon_dotprod):
        mov             v22.16b, v27.16b
        ld1             {v4.8b}, [x1], x2
        sub             v4.16b, v4.16b, v24.16b
        tbl             v2.16b, {v4.16b}, v28.16b
        sdot            v22.4s, v2.16b, v25.4b[0]
        shrn            v22.4h, v22.4s, #2
        ret

        .align JUMP_ALIGN
40:     // prep HV6 - 4xN
        ldr             s25, [x5, #2]
        add             x1, x1, #2

        bl              L(prep_hv_filter4_neon_dotprod)
        mov             v16.16b, v22.16b
        bl              L(prep_hv_filter4_neon_dotprod)
        mov             v17.16b, v22.16b
        bl              L(prep_hv_filter4_neon_dotprod)
        mov             v18.16b, v22.16b
        bl              L(prep_hv_filter4_neon_dotprod)
        mov             v19.16b, v22.16b
        bl              L(prep_hv_filter4_neon_dotprod)
        mov             v20.16b, v22.16b

        .align LOOP_ALIGN
4:
        ld1             {v4.8b}, [x1], x2

        smull           v0.4s, v16.4h, v7.h[1]
        smlal           v0.4s, v17.4h, v7.h[2]
        sub             v4.16b, v4.16b, v24.16b
        mov             v16.16b, v17.16b
        mov             v17.16b, v18.16b

        smlal           v0.4s, v18.4h, v7.h[3]
        smlal           v0.4s, v19.4h, v7.h[4]
        tbl             v2.16b, {v4.16b}, v28.16b
        mov             v5.16b, v27.16b

        mov             v18.16b, v19.16b
        mov             v19.16b, v20.16b
        sdot            v5.4s, v2.16b, v25.4b[0]

        smlal           v0.4s, v20.4h, v7.h[5]
        shrn            v20.4h, v5.4s, #2

        smlal           v0.4s, v20.4h, v7.h[6]

        rshrn           v0.4h, v0.4s, #6

        str             d0, [x0], #8
        subs            w4, w4, #1
        b.gt            4b
        ret             x15


// tmp(x0) src-4(x1) src_stride(x2) w(w3) h(w4) fh(x5)
        .align JUMP_ALIGN
L(prep_8tap_h_neon_dotprod):
        adr             x9, L(prep_8tap_h_neon_dotprod_tbl)
        ldrh            w8, [x9, x12, lsl #1]
        sub             x9, x9, x8
        br              x9

        .align JUMP_ALIGN
40:     // prep H - 4xN
        AARCH64_VALID_JUMP_TARGET
        add             x1, x1, #2
        ldr             s26, [x5, #2]

        .align LOOP_ALIGN
4:
        ldr             d0, [x1]
        ldr             d1, [x1, x2]
        add             x1, x1, x2, lsl #1

        sub             v0.8b, v0.8b, v24.8b
        sub             v1.8b, v1.8b, v24.8b

        mov             v4.16b, v27.16b
        mov             v5.16b, v27.16b

        tbl             v2.16b, {v0.16b}, v28.16b
        tbl             v3.16b, {v1.16b}, v28.16b

        sdot            v4.4s, v2.16b, v26.4b[0]
        sdot            v5.4s, v3.16b, v26.4b[0]

        subs            w4, w4, #2
        shrn            v4.4h, v4.4s, #2
        shrn2           v4.8h, v5.4s, #2

        str             q4, [x0], #16
        b.gt            4b
        ret

        .align JUMP_ALIGN
80:     // prep H - 8xN
        AARCH64_VALID_JUMP_TARGET
        ldr             q29, L(h_tbl_neon_dotprod + 16)
        ldr             q30, L(h_tbl_neon_dotprod + 32)
        ldr             d26, [x5]

        .align LOOP_ALIGN
8:
        ldr             q0, [x1]
        ldr             q16, [x1, x2]
        add             x1, x1, x2, lsl #1

        sub             v0.16b, v0.16b, v24.16b
        sub             v16.16b, v16.16b, v24.16b

        mov             v4.16b, v27.16b
        mov             v5.16b, v27.16b
        mov             v20.16b, v27.16b
        mov             v21.16b, v27.16b

        tbl             v1.16b, {v0.16b}, v28.16b
        tbl             v2.16b, {v0.16b}, v29.16b
        tbl             v3.16b, {v0.16b}, v30.16b
        tbl             v17.16b, {v16.16b}, v28.16b
        tbl             v18.16b, {v16.16b}, v29.16b
        tbl             v19.16b, {v16.16b}, v30.16b

        sdot            v4.4s, v1.16b, v26.4b[0]
        sdot            v5.4s, v2.16b, v26.4b[0]
        sdot            v20.4s, v17.16b, v26.4b[0]
        sdot            v21.4s, v18.16b, v26.4b[0]
        sdot            v4.4s, v2.16b, v26.4b[1]
        sdot            v5.4s, v3.16b, v26.4b[1]
        sdot            v20.4s, v18.16b, v26.4b[1]
        sdot            v21.4s, v19.16b, v26.4b[1]

        uzp1            v4.8h, v4.8h, v5.8h
        uzp1            v20.8h, v20.8h, v21.8h
        sshr            v4.8h, v4.8h, #2
        sshr            v20.8h, v20.8h, #2

        subs            w4, w4, #2
        stp             q4, q20, [x0], #32
        b.gt            8b
        ret

        .align JUMP_ALIGN
160:    // prep H - 16xN
        AARCH64_VALID_JUMP_TARGET
        ldr             q29, L(h_tbl_neon_dotprod + 16)
        ldr             q30, L(h_tbl_neon_dotprod + 32)
        ldr             q31, L(h_tbl_neon_dotprod + 48)
        ldr             d26, [x5]

        .align LOOP_ALIGN
16:
        ldp             q16, q17, [x1]
        add             x1, x1, x2

        sub             v16.16b, v16.16b, v24.16b
        sub             v17.16b, v17.16b, v24.16b

        mov             v6.16b, v27.16b
        mov             v7.16b, v27.16b
        mov             v22.16b, v27.16b
        mov             v23.16b, v27.16b

        tbl             v0.16b, {v16.16b}, v28.16b
        tbl             v1.16b, {v16.16b}, v29.16b
        tbl             v2.16b, {v16.16b}, v30.16b
        tbl             v3.16b, {v16.16b, v17.16b}, v31.16b
        tbl             v4.16b, {v17.16b}, v28.16b

        sdot            v6.4s, v0.16b, v26.4b[0]
        sdot            v7.4s, v1.16b, v26.4b[0]
        sdot            v22.4s, v2.16b, v26.4b[0]
        sdot            v23.4s, v3.16b, v26.4b[0]
        sdot            v6.4s, v1.16b, v26.4b[1]
        sdot            v7.4s, v2.16b, v26.4b[1]
        sdot            v22.4s, v3.16b, v26.4b[1]
        sdot            v23.4s, v4.16b, v26.4b[1]

        uzp1            v6.8h, v6.8h, v7.8h
        uzp1            v22.8h, v22.8h, v23.8h
        sshr            v6.8h, v6.8h, #2
        sshr            v22.8h, v22.8h, #2

        subs            w4, w4, #1
        stp             q6, q22, [x0], #32
        b.gt            16b
        ret

        .align JUMP_ALIGN
320:    // prep H - 32xN+
640:
1280:
        AARCH64_VALID_JUMP_TARGET
        ldr             q29, L(h_tbl_neon_dotprod + 16)
        ldr             q30, L(h_tbl_neon_dotprod + 32)
        ldr             q31, L(h_tbl_neon_dotprod + 48)
        ldr             d26, [x5]
        sub             x2, x2, w3, uxtw
        mov             x8, x3

        .align LOOP_ALIGN
32:
        ldp             q16, q17, [x1], #16

        sub             v16.16b, v16.16b, v24.16b
        sub             v17.16b, v17.16b, v24.16b

        mov             v6.16b, v27.16b
        mov             v7.16b, v27.16b
        mov             v22.16b, v27.16b
        mov             v23.16b, v27.16b

        tbl             v0.16b, {v16.16b}, v28.16b
        tbl             v1.16b, {v16.16b}, v29.16b
        tbl             v2.16b, {v16.16b}, v30.16b
        tbl             v3.16b, {v16.16b, v17.16b}, v31.16b
        tbl             v4.16b, {v17.16b}, v28.16b

        sdot            v6.4s, v0.16b, v26.4b[0]
        sdot            v7.4s, v1.16b, v26.4b[0]
        sdot            v22.4s, v2.16b, v26.4b[0]
        sdot            v23.4s, v3.16b, v26.4b[0]
        sdot            v6.4s, v1.16b, v26.4b[1]
        sdot            v7.4s, v2.16b, v26.4b[1]
        sdot            v22.4s, v3.16b, v26.4b[1]
        sdot            v23.4s, v4.16b, v26.4b[1]

        uzp1            v6.8h, v6.8h, v7.8h
        uzp1            v22.8h, v22.8h, v23.8h
        sshr            v6.8h, v6.8h, #2
        sshr            v22.8h, v22.8h, #2

        subs            w8, w8, #16
        stp             q6, q22, [x0], #32
        b.gt            32b

        add             x1, x1, x2
        mov             x8, x3
        subs            w4, w4, #1
        b.gt            32b
        ret

L(prep_8tap_h_neon_dotprod_tbl):
        .hword (L(prep_8tap_h_neon_dotprod_tbl) - 1280b)
        .hword (L(prep_8tap_h_neon_dotprod_tbl) - 640b)
        .hword (L(prep_8tap_h_neon_dotprod_tbl) - 320b)
        .hword (L(prep_8tap_h_neon_dotprod_tbl) - 160b)
        .hword (L(prep_8tap_h_neon_dotprod_tbl) - 80b)
        .hword (L(prep_8tap_h_neon_dotprod_tbl) - 40b)


// tmp(x0) src(x1) src_stride(x2) w(w3) h(w4) mx(w5) my(w6) filter_type(w7)
        .align JUMP_ALIGN
L(prep_neon_dotprod):
        adr             x9, L(prep_neon_dotprod_tbl)
        ldrh            w8, [x9, x12, lsl #1]
        sub             x9, x9, x8
        br              x9

40:
        AARCH64_VALID_JUMP_TARGET
        .align LOOP_ALIGN
4:
        ld1             {v0.s}[0], [x1], x2
        ld1             {v0.s}[1], [x1], x2
        ld1             {v1.s}[0], [x1], x2
        ld1             {v1.s}[1], [x1], x2
        subs            w4, w4, #4
        ushll           v0.8h, v0.8b, #4
        ushll           v1.8h, v1.8b, #4
        stp             q0, q1, [x0], #32
        b.gt            4b
        ret

80:
        AARCH64_VALID_JUMP_TARGET
        .align LOOP_ALIGN
8:
        ld1             {v0.8b}, [x1], x2
        ld1             {v1.8b}, [x1], x2
        ld1             {v2.8b}, [x1], x2
        ld1             {v3.8b}, [x1], x2
        ushll           v0.8h, v0.8b, #4
        ushll           v1.8h, v1.8b, #4
        ushll           v2.8h, v2.8b, #4
        ushll           v3.8h, v3.8b, #4
        subs            w4, w4, #4
        stp             q0, q1, [x0]
        stp             q2, q3, [x0, #32]
        add             x0, x0, #64
        b.gt            8b
        ret

160:
        AARCH64_VALID_JUMP_TARGET
        .align LOOP_ALIGN
16:
        ldr             q1, [x1]
        ldr             q3, [x1, x2]
        add             x1, x1, x2, lsl #1
        ushll           v0.8h, v1.8b, #4
        ushll2          v1.8h, v1.16b, #4
        ushll           v2.8h, v3.8b, #4
        ushll2          v3.8h, v3.16b, #4
        subs            w4, w4, #2
        stp             q0, q1, [x0]
        stp             q2, q3, [x0, #32]
        add             x0, x0, #64
        b.gt            16b
        ret

320:
        AARCH64_VALID_JUMP_TARGET
        .align LOOP_ALIGN
32:
        ldp             q4, q5, [x1]
        add             x1, x1, x2
        ushll           v0.8h, v4.8b, #4
        ushll2          v1.8h, v4.16b, #4
        ldp             q6, q7, [x1]
        add             x1, x1, x2
        ushll           v2.8h, v5.8b, #4
        ushll2          v3.8h, v5.16b, #4
        ushll           v4.8h, v6.8b, #4
        ushll2          v5.8h, v6.16b, #4
        stp             q0, q1, [x0]
        stp             q2, q3, [x0, #32]
        subs            w4, w4, #2
        ushll           v6.8h, v7.8b, #4
        ushll2          v7.8h, v7.16b, #4
        stp             q4, q5, [x0, #64]
        stp             q6, q7, [x0, #96]
        add             x0, x0, #128
        b.gt            32b
        ret

640:
        AARCH64_VALID_JUMP_TARGET
        .align LOOP_ALIGN
64:
        ldp             q4, q5, [x1]
        ldp             q6, q7, [x1, #32]
        add             x1, x1, x2
        ushll           v0.8h, v4.8b, #4
        ushll2          v1.8h, v4.16b, #4
        ushll           v2.8h, v5.8b, #4
        ushll2          v3.8h, v5.16b, #4
        ushll           v4.8h, v6.8b, #4
        ushll2          v5.8h, v6.16b, #4
        ushll           v6.8h, v7.8b, #4
        ushll2          v7.8h, v7.16b, #4
        subs            w4, w4, #1
        stp             q0, q1, [x0]
        stp             q2, q3, [x0, #32]
        stp             q4, q5, [x0, #64]
        stp             q6, q7, [x0, #96]
        add             x0, x0, #128
        b.gt            64b
        ret

1280:
        AARCH64_VALID_JUMP_TARGET
        .align LOOP_ALIGN
128:
        ldp             q24, q25, [x1]
        ldp             q26, q27, [x1, #32]
        ushll           v0.8h, v24.8b, #4
        ushll2          v1.8h, v24.16b, #4
        ushll           v2.8h, v25.8b, #4
        ushll2          v3.8h, v25.16b, #4
        ushll           v4.8h, v26.8b, #4
        ushll2          v5.8h, v26.16b, #4
        ushll           v6.8h, v27.8b, #4
        ushll2          v7.8h, v27.16b, #4
        stp             q0, q1, [x0]
        stp             q2, q3, [x0, #32]
        stp             q4, q5, [x0, #64]
        stp             q6, q7, [x0, #96]
        ldp             q28, q29, [x1, #64]
        ldp             q30, q31, [x1, #96]
        add             x1, x1, x2
        ushll           v16.8h, v28.8b, #4
        ushll2          v17.8h, v28.16b, #4
        ushll           v18.8h, v29.8b, #4
        ushll2          v19.8h, v29.16b, #4
        ushll           v20.8h, v30.8b, #4
        ushll2          v21.8h, v30.16b, #4
        ushll           v22.8h, v31.8b, #4
        ushll2          v23.8h, v31.16b, #4
        subs            w4, w4, #1
        stp             q16, q17, [x0, #128]
        stp             q18, q19, [x0, #160]
        stp             q20, q21, [x0, #192]
        stp             q22, q23, [x0, #224]
        add             x0, x0, #256
        b.gt            128b
        ret

L(prep_neon_dotprod_tbl):
        .hword (L(prep_neon_dotprod_tbl) - 1280b)
        .hword (L(prep_neon_dotprod_tbl) - 640b)
        .hword (L(prep_neon_dotprod_tbl) - 320b)
        .hword (L(prep_neon_dotprod_tbl) - 160b)
        .hword (L(prep_neon_dotprod_tbl) - 80b)
        .hword (L(prep_neon_dotprod_tbl) - 40b)
endfunc


// dst(x0) dst_stride(x1) src(x2) src_stride(x3) w(w4) h(w5) mx(w6) my(w7)
function put_8tap_sharp_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, SHARP1
        mov             x9, SHARP1
        b               put_8tap_neon_dotprod
endfunc

function put_8tap_sharp_smooth_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, SHARP1
        mov             x9, SMOOTH1
        b               put_8tap_neon_dotprod
endfunc

function put_8tap_sharp_regular_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, SHARP1
        mov             x9, REGULAR1
        b               put_8tap_neon_dotprod
endfunc

function put_8tap_smooth_sharp_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, SMOOTH1
        mov             x9, SHARP1
        b               put_8tap_neon_dotprod
endfunc

function put_8tap_smooth_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, SMOOTH1
        mov             x9, SMOOTH1
        b               put_8tap_neon_dotprod
endfunc

function put_8tap_smooth_regular_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, SMOOTH1
        mov             x9, REGULAR1
        b               put_8tap_neon_dotprod
endfunc

function put_8tap_regular_sharp_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, REGULAR1
        mov             x9, SHARP1
        b               put_8tap_neon_dotprod
endfunc

function put_8tap_regular_smooth_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, REGULAR1
        mov             x9, SMOOTH1
        b               put_8tap_neon_dotprod
endfunc

function put_8tap_regular_8bpc_neon_dotprod, export=1, align=FUNC_ALIGN
        mov             x8, REGULAR1
        mov             x9, REGULAR1
        // b            put_8tap_neon_dotprod   // fallthrough
endfunc


// dst(x0) dst_stride(x1) src(x2) src_stride(x3) w(w4) h(w5) mx(w6) my(w7)
function put_8tap_neon_dotprod, align=FUNC_ALIGN
        clz             w12, w4
        mov             w10,  #0x4081   // (1 << 14) | (1 << 7) | (1 << 0)
        sub             w12, w12, #24   // for jump tables
        movrel          x11, X(mc_subpel_filters)
        cbnz            w6, 1f          // prep HV or H
        cbz             w7, L(put_neon_dotprod)

        // put V cases
        madd            w7, w7, w10, w9
        adr             x9, L(put_8tap_v_neon_dotprod_tbl)
        sub             x2, x2, x3
        ldrh            w8, [x9, x12, lsl #1]
        ldr             q6, L(v_tbl_neon_dotprod)
        ubfx            w10, w7, #7, #7
        and             w7, w7, #0x7F
        ldr             q28, L(v_tbl_neon_dotprod + 16)
        cmp             w5, #4
        csel            w7, w7, w10, le
        ldr             q29, L(v_tbl_neon_dotprod + 32)
        sub             x2, x2, x3, lsl #1      // src - src_stride * 3
        add             x7, x11, x7, lsl #3     // subpel V filter address
        sub             x9, x9, x8
        ldr             d7, [x7]
        movi            v5.16b, #128
        br              x9

        .align JUMP_ALIGN
20:     // put V - 2xN
        AARCH64_VALID_JUMP_TARGET
        ldr             h16, [x2]
        ldr             h17, [x2, x3]
        add             x2, x2, x3, lsl #1
        ldr             h18, [x2]
        ldr             h19, [x2, x3]
        add             x2, x2, x3, lsl #1

        ldr             h20, [x2]
        ldr             h21, [x2, x3]
        add             x2, x2, x3, lsl #1
        ldr             h22, [x2]
        ldr             h23, [x2, x3]
        add             x2, x2, x3, lsl #1

        zip1            v0.8b, v16.8b, v17.8b
        zip1            v2.8b, v18.8b, v19.8b
        zip1            v18.8b, v20.8b, v21.8b
        zip1            v24.8b, v22.8b, v23.8b

        zip1            v16.4h, v0.4h, v2.4h
        zip1            v17.4h, v18.4h, v24.4h

        sub             v16.8b, v16.8b, v5.8b
        sub             v17.8b, v17.8b, v5.8b

        .align LOOP_ALIGN
2:
        ldr             h18, [x2]
        ldr             h21, [x2, x3]
        add             x2, x2, x3, lsl #1

        movi            v0.4s, #32, lsl #8      // 64 * 128, bias for SDOT
        movi            v1.4s, #32, lsl #8

        sub             v18.8b, v18.8b, v5.8b
        sub             v21.8b, v21.8b, v5.8b

        tbl             v19.16b, {v16.16b, v17.16b}, v6.16b
        tbl             v20.16b, {v17.16b, v18.16b}, v28.16b

        sdot            v0.4s, v16.16b, v7.4b[0]
        sdot            v0.4s, v17.16b, v7.4b[1]

        tbl             v16.16b, {v19.16b, v20.16b}, v6.16b
        tbl             v17.16b, {v20.16b, v21.16b}, v28.16b

        sdot            v1.4s, v19.16b, v7.4b[0]
        sdot            v1.4s, v20.16b, v7.4b[1]

        uzp1            v0.8h, v0.8h, v1.8h
        subs            w5, w5, #2
        sqrshrun        v0.8b, v0.8h, #6

        mov             v1.h[0], v0.h[2]
        str             h0, [x0]
        str             h1, [x0, x1]
        add             x0, x0, x1, lsl #1
        b.gt            2b
        ret

        .align JUMP_ALIGN
40:     // put V - 4xN
        AARCH64_VALID_JUMP_TARGET
        ldr             s16, [x2]
        ldr             s17, [x2, x3]
        add             x2, x2, x3, lsl #1
        ldr             s18, [x2]
        ldr             s19, [x2, x3]
        add             x2, x2, x3, lsl #1

        ldr             s20, [x2]
        ldr             s21, [x2, x3]
        add             x2, x2, x3, lsl #1
        ldr             s22, [x2]
        ldr             s23, [x2, x3]
        add             x2, x2, x3, lsl #1

        zip1            v0.8b, v16.8b, v17.8b
        zip1            v2.8b, v18.8b, v19.8b
        zip1            v18.8b, v20.8b, v21.8b
        zip1            v24.8b, v22.8b, v23.8b

        zip1            v16.8h, v0.8h, v2.8h
        zip1            v17.8h, v18.8h, v24.8h

        sub             v16.16b, v16.16b, v5.16b
        sub             v17.16b, v17.16b, v5.16b

        .align LOOP_ALIGN
4:
        ldr             s18, [x2]
        ldr             s21, [x2, x3]
        add             x2, x2, x3, lsl #1

        movi            v0.4s, #32, lsl #8      // 64 * 128, bias for SDOT
        movi            v1.4s, #32, lsl #8

        sub             v18.16b, v18.16b, v5.16b
        sub             v21.16b, v21.16b, v5.16b

        tbl             v19.16b, {v16.16b, v17.16b}, v6.16b
        tbl             v20.16b, {v17.16b, v18.16b}, v28.16b

        sdot            v0.4s, v16.16b, v7.4b[0]
        sdot            v0.4s, v17.16b, v7.4b[1]

        tbl             v16.16b, {v19.16b, v20.16b}, v6.16b
        tbl             v17.16b, {v20.16b, v21.16b}, v28.16b

        sdot            v1.4s, v19.16b, v7.4b[0]
        sdot            v1.4s, v20.16b, v7.4b[1]

        uzp1            v0.8h, v0.8h, v1.8h
        subs            w5, w5, #2
        sqrshrun        v0.8b, v0.8h, #6

        mov             v1.s[0], v0.s[1]
        str             s0, [x0]
        str             s1, [x0, x1]
        add             x0, x0, x1, lsl #1
        b.gt            4b
        ret

        .align JUMP_ALIGN
80:     // put V - 8xN
        AARCH64_VALID_JUMP_TARGET
        ldr             d16, [x2]
        ldr             d17, [x2, x3]
        add             x2, x2, x3, lsl #1
        ldr             d18, [x2]
        ldr             d19, [x2, x3]
        add             x2, x2, x3, lsl #1

        ldr             d20, [x2]
        ldr             d21, [x2, x3]
        add             x2, x2, x3, lsl #1
        ldr             d22, [x2]
        ldr             d23, [x2, x3]
        add             x2, x2, x3, lsl #1

        zip1            v0.16b, v16.16b, v17.16b
        zip1            v2.16b, v18.16b, v19.16b
        zip1            v18.16b, v20.16b, v21.16b
        zip1            v24.16b, v22.16b, v23.16b

        zip1            v16.8h, v0.8h, v2.8h
        zip2            v19.8h, v0.8h, v2.8h
        zip1            v17.8h, v18.8h, v24.8h
        zip2            v20.8h, v18.8h, v24.8h

        sub             v16.16b, v16.16b, v5.16b
        sub             v19.16b, v19.16b, v5.16b
        sub             v17.16b, v17.16b, v5.16b
        sub             v20.16b, v20.16b, v5.16b

        .align LOOP_ALIGN
8:
        ldr             d21, [x2]
        ldr             d27, [x2, x3]
        add             x2, x2, x3, lsl #1

        movi            v0.4s, #32, lsl #8      // 64 * 128, bias for SDOT
        movi            v1.4s, #32, lsl #8
        movi            v2.4s, #32, lsl #8
        movi            v3.4s, #32, lsl #8

        sub             v18.16b, v21.16b, v5.16b
        sub             v21.16b, v21.16b, v5.16b
        sub             v24.16b, v27.16b, v5.16b
        sub             v27.16b, v27.16b, v5.16b

        tbl             v22.16b, {v16.16b, v17.16b}, v6.16b
        tbl             v25.16b, {v19.16b, v20.16b}, v6.16b
        tbl             v23.16b, {v17.16b, v18.16b}, v28.16b
        tbl             v26.16b, {v20.16b, v21.16b}, v29.16b

        sdot            v0.4s, v16.16b, v7.4b[0]
        sdot            v0.4s, v17.16b, v7.4b[1]
        sdot            v1.4s, v19.16b, v7.4b[0]
        sdot            v1.4s, v20.16b, v7.4b[1]

        tbl             v16.16b, {v22.16b, v23.16b}, v6.16b
        tbl             v19.16b, {v25.16b, v26.16b}, v6.16b
        tbl             v17.16b, {v23.16b, v24.16b}, v28.16b
        tbl             v20.16b, {v26.16b, v27.16b}, v29.16b

        sdot            v2.4s, v22.16b, v7.4b[0]
        sdot            v2.4s, v23.16b, v7.4b[1]
        sdot            v3.4s, v25.16b, v7.4b[0]
        sdot            v3.4s, v26.16b, v7.4b[1]

        subs            w5, w5, #2
        uzp1            v0.8h, v0.8h, v1.8h
        uzp1            v2.8h, v2.8h, v3.8h
        sqrshrun        v0.8b, v0.8h, #6
        sqrshrun        v1.8b, v2.8h, #6

        str             d0, [x0]
        str             d1, [x0, x1]
        add             x0, x0, x1, lsl #1
        b.gt            8b
        ret

        .align JUMP_ALIGN
160:    // put V - 16xN+
320:
640:
1280:
        AARCH64_VALID_JUMP_TARGET
        ldr             q30, L(v_tbl_neon_dotprod + 48)
        ldr             q31, L(v_tbl_neon_dotprod + 64)

        .align LOOP_ALIGN
161:
        mov             x7, x2
        mov             x6, x0
        mov             x8, x5

        ldr             q16, [x7]
        ldr             q17, [x7, x3]
        add             x7, x7, x3, lsl #1
        ldr             q18, [x7]
        ldr             q19, [x7, x3]
        add             x7, x7, x3, lsl #1

        zip1            v0.16b, v16.16b, v17.16b
        zip2            v1.16b, v16.16b, v17.16b
        zip1            v2.16b, v18.16b, v19.16b
        zip2            v3.16b, v18.16b, v19.16b

        ldr             q20, [x7]
        ldr             q21, [x7, x3]
        add             x7, x7, x3, lsl #1
        ldr             q22, [x7]
        ldr             q23, [x7, x3]
        add             x7, x7, x3, lsl #1

        zip1            v18.16b, v20.16b, v21.16b
        zip2            v21.16b, v20.16b, v21.16b
        zip1            v24.16b, v22.16b, v23.16b
        zip2            v27.16b, v22.16b, v23.16b

        zip1            v16.8h, v0.8h, v2.8h
        zip2            v19.8h, v0.8h, v2.8h
        zip1            v22.8h, v1.8h, v3.8h
        zip2            v25.8h, v1.8h, v3.8h

        zip1            v17.8h, v18.8h, v24.8h
        zip2            v20.8h, v18.8h, v24.8h
        zip1            v23.8h, v21.8h, v27.8h
        zip2            v26.8h, v21.8h, v27.8h

        sub             v16.16b, v16.16b, v5.16b
        sub             v19.16b, v19.16b, v5.16b
        sub             v22.16b, v22.16b, v5.16b
        sub             v25.16b, v25.16b, v5.16b

        sub             v17.16b, v17.16b, v5.16b
        sub             v20.16b, v20.16b, v5.16b
        sub             v23.16b, v23.16b, v5.16b
        sub             v26.16b, v26.16b, v5.16b

        .align LOOP_ALIGN
16:
        ldr             q27, [x7]
        add             x7, x7, x3

        movi            v0.4s, #32, lsl #8      // 64 * 128, bias for SDOT
        movi            v1.4s, #32, lsl #8
        movi            v2.4s, #32, lsl #8
        movi            v3.4s, #32, lsl #8

        sub             v18.16b, v27.16b, v5.16b
        sub             v21.16b, v27.16b, v5.16b
        sub             v24.16b, v27.16b, v5.16b
        sub             v27.16b, v27.16b, v5.16b

        sdot            v0.4s, v16.16b, v7.4b[0]
        sdot            v1.4s, v19.16b, v7.4b[0]
        sdot            v2.4s, v22.16b, v7.4b[0]
        sdot            v3.4s, v25.16b, v7.4b[0]

        tbl             v16.16b, {v16.16b, v17.16b}, v6.16b
        tbl             v19.16b, {v19.16b, v20.16b}, v6.16b
        tbl             v22.16b, {v22.16b, v23.16b}, v6.16b
        tbl             v25.16b, {v25.16b, v26.16b}, v6.16b

        sdot            v0.4s, v17.16b, v7.4b[1]
        sdot            v1.4s, v20.16b, v7.4b[1]
        sdot            v2.4s, v23.16b, v7.4b[1]
        sdot            v3.4s, v26.16b, v7.4b[1]

        tbl             v17.16b, {v17.16b, v18.16b}, v28.16b
        tbl             v20.16b, {v20.16b, v21.16b}, v29.16b
        tbl             v23.16b, {v23.16b, v24.16b}, v30.16b
        tbl             v26.16b, {v26.16b, v27.16b}, v31.16b

        subs            w8, w8, #1
        uzp1            v0.8h, v0.8h, v1.8h
        uzp1            v2.8h, v2.8h, v3.8h
        sqrshrun        v0.8b, v0.8h, #6
        sqrshrun2       v0.16b, v2.8h, #6

        st1             {v0.16b}, [x6], x1
        b.gt            16b

        add             x0, x0, #16
        add             x2, x2, #16
        subs            w4, w4, #16
        b.gt            161b
        ret

L(put_8tap_v_neon_dotprod_tbl):
        .hword (L(put_8tap_v_neon_dotprod_tbl) - 1280b)
        .hword (L(put_8tap_v_neon_dotprod_tbl) - 640b)
        .hword (L(put_8tap_v_neon_dotprod_tbl) - 320b)
        .hword (L(put_8tap_v_neon_dotprod_tbl) - 160b)
        .hword (L(put_8tap_v_neon_dotprod_tbl) - 80b)
        .hword (L(put_8tap_v_neon_dotprod_tbl) - 40b)
        .hword (L(put_8tap_v_neon_dotprod_tbl) - 20b)


        .align JUMP_ALIGN
1:      // put H or HV cases
        madd            w6, w6, w10, w8
        madd            w13, w7, w10, w9        // for HV
        sub             x2, x2, #4              // src - 4
        ubfx            w8, w6, #7, #7
        and             w6, w6, #0x7F
        ubfx            w10, w13, #7, #7        // for HV
        and             w13, w13, #0x7F         // for HV
        ldr             q28, L(h_tbl_neon_dotprod)
        cmp             w4, #4
        csel            w6, w6, w8, le
        add             x6, x11, x6, lsl #3     // subpel H filter address
        movi            v24.16b, #128
        cbz             w7, L(put_8tap_h_neon_dotprod)

        // put HV cases
        cmp             w5, #4
        csel            w13, w13, w10, le
        sub             x2, x2, x3, lsl #1      // src - src_stride * 2 - 4
        ldr             q27, L(hv_tbl_neon_dotprod)
        add             x7, x11, x13, lsl #3    // subpel V filter address
        mov             x15, x30
        ldr             d7, [x7]
        mov             w14, 0x2002             // FILTER_WEIGHT * 128 + rounding
        sxtl            v7.8h, v7.8b
        dup             v31.4s, w14
        cmp             w9, SHARP1
        b.ne            L(put_6tap_hv_neon_dotprod) // if vertical != SHARP

        // put HV 8-tap cases
        sub             x2, x2, x3              // src - src_stride * 3 - 4
        cmp             w4, #4
        b.eq            40f
        b.lt            20f

        // .align JUMP_ALIGN    // fallthrough
80:     // put HV8 - 8xN+
        ldr             q29, L(h_tbl_neon_dotprod + 16)
        ldr             q30, L(h_tbl_neon_dotprod + 32)
        ldr             d26, [x6]

        .align LOOP_ALIGN
81:
        mov             x7, x2
        mov             x6, x0
        mov             x8, x5

        bl              L(put_hv_filter8_neon_dotprod)
        mov             v16.16b, v22.16b
        bl              L(put_hv_filter8_neon_dotprod)
        mov             v17.16b, v22.16b
        bl              L(put_hv_filter8_neon_dotprod)
        mov             v18.16b, v22.16b
        bl              L(put_hv_filter8_neon_dotprod)
        mov             v19.16b, v22.16b
        bl              L(put_hv_filter8_neon_dotprod)
        mov             v20.16b, v22.16b
        bl              L(put_hv_filter8_neon_dotprod)
        mov             v21.16b, v22.16b
        bl              L(put_hv_filter8_neon_dotprod)

        .align LOOP_ALIGN
8:
        ldr             q23, [x7]
        add             x7, x7, x3

        smull           v0.4s, v16.4h, v7.h[0]
        smull2          v1.4s, v16.8h, v7.h[0]
        mov             v16.16b, v17.16b

        sub             v23.16b, v23.16b, v24.16b

        mov             v5.16b, v31.16b
        mov             v6.16b, v31.16b

        smlal           v0.4s, v17.4h, v7.h[1]
        smlal2          v1.4s, v17.8h, v7.h[1]
        mov             v17.16b, v18.16b

        tbl             v2.16b, {v23.16b}, v28.16b
        tbl             v3.16b, {v23.16b}, v29.16b
        tbl             v4.16b, {v23.16b}, v30.16b

        smlal           v0.4s, v18.4h, v7.h[2]
        smlal2          v1.4s, v18.8h, v7.h[2]
        mov             v18.16b, v19.16b

        sdot            v5.4s, v2.16b, v26.4b[0]
        sdot            v6.4s, v3.16b, v26.4b[0]

        smlal           v0.4s, v19.4h, v7.h[3]
        smlal2          v1.4s, v19.8h, v7.h[3]
        mov             v19.16b, v20.16b

        sdot            v5.4s, v3.16b, v26.4b[1]
        sdot            v6.4s, v4.16b, v26.4b[1]

        smlal           v0.4s, v20.4h, v7.h[4]
        smlal2          v1.4s, v20.8h, v7.h[4]
        mov             v20.16b, v21.16b

        smlal           v0.4s, v21.4h, v7.h[5]
        smlal2          v1.4s, v21.8h, v7.h[5]
        mov             v21.16b, v22.16b

        smlal           v0.4s, v22.4h, v7.h[6]
        smlal2          v1.4s, v22.8h, v7.h[6]
        shrn            v22.4h, v5.4s, #2
        shrn2           v22.8h, v6.4s, #2

        smlal           v0.4s, v22.4h, v7.h[7]
        smlal2          v1.4s, v22.8h, v7.h[7]

        tbl             v0.16b, {v0.16b, v1.16b}, v27.16b
        subs            w8, w8, #1
        sqrshrun        v0.8b, v0.8h, #2

        st1             {v0.8b}, [x6], x1
        b.gt            8b

        add             x0, x0, #8
        add             x2, x2, #8
        subs            w4, w4, #8
        b.gt            81b
        ret             x15

        .align JUMP_ALIGN
40:     // put HV8 - 4xN
        ldr             s26, [x6, #2]
        add             x2, x2, #2

        bl              L(put_hv_filter4_neon_dotprod)
        mov             v16.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v17.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v18.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v19.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v20.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v21.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)

        .align LOOP_ALIGN
4:
        ld1             {v4.8b}, [x2], x3

        smull           v0.4s, v16.4h, v7.h[0]
        smlal           v0.4s, v17.4h, v7.h[1]
        mov             v16.16b, v17.16b
        mov             v17.16b, v18.16b
        sub             v4.16b, v4.16b, v24.16b

        smlal           v0.4s, v18.4h, v7.h[2]
        smlal           v0.4s, v19.4h, v7.h[3]
        tbl             v2.16b, {v4.16b}, v28.16b
        mov             v5.16b, v31.16b

        mov             v18.16b, v19.16b
        mov             v19.16b, v20.16b

        smlal           v0.4s, v20.4h, v7.h[4]
        smlal           v0.4s, v21.4h, v7.h[5]

        sdot            v5.4s, v2.16b, v26.4b[0]
        mov             v20.16b, v21.16b
        mov             v21.16b, v22.16b

        subs            w5, w5, #1
        smlal           v0.4s, v22.4h, v7.h[6]
        shrn            v22.4h, v5.4s, #2

        smlal           v0.4s, v22.4h, v7.h[7]
        tbl             v0.16b, {v0.16b, v1.16b}, v27.16b
        sqrshrun        v0.8b, v0.8h, #2

        str             s0, [x0]
        add             x0, x0, x1
        b.gt            4b
        ret             x15

        .align JUMP_ALIGN
20:     // put HV8 - 2xN
        ldr             s26, [x6, #2]
        add             x2, x2, #2

        bl              L(put_hv_filter4_neon_dotprod)
        mov             v16.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v17.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v18.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v19.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v20.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v21.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)

        .align LOOP_ALIGN
2:
        ld1             {v4.8b}, [x2], x3

        smull           v0.4s, v16.4h, v7.h[0]
        smlal           v0.4s, v17.4h, v7.h[1]
        mov             v16.16b, v17.16b
        mov             v17.16b, v18.16b
        sub             v4.16b, v4.16b, v24.16b

        smlal           v0.4s, v18.4h, v7.h[2]
        smlal           v0.4s, v19.4h, v7.h[3]
        tbl             v2.16b, {v4.16b}, v28.16b
        mov             v5.16b, v31.16b

        mov             v18.16b, v19.16b
        mov             v19.16b, v20.16b

        smlal           v0.4s, v20.4h, v7.h[4]
        smlal           v0.4s, v21.4h, v7.h[5]

        sdot            v5.4s, v2.16b, v26.4b[0]
        mov             v20.16b, v21.16b
        mov             v21.16b, v22.16b

        subs            w5, w5, #1
        smlal           v0.4s, v22.4h, v7.h[6]
        shrn            v22.4h, v5.4s, #2

        smlal           v0.4s, v22.4h, v7.h[7]
        tbl             v0.16b, {v0.16b, v1.16b}, v27.16b
        sqrshrun        v0.8b, v0.8h, #2

        str             h0, [x0]
        add             x0, x0, x1
        b.gt            2b
        ret             x15


// dst(x0) dst_stride(x1) src-4-2*src_stride(x2) src_stride(x3) w(w4) h(w5) fh(x6) fv(x7)
        .align JUMP_ALIGN
L(put_6tap_hv_neon_dotprod):
        cmp             w4, #4
        b.eq            40f
        b.lt            20f

        // .align JUMP_ALIGN    // fallthrough
80:     // put HV6 - 8xN+
        ldr             q29, L(h_tbl_neon_dotprod + 16)
        ldr             q30, L(h_tbl_neon_dotprod + 32)
        ldr             d26, [x6]

        .align LOOP_ALIGN
81:
        mov             x7, x2
        mov             x6, x0
        mov             x8, x5

        bl              L(put_hv_filter8_neon_dotprod)
        mov             v16.16b, v22.16b
        bl              L(put_hv_filter8_neon_dotprod)
        mov             v17.16b, v22.16b
        bl              L(put_hv_filter8_neon_dotprod)
        mov             v18.16b, v22.16b
        bl              L(put_hv_filter8_neon_dotprod)
        mov             v19.16b, v22.16b
        bl              L(put_hv_filter8_neon_dotprod)
        mov             v20.16b, v22.16b

        .align LOOP_ALIGN
8:
        ldr             q23, [x7]
        add             x7, x7, x3

        smull           v0.4s, v16.4h, v7.h[1]
        smull2          v1.4s, v16.8h, v7.h[1]
        sub             v23.16b, v23.16b, v24.16b
        mov             v16.16b, v17.16b

        mov             v5.16b, v31.16b
        mov             v6.16b, v31.16b

        tbl             v2.16b, {v23.16b}, v28.16b
        tbl             v3.16b, {v23.16b}, v29.16b

        smlal           v0.4s, v17.4h, v7.h[2]
        smlal2          v1.4s, v17.8h, v7.h[2]
        tbl             v4.16b, {v23.16b}, v30.16b
        mov             v17.16b, v18.16b

        sdot            v5.4s, v2.16b, v26.4b[0]
        sdot            v6.4s, v3.16b, v26.4b[0]
        smlal           v0.4s, v18.4h, v7.h[3]
        smlal2          v1.4s, v18.8h, v7.h[3]
        mov             v18.16b, v19.16b

        sdot            v5.4s, v3.16b, v26.4b[1]
        sdot            v6.4s, v4.16b, v26.4b[1]
        smlal           v0.4s, v19.4h, v7.h[4]
        smlal2          v1.4s, v19.8h, v7.h[4]
        mov             v19.16b, v20.16b
        uzp1            v23.8h, v5.8h, v6.8h

        smlal           v0.4s, v20.4h, v7.h[5]
        smlal2          v1.4s, v20.8h, v7.h[5]
        sshr            v20.8h, v23.8h, #2

        subs            w8, w8, #1
        smlal           v0.4s, v20.4h, v7.h[6]
        smlal2          v1.4s, v20.8h, v7.h[6]

        tbl             v0.16b, {v0.16b, v1.16b}, v27.16b
        sqrshrun        v0.8b, v0.8h, #2

        st1             {v0.8b}, [x6], x1
        b.gt            8b

        add             x0, x0, #8
        add             x2, x2, #8
        subs            w4, w4, #8
        b.gt            81b
        ret             x15

        .align FUNC_ALIGN
L(put_hv_filter8_neon_dotprod):
        ldr             q4, [x7]
        add             x7, x7, x3
        sub             v4.16b, v4.16b, v24.16b
        mov             v22.16b, v31.16b
        mov             v23.16b, v31.16b
        tbl             v2.16b, {v4.16b}, v28.16b
        tbl             v3.16b, {v4.16b}, v29.16b
        tbl             v4.16b, {v4.16b}, v30.16b
        sdot            v22.4s, v2.16b, v26.4b[0]
        sdot            v22.4s, v3.16b, v26.4b[1]
        sdot            v23.4s, v3.16b, v26.4b[0]
        sdot            v23.4s, v4.16b, v26.4b[1]
        shrn            v22.4h, v22.4s, #2
        shrn2           v22.8h, v23.4s, #2
        ret

        .align FUNC_ALIGN
L(put_hv_filter4_neon_dotprod):
        mov             v22.16b, v31.16b
        ld1             {v4.8b}, [x2], x3
        sub             v4.16b, v4.16b, v24.16b
        tbl             v2.16b, {v4.16b}, v28.16b
        sdot            v22.4s, v2.16b, v26.4b[0]
        shrn            v22.4h, v22.4s, #2
        ret

        .align JUMP_ALIGN
40:     // put HV6 - 4xN
        ldr             s26, [x6, #2]
        add             x2, x2, #2

        bl              L(put_hv_filter4_neon_dotprod)
        mov             v16.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v17.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v18.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v19.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v20.16b, v22.16b

        .align LOOP_ALIGN
4:
        ld1             {v4.8b}, [x2], x3

        smull           v0.4s, v16.4h, v7.h[1]
        smlal           v0.4s, v17.4h, v7.h[2]
        sub             v4.16b, v4.16b, v24.16b
        mov             v16.16b, v17.16b
        mov             v17.16b, v18.16b

        smlal           v0.4s, v18.4h, v7.h[3]
        smlal           v0.4s, v19.4h, v7.h[4]
        tbl             v2.16b, {v4.16b}, v28.16b
        mov             v5.16b, v31.16b

        mov             v18.16b, v19.16b
        mov             v19.16b, v20.16b
        sdot            v5.4s, v2.16b, v26.4b[0]

        smlal           v0.4s, v20.4h, v7.h[5]
        shrn            v20.4h, v5.4s, #2

        subs            w5, w5, #1
        smlal           v0.4s, v20.4h, v7.h[6]

        tbl             v0.16b, {v0.16b}, v27.16b
        sqrshrun        v0.8b, v0.8h, #2

        str             s0, [x0]
        add             x0, x0, x1
        b.gt            4b
        ret             x15

        .align JUMP_ALIGN
20:     // put HV6 - 2xN
        ldr             s26, [x6, #2]
        add             x2, x2, #2

        bl              L(put_hv_filter4_neon_dotprod)
        mov             v16.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v17.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v18.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v19.16b, v22.16b
        bl              L(put_hv_filter4_neon_dotprod)
        mov             v20.16b, v22.16b

        .align LOOP_ALIGN
2:
        ld1             {v4.8b}, [x2], x3

        smull           v0.4s, v16.4h, v7.h[1]
        smlal           v0.4s, v17.4h, v7.h[2]
        sub             v4.16b, v4.16b, v24.16b
        mov             v16.16b, v17.16b
        mov             v17.16b, v18.16b

        smlal           v0.4s, v18.4h, v7.h[3]
        smlal           v0.4s, v19.4h, v7.h[4]
        tbl             v2.16b, {v4.16b}, v28.16b
        mov             v5.16b, v31.16b

        mov             v18.16b, v19.16b
        mov             v19.16b, v20.16b
        sdot            v5.4s, v2.16b, v26.4b[0]

        smlal           v0.4s, v20.4h, v7.h[5]
        shrn            v20.4h, v5.4s, #2

        subs            w5, w5, #1
        smlal           v0.4s, v20.4h, v7.h[6]

        tbl             v0.16b, {v0.16b}, v27.16b
        sqrshrun        v0.8b, v0.8h, #2

        str             h0, [x0]
        add             x0, x0, x1
        b.gt            2b
        ret             x15


// dst(x0) dst_stride(x1) src-4(x2) src_stride(x3) w(w4) h(w5) fh(x6)
        .align JUMP_ALIGN
L(put_8tap_h_neon_dotprod):
        adr             x9, L(put_8tap_h_neon_dotprod_tbl)
        ldrh            w8, [x9, x12, lsl #1]
        mov             w10, #0x2022    // 64 * 128 + 34, bias and rounding for SDOT
        sub             x9, x9, x8
        dup             v27.4s, w10
        br              x9

        .align JUMP_ALIGN
20:     // put H - 2xN
        AARCH64_VALID_JUMP_TARGET
        add             x2, x2, #2
        ldr             s6, [x6, #2]

        .align LOOP_ALIGN
2:
        ldr             d0, [x2]
        ldr             d1, [x2, x3]
        add             x2, x2, x3, lsl #1

        sub             v0.8b, v0.8b, v24.8b
        sub             v1.8b, v1.8b, v24.8b

        mov             v4.16b, v27.16b
        mov             v5.16b, v27.16b

        tbl             v2.16b, {v0.16b}, v28.16b
        tbl             v3.16b, {v1.16b}, v28.16b

        sdot            v4.4s, v2.16b, v6.4b[0]
        sdot            v5.4s, v3.16b, v6.4b[0]

        uzp1            v4.8h, v4.8h, v5.8h
        sqshrun         v4.8b, v4.8h, #6

        subs            w5, w5, #2
        fmov            x7, d4
        lsr             x8, x7, #32
        strh            w7, [x0]
        strh            w8, [x0, x1]
        add             x0, x0, x1, lsl #1
        b.gt            2b
        ret

        .align JUMP_ALIGN
40:     // put H - 4xN
        AARCH64_VALID_JUMP_TARGET
        add             x2, x2, #2
        ldr             s6, [x6, #2]

        .align LOOP_ALIGN
4:
        ldr             d0, [x2]
        ldr             d1, [x2, x3]
        add             x2, x2, x3, lsl #1

        sub             v0.8b, v0.8b, v24.8b
        sub             v1.8b, v1.8b, v24.8b

        mov             v4.16b, v27.16b
        mov             v5.16b, v27.16b

        tbl             v2.16b, {v0.16b}, v28.16b
        tbl             v3.16b, {v1.16b}, v28.16b

        sdot            v4.4s, v2.16b, v6.4b[0]
        sdot            v5.4s, v3.16b, v6.4b[0]

        uzp1            v4.8h, v4.8h, v5.8h
        sqshrun         v4.8b, v4.8h, #6

        subs            w5, w5, #2
        fmov            x7, d4
        lsr             x8, x7, #32
        str             w7, [x0]
        str             w8, [x0, x1]
        add             x0, x0, x1, lsl #1
        b.gt            4b
        ret

        .align JUMP_ALIGN
80:     // put H - 8xN
        AARCH64_VALID_JUMP_TARGET
        ldr             q29, L(h_tbl_neon_dotprod + 16)
        ldr             q30, L(h_tbl_neon_dotprod + 32)
        ldr             d26, [x6]

        .align LOOP_ALIGN
8:
        ldr             q0, [x2]
        ldr             q16, [x2, x3]
        add             x2, x2, x3, lsl #1

        sub             v0.16b, v0.16b, v24.16b
        sub             v16.16b, v16.16b, v24.16b

        mov             v4.16b, v27.16b
        mov             v5.16b, v27.16b
        mov             v20.16b, v27.16b
        mov             v21.16b, v27.16b

        tbl             v1.16b, {v0.16b}, v28.16b
        tbl             v2.16b, {v0.16b}, v29.16b
        tbl             v3.16b, {v0.16b}, v30.16b
        tbl             v17.16b, {v16.16b}, v28.16b
        tbl             v18.16b, {v16.16b}, v29.16b
        tbl             v19.16b, {v16.16b}, v30.16b

        sdot            v4.4s, v1.16b, v26.4b[0]
        sdot            v5.4s, v2.16b, v26.4b[0]
        sdot            v20.4s, v17.16b, v26.4b[0]
        sdot            v21.4s, v18.16b, v26.4b[0]
        sdot            v4.4s, v2.16b, v26.4b[1]
        sdot            v5.4s, v3.16b, v26.4b[1]
        sdot            v20.4s, v18.16b, v26.4b[1]
        sdot            v21.4s, v19.16b, v26.4b[1]

        uzp1            v4.8h, v4.8h, v5.8h
        uzp1            v20.8h, v20.8h, v21.8h
        sqshrun         v4.8b, v4.8h, #6
        sqshrun         v20.8b, v20.8h, #6

        subs            w5, w5, #2
        str             d4, [x0]
        str             d20, [x0, x1]
        add             x0, x0, x1, lsl #1
        b.gt            8b
        ret

        .align JUMP_ALIGN
160:    // put H - 16xN
        AARCH64_VALID_JUMP_TARGET
        ldr             q29, L(h_tbl_neon_dotprod + 16)
        ldr             q30, L(h_tbl_neon_dotprod + 32)
        ldr             q31, L(h_tbl_neon_dotprod + 48)
        ldr             d26, [x6]

        .align LOOP_ALIGN
16:
        ldp             q16, q17, [x2]
        add             x2, x2, x3

        sub             v16.16b, v16.16b, v24.16b
        sub             v17.16b, v17.16b, v24.16b

        mov             v6.16b, v27.16b
        mov             v7.16b, v27.16b
        mov             v22.16b, v27.16b
        mov             v23.16b, v27.16b

        tbl             v0.16b, {v16.16b}, v28.16b
        tbl             v1.16b, {v16.16b}, v29.16b
        tbl             v2.16b, {v16.16b}, v30.16b
        tbl             v3.16b, {v16.16b, v17.16b}, v31.16b
        tbl             v4.16b, {v17.16b}, v28.16b

        sdot            v6.4s, v0.16b, v26.4b[0]
        sdot            v7.4s, v1.16b, v26.4b[0]
        sdot            v22.4s, v2.16b, v26.4b[0]
        sdot            v23.4s, v3.16b, v26.4b[0]
        sdot            v6.4s, v1.16b, v26.4b[1]
        sdot            v7.4s, v2.16b, v26.4b[1]
        sdot            v22.4s, v3.16b, v26.4b[1]
        sdot            v23.4s, v4.16b, v26.4b[1]

        uzp1            v6.8h, v6.8h, v7.8h
        uzp1            v22.8h, v22.8h, v23.8h
        sqshrun         v6.8b, v6.8h, #6
        sqshrun2        v6.16b, v22.8h, #6

        subs            w5, w5, #1
        str             q6, [x0]
        add             x0, x0, x1
        b.gt            16b
        ret

        .align JUMP_ALIGN
320:    // put H - 32xN+
640:
1280:
        AARCH64_VALID_JUMP_TARGET
        ldr             q29, L(h_tbl_neon_dotprod + 16)
        ldr             q30, L(h_tbl_neon_dotprod + 32)
        ldr             q31, L(h_tbl_neon_dotprod + 48)
        ldr             d26, [x6]
        sub             x1, x1, w4, uxtw
        sub             x3, x3, w4, uxtw
        mov             x8, x4

        .align LOOP_ALIGN
32:
        ldp             q16, q17, [x2], #16

        sub             v16.16b, v16.16b, v24.16b
        sub             v17.16b, v17.16b, v24.16b

        mov             v6.16b, v27.16b
        mov             v7.16b, v27.16b
        mov             v22.16b, v27.16b
        mov             v23.16b, v27.16b

        tbl             v0.16b, {v16.16b}, v28.16b
        tbl             v1.16b, {v16.16b}, v29.16b
        tbl             v2.16b, {v16.16b}, v30.16b
        tbl             v3.16b, {v16.16b, v17.16b}, v31.16b
        tbl             v4.16b, {v17.16b}, v28.16b

        sdot            v6.4s, v0.16b, v26.4b[0]
        sdot            v7.4s, v1.16b, v26.4b[0]
        sdot            v22.4s, v2.16b, v26.4b[0]
        sdot            v23.4s, v3.16b, v26.4b[0]
        sdot            v6.4s, v1.16b, v26.4b[1]
        sdot            v7.4s, v2.16b, v26.4b[1]
        sdot            v22.4s, v3.16b, v26.4b[1]
        sdot            v23.4s, v4.16b, v26.4b[1]

        uzp1            v6.8h, v6.8h, v7.8h
        uzp1            v22.8h, v22.8h, v23.8h
        sqshrun         v6.8b, v6.8h, #6
        sqshrun2        v6.16b, v22.8h, #6

        subs            w8, w8, #16
        str             q6, [x0], #16
        b.gt            32b

        add             x2, x2, x3
        add             x0, x0, x1
        mov             x8, x4
        subs            w5, w5, #1
        b.gt            32b
        ret

L(put_8tap_h_neon_dotprod_tbl):
        .hword (L(put_8tap_h_neon_dotprod_tbl) - 1280b)
        .hword (L(put_8tap_h_neon_dotprod_tbl) - 640b)
        .hword (L(put_8tap_h_neon_dotprod_tbl) - 320b)
        .hword (L(put_8tap_h_neon_dotprod_tbl) - 160b)
        .hword (L(put_8tap_h_neon_dotprod_tbl) - 80b)
        .hword (L(put_8tap_h_neon_dotprod_tbl) - 40b)
        .hword (L(put_8tap_h_neon_dotprod_tbl) - 20b)


// dst(x0) dst_stride(x1) src(x2) src_stride(x3) w(w4) h(w5) mx(w6) my(w7) filter_type(w8)
        .align JUMP_ALIGN
L(put_neon_dotprod):
        adr             x9, L(put_neon_dotprod_tbl)
        ldrh            w8, [x9, x12, lsl #1]
        sub             x9, x9, x8
        br              x9

20:
        AARCH64_VALID_JUMP_TARGET
        .align LOOP_ALIGN
2:
        ldrh            w9, [x2]
        ldrh            w10, [x2, x3]
        add             x2, x2, x3, lsl #1
        subs            w5, w5, #2
        strh            w9, [x0]
        strh            w10, [x0, x1]
        add             x0, x0, x1, lsl #1
        b.gt            2b
        ret

40:
        AARCH64_VALID_JUMP_TARGET
        .align LOOP_ALIGN
4:
        ldr             w9, [x2]
        ldr             w10, [x2, x3]
        add             x2, x2, x3, lsl #1
        subs            w5, w5, #2
        str             w9, [x0]
        str             w10, [x0, x1]
        add             x0, x0, x1, lsl #1
        b.gt            4b
        ret

80:
        AARCH64_VALID_JUMP_TARGET
        .align LOOP_ALIGN
8:
        ldr             x9, [x2]
        ldr             x10, [x2, x3]
        add             x2, x2, x3, lsl #1
        subs            w5, w5, #2
        str             x9, [x0]
        str             x10, [x0, x1]
        add             x0, x0, x1, lsl #1
        b.gt            8b
        ret

160:
        AARCH64_VALID_JUMP_TARGET
        .align LOOP_ALIGN
16:
        ldr             q0, [x2]
        ldr             q1, [x2, x3]
        add             x2, x2, x3, lsl #1
        subs            w5, w5, #2
        str             q0, [x0]
        str             q1, [x0, x1]
        add             x0, x0, x1, lsl #1
        b.gt            16b
        ret

320:
        AARCH64_VALID_JUMP_TARGET
        .align LOOP_ALIGN
32:
        ldp             q0, q1, [x2]
        add             x2, x2, x3
        stp             q0, q1, [x0]
        add             x0, x0, x1
        ldp             q2, q3, [x2]
        add             x2, x2, x3
        stp             q2, q3, [x0]
        add             x0, x0, x1
        subs            w5, w5, #2
        b.gt            32b
        ret

640:
        AARCH64_VALID_JUMP_TARGET
        .align LOOP_ALIGN
64:
        ldp             q0, q1, [x2]
        stp             q0, q1, [x0]
        ldp             q2, q3, [x2, #32]
        add             x2, x2, x3
        stp             q2, q3, [x0, #32]
        add             x0, x0, x1
        subs            w5, w5, #1
        b.gt            64b
        ret

1280:
        AARCH64_VALID_JUMP_TARGET
        .align LOOP_ALIGN
128:
        ldp             q0, q1, [x2]
        stp             q0, q1, [x0]
        ldp             q2, q3, [x2, #32]
        stp             q2, q3, [x0, #32]
        ldp             q4, q5, [x2, #64]
        stp             q4, q5, [x0, #64]
        ldp             q6, q7, [x2, #96]
        add             x2, x2, x3
        stp             q6, q7, [x0, #96]
        add             x0, x0, x1
        subs            w5, w5, #1
        b.gt            128b
        ret

L(put_neon_dotprod_tbl):
        .hword (L(put_neon_dotprod_tbl) - 1280b)
        .hword (L(put_neon_dotprod_tbl) - 640b)
        .hword (L(put_neon_dotprod_tbl) - 320b)
        .hword (L(put_neon_dotprod_tbl) - 160b)
        .hword (L(put_neon_dotprod_tbl) - 80b)
        .hword (L(put_neon_dotprod_tbl) - 40b)
        .hword (L(put_neon_dotprod_tbl) - 20b)
endfunc

DISABLE_DOTPROD
#endif  // HAVE_DOTPROD
