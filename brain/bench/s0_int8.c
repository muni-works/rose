/* S0 actor, int8 on a Cortex-M7 (STM32H7): cycles per forward.
 *
 *   python3 gen_weights.py > s0_weights.h     (518 KB of const int8: lands in flash)
 *   add s0_int8.c + s0_weights.h to any STM32Cube project for the board,
 *   call s0_bench() after clock setup, read the printout (or s0_cycles in the debugger).
 *
 * Shape 710 -> 512 -> 256 -> 128 -> 20, ELU, the same as ipse's brain_bench `s0`
 * (which checks this kernel against f32 on the host). Weights int8 per-layer
 * scale, activations requantized to int8 per layer, i32 accumulate.
 * With __ARM_FEATURE_DSP the inner loop is 2 SMLADs per 4 weights.
 */
#include <stdint.h>
#include <math.h>
#include "s0_weights.h"   /* S0_N[5], s0_w0..s0_w3, s0_b0..s0_b3, s0_scale[4] */

#if defined(__ARM_FEATURE_DSP)
#include "cmsis_compiler.h"
#endif

#define DWT_CTRL   (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)
#define DEMCR      (*(volatile uint32_t *)0xE000EDFC)

static int8_t bufq[712];   /* input padded to a multiple of 4 */
static float  bufa[512], bufb[512];
volatile uint32_t s0_cycles, s0_cycles_min = 0xffffffff;

static int32_t dot_i8(const int8_t *w, const int8_t *x, int n) {
#if defined(__ARM_FEATURE_DSP)
    int32_t acc = 0;
    const uint32_t *w4 = (const uint32_t *)w, *x4 = (const uint32_t *)x;
    for (int i = 0; i < n / 4; i++) {
        uint32_t a = w4[i], b = x4[i];
        acc = __SMLAD(__SXTB16(a), __SXTB16(b), acc);                    /* bytes 0, 2 */
        acc = __SMLAD(__SXTB16(__ROR(a, 8)), __SXTB16(__ROR(b, 8)), acc); /* bytes 1, 3 */
    }
    return acc;
#else
    int32_t acc = 0;
    for (int i = 0; i < n; i++) acc += (int32_t)w[i] * x[i];
    return acc;
#endif
}

static void layer(const int8_t *w, const float *b, float scale, int n_in, int n_out,
                  const float *in, float *out, int act) {
    float m = 1e-8f;
    for (int i = 0; i < n_in; i++) m = fmaxf(m, fabsf(in[i]));
    float xs = m / 127.0f, inv = 127.0f / m;
    for (int i = 0; i < n_in; i++) bufq[i] = (int8_t)lrintf(in[i] * inv);
    for (int i = n_in; i < ((n_in + 3) & ~3); i++) bufq[i] = 0;
    int stride = (n_in + 3) & ~3;   /* rows padded by gen_weights.py */
    for (int o = 0; o < n_out; o++) {
        float y = dot_i8(w + o * stride, bufq, stride) * scale * xs + b[o];
        out[o] = (act && y < 0.0f) ? expm1f(y) : y;
    }
}

const float *s0_forward(const float *x) {
    layer(s0_w0, s0_b0, s0_scale[0], S0_N0, S0_N1, x, bufa, 1);
    layer(s0_w1, s0_b1, s0_scale[1], S0_N1, S0_N2, bufa, bufb, 1);
    layer(s0_w2, s0_b2, s0_scale[2], S0_N2, S0_N3, bufb, bufa, 1);
    layer(s0_w3, s0_b3, s0_scale[3], S0_N3, S0_N4, bufa, bufb, 0);
    return bufb;
}

void s0_bench(void) {
    static float x[S0_N0];
    for (int i = 0; i < S0_N0; i++) x[i] = (float)((i * 37) % 101) / 50.0f - 1.0f;
    DEMCR |= 1u << 24;          /* TRCENA */
    DWT_CYCCNT = 0;
    DWT_CTRL |= 1u;             /* CYCCNTENA */
    for (int k = 0; k < 100; k++) {
        uint32_t t0 = DWT_CYCCNT;
        (void)s0_forward(x);
        uint32_t c = DWT_CYCCNT - t0;
        s0_cycles = c;
        if (c < s0_cycles_min) s0_cycles_min = c;
    }
    /* µs = s0_cycles_min / (SYSCLK MHz). 1 kHz S0 needs < ~500 µs, leaving room for the bus. */
}
