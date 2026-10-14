/* Host oracle for EasyFloat's RNE, DAZ and FTZ arithmetic. Not RTL. */
#include <math.h>
#include <float.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
static float clean(float x) {
    return fpclassify(x) == FP_SUBNORMAL ? copysignf(0.0f, x) : x;
}
static float easy_fma(float a, float b, float c) {
    a = clean(a);
    b = clean(b);
    c = clean(c);
    float z = fmaf(a, b, c);
    if (isnan(z)) {
        // EasyFloat emits a positive canonical quiet NaN, independent of the
        // input payload or the host libm's invalid-operation NaN sign.
        const uint32_t bits = UINT32_C(0x7fc00000);
        memcpy(&z, &bits, sizeof(z));
        return z;
    }
    /* EasyFloat rounds the normalized mantissa before checking underflow.
       Close to FLT_MIN this differs from IEEE gradual-underflow rounding. */
    if (fabsf(z) == FLT_MIN) {
        long double exact = (long double)a * (long double)b + (long double)c;
        long double threshold = (long double)FLT_MIN - ldexpl(1.0L, -151);
        if (fabsl(exact) < threshold)
            return copysignf(0.0f, z);
    }
    return clean(z);
}
void panel_fma(const float *a, const float *b, const float *c, float *o, size_t n) {
    for (size_t i = 0; i < n; ++i)
        o[i] = easy_fma(a[i], b[i], c[i]);
}
void panel_dot(const float *a, const float *b, float *c, int m, int n, int k) {
    for (int i = 0; i < m; ++i)
        for (int j = 0; j < n; ++j) {
            float s = c[i * n + j];
            for (int t = 0; t < k; ++t)
                s = easy_fma(a[i * k + t], b[t * n + j], s);
            c[i * n + j] = s;
        }
}
