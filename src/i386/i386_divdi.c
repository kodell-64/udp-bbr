/*
 * i386_divdi.c - 64-bit integer division helpers normally supplied by
 * libgcc. On 32-bit x86, gcc turns uint64_t division into calls to these
 * functions. They are only needed when linking without the 32-bit libgcc
 * (e.g. the static musl build in the Makefile); with gcc-multilib
 * installed, libgcc provides them and this file compiles to nothing.
 *
 * Written with shifts, compares and subtraction only, which gcc emits
 * inline on i386, so the helpers never call themselves.
 */
#if defined(__i386__) && defined(PROVIDE_DIVDI)

typedef unsigned long long u64;
typedef long long          s64;

u64 __udivmoddi4(u64 n, u64 d, u64 *rp);
u64 __udivdi3(u64 n, u64 d);
u64 __umoddi3(u64 n, u64 d);
s64 __divdi3(s64 a, s64 b);
s64 __moddi3(s64 a, s64 b);
s64 __divmoddi4(s64 a, s64 b, s64 *rp);

u64 __udivmoddi4(u64 n, u64 d, u64 *rp)
{
    if (d == 0)
        __builtin_trap();                       /* division by zero */
    if ((n >> 32) == 0 && (d >> 32) == 0) {     /* fits the CPU's divide */
        unsigned a = (unsigned)n, b = (unsigned)d;
        if (rp) *rp = a % b;
        return a / b;
    }
    if (d > n) {
        if (rp) *rp = n;
        return 0;
    }
    int s = 0;                                  /* align d under n */
    while (!(d >> 63) && (d << 1) <= n) {
        d <<= 1;
        s++;
    }
    u64 q = 0;
    for (; s >= 0; s--) {                       /* restoring division */
        q <<= 1;
        if (n >= d) {
            n -= d;
            q |= 1;
        }
        d >>= 1;
    }
    if (rp) *rp = n;
    return q;
}

u64 __udivdi3(u64 n, u64 d) { return __udivmoddi4(n, d, 0); }
u64 __umoddi3(u64 n, u64 d) { u64 r; __udivmoddi4(n, d, &r); return r; }

/* Signed: quotient truncates toward zero, remainder takes the dividend's sign. */
s64 __divmoddi4(s64 a, s64 b, s64 *rp)
{
    u64 ua = a < 0 ? 0 - (u64)a : (u64)a;
    u64 ub = b < 0 ? 0 - (u64)b : (u64)b;
    u64 r, q = __udivmoddi4(ua, ub, &r);
    if ((a < 0) != (b < 0)) q = 0 - q;
    if (a < 0) r = 0 - r;
    if (rp) *rp = (s64)r;
    return (s64)q;
}

s64 __divdi3(s64 a, s64 b) { return __divmoddi4(a, b, 0); }
s64 __moddi3(s64 a, s64 b) { s64 r; __divmoddi4(a, b, &r); return r; }

#else
typedef int i386_divdi_unused;   /* ISO C forbids an empty translation unit */
#endif
