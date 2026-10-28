/*
 * compat.h - lets the same sources build on modern Linux (gcc 13, glibc
 * or musl) and on late-1990s Linux (gcc 2.7.2.3, glibc 2.0 or libc5,
 * kernel 2.0/2.2).
 *
 * Rules the code follows so it builds with both:
 *   - C89: declarations at the start of a block, no // comments,
 *     no designated initializers, no stdint.h (own u8..u64 types);
 *   - select() for waiting (no poll.h / ppoll);
 *   - gettimeofday() when there is no CLOCK_MONOTONIC;
 *   - lseek()+read()/write() instead of pread()/pwrite();
 *   - gethostbyname() instead of getaddrinfo();
 *   - no prctl() unless the headers exist;
 *   - 64-bit values printed via (double) and %.0f, not %llu.
 */
#ifndef COMPAT_H
#define COMPAT_H

#include <sys/types.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

#if defined(__has_include)
# if __has_include(<sys/prctl.h>) && __has_include(<linux/prctl.h>)
#  include <sys/prctl.h>
# endif
#endif

typedef unsigned char      u8;
typedef unsigned short     u16;
typedef unsigned int       u32;   /* int is 32 bits on every Linux target */
typedef unsigned long long u64;
typedef long long          s64;

#define S64_MAX 0x7fffffffffffffffLL

#ifdef __GNUC__
#define INLINE __inline__
#else
#define INLINE
#endif

/* socklen_t first appeared in glibc 2.1.
 * glibc 2.0 used size_t; Linux libc 4/5 used int. If detection gets it
 * wrong, build with -DSOCKLEN_T=int (or size_t, or socklen_t). */
#if defined(SOCKLEN_T)
typedef SOCKLEN_T sock_len;
#elif defined(__GLIBC__) && (__GLIBC__ > 2 || (__GLIBC__ == 2 && __GLIBC_MINOR__ >= 1))
typedef socklen_t sock_len;             /* glibc 2.1 and later            */
#elif defined(__GLIBC__) && __GLIBC__ == 2
typedef size_t sock_len;                /* glibc 2.0 declared size_t *    */
#elif defined(__GLIBC__) || defined(__GNU_LIBRARY__) || defined(_LINUX_C_LIB_VERSION_MAJOR)
typedef int sock_len;                   /* Linux libc 4/5                 */
#else
typedef socklen_t sock_len;             /* musl and other modern libcs    */
#endif

/* Microseconds from a monotonic clock where available. Old kernels and
 * libraries only have gettimeofday(), which is fine unless someone sets
 * the clock in the middle of a transfer. */
static INLINE s64 now_us(void)
{
#if defined(CLOCK_MONOTONIC) && !defined(NO_CLOCK_GETTIME)
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (s64)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
#else
    struct timeval tv;
    gettimeofday(&tv, 0);
    return (s64)tv.tv_sec * 1000000 + tv.tv_usec;
#endif
}

/* Sleep until fd1 or fd2 (-1 = unused) is readable, or timeout_us passes. */
static INLINE void wait_readable(int fd1, int fd2, s64 timeout_us)
{
    fd_set rs;
    struct timeval tv;
    int maxfd = fd1 > fd2 ? fd1 : fd2;

    if (timeout_us < 0)
        timeout_us = 0;
    FD_ZERO(&rs);
    if (fd1 >= 0) FD_SET(fd1, &rs);
    if (fd2 >= 0) FD_SET(fd2, &rs);
    tv.tv_sec = (long)(timeout_us / 1000000);
    tv.tv_usec = (long)(timeout_us % 1000000);
    select(maxfd + 1, &rs, 0, 0, &tv);
}

/* Ask for precise sleeps (default timer slack is 50 us) where supported. */
static INLINE void precise_timers(void)
{
#ifdef PR_SET_TIMERSLACK
    prctl(PR_SET_TIMERSLACK, 1UL);
#endif
}

static INLINE void set_nonblock(int fd)
{
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
}

/* Positioned read/write without pread()/pwrite() (glibc 2.0 lacks them).
 * Return 0 on success, -1 on error. */
static INLINE int seek_to(int fd, u64 off)
{
    if (sizeof(off_t) < 8 && off > 0x7fffffffULL) {
        errno = EFBIG;                    /* 32-bit off_t: 2 GB limit */
        return -1;
    }
    return lseek(fd, (off_t)off, SEEK_SET) == (off_t)-1 ? -1 : 0;
}

static INLINE int read_at(int fd, void *buf, u32 len, u64 off)
{
    char *p = (char *)buf;
    if (seek_to(fd, off) < 0)
        return -1;
    while (len > 0) {
        ssize_t r = read(fd, p, len);
        if (r < 0 && errno == EINTR) continue;
        if (r <= 0) return -1;
        p += r;
        len -= (u32)r;
    }
    return 0;
}

static INLINE int write_at(int fd, const void *buf, u32 len, u64 off)
{
    const char *p = (const char *)buf;
    if (seek_to(fd, off) < 0)
        return -1;
    while (len > 0) {
        ssize_t r = write(fd, p, len);
        if (r < 0 && errno == EINTR) continue;
        if (r <= 0) return -1;
        p += r;
        len -= (u32)r;
    }
    return 0;
}

/* Resolve a host name or dotted address to an IPv4 address. */
static INLINE int resolve_ipv4(const char *host, struct in_addr *out)
{
    struct hostent *h;
    if (inet_aton(host, out))
        return 0;
    h = gethostbyname(host);
    if (!h || h->h_addrtype != AF_INET || h->h_length != 4)
        return -1;
    memcpy(out, h->h_addr_list[0], 4);
    return 0;
}

#endif
