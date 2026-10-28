/*
 * server.c - UDP file server that sends with BBRv3 congestion control.
 *
 *   ./udpbbr_server [-p port] [-d directory] [-l log.csv] [-q]
 *
 * A client sends REQ <filename>; the server answers META and streams the
 * file in CHUNK_SIZE pieces. Reliability is QUIC-like: monotonically
 * increasing packet numbers, SACK-style ACK ranges, packet- and
 * time-threshold loss detection (RFC 9002), and a probe timeout (PTO) for
 * tail losses. BBRv3 decides how much may be in flight (cwnd) and how fast
 * packets leave (pacing rate).
 *
 * One transfer at a time, to keep the example readable.
 * Plain C89 + long long (see compat.h): builds with gcc 2.7.2.3 too.
 */
#define _GNU_SOURCE
#define _FILE_OFFSET_BITS 64
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#include "compat.h"
#include "bbr3.h"
#include "proto.h"

#define ST_INFLIGHT 0
#define ST_ACKED    1
#define ST_LOST     2

#define MAX_ACK_DELAY_US   2000        /* receiver acks within ~1 ms       */
#define IDLE_ABORT_US      20000000    /* give up after 20 s without ACKs  */
#define PKT_SIZE           (DATA_HDR + CHUNK_SIZE)
#define SEND_BATCH         64          /* packets per loop iteration, max  */

typedef struct {
    bbr_pkt_t info;
    u64       chunk;
    u8        state;
} sent_t;

typedef struct {
    int fd;
    struct sockaddr_in peer;
    u32 session;

    int  file_fd;
    u64  size, n_chunks;
    u8  *chunk_acked, *chunk_queued;
    u64  acked_chunks, next_new_chunk;
    u64 *retx;                          /* ring buffer of chunks to resend */
    u64  retx_head, retx_tail, retx_cap;

    sent_t *sent;                       /* indexed by packet number        */
    u64  sent_cap, pn_next, first_unacked;
    s64  largest_acked;

    s64  srtt, rttvar, min_rtt, latest_rtt;
    int  have_rtt;
    s64  loss_time;                     /* time-threshold loss timer       */
    s64  last_ack_eliciting_sent, last_ack_time;
    int  pto_count, probes;
    double next_send_time;

    bbr3_t bbr;

    u64  pkts_sent, retx_sent, lost_pkts, spurious, acks_rcvd;
    s64  start;
    int  done, aborted;
} xfer_t;

static int    g_quiet;
static FILE  *g_log;
static double g_timer_gran_us;          /* how coarse this system's sleeps are */

/* ------------------------------------------------------------------ */
static void die(const char *m)
{
    perror(m);
    exit(1);
}

static void *xrealloc(void *p, size_t n)
{
    p = realloc(p, n);
    if (!p) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }
    return p;
}

static void *xcalloc(size_t n, size_t sz)
{
    void *p = calloc(n, sz);
    if (!p) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }
    return p;
}

static void send_to_peer(xfer_t *x, const u8 *buf, size_t len)
{
    if (sendto(x->fd, (const char *)buf, len, 0, (struct sockaddr *)&x->peer,
               sizeof(x->peer)) < 0 && errno != EAGAIN && errno != ENOBUFS)
        perror("sendto");
}

static void send_meta(int fd, struct sockaddr_in *to, int status, u32 session, u64 size)
{
    u8 b[32], *p = b;
    *p++ = PKT_META;
    *p++ = (u8)status;
    p = put32(p, session);
    p = put64(p, size);
    p = put16(p, CHUNK_SIZE);
    sendto(fd, (char *)b, p - b, 0, (struct sockaddr *)to, sizeof(*to));
}

/* Old kernels (HZ=100) can't sleep for less than ~10 ms; measure it so
 * the pacer may catch up with a burst after oversleeping. */
static void measure_timer_granularity(void)
{
    int i;
    g_timer_gran_us = 0;
    for (i = 0; i < 3; i++) {
        s64 t = now_us();
        wait_readable(-1, -1, 100);
        t = now_us() - t;
        if ((double)t > g_timer_gran_us)
            g_timer_gran_us = (double)t;
    }
}

/* ------------------------------------------------------------------ */
/* Retransmission queue                                                  */
/* ------------------------------------------------------------------ */
static void queue_retx(xfer_t *x, u64 chunk)
{
    if (x->chunk_acked[chunk] || x->chunk_queued[chunk])
        return;
    x->retx[x->retx_tail % x->retx_cap] = chunk;
    x->retx_tail++;
    x->chunk_queued[chunk] = 1;
}

static int have_data_to_send(xfer_t *x)
{
    for (;;) {                          /* drop chunks acked while queued */
        u64 c;
        if (x->retx_head == x->retx_tail)
            return x->next_new_chunk < x->n_chunks;
        c = x->retx[x->retx_head % x->retx_cap];
        if (!x->chunk_acked[c])
            return 1;
        x->chunk_queued[c] = 0;
        x->retx_head++;
    }
}

static int next_chunk(xfer_t *x, u64 *chunk, int *is_retx)
{
    if (!have_data_to_send(x))
        return 0;
    if (x->retx_head != x->retx_tail) {
        *chunk = x->retx[x->retx_head % x->retx_cap];
        x->retx_head++;
        x->chunk_queued[*chunk] = 0;
        *is_retx = 1;
        return 1;
    }
    *chunk = x->next_new_chunk++;
    *is_retx = 0;
    return 1;
}

/* ------------------------------------------------------------------ */
/* Sending                                                               */
/* ------------------------------------------------------------------ */
static void send_data(xfer_t *x, u64 chunk, int is_retx, s64 now)
{
    u8 buf[MAX_PKT], *p = buf;
    u64 off = chunk * CHUNK_SIZE;
    u32 len = (u32)((x->size - off) < CHUNK_SIZE ? (x->size - off) : CHUNK_SIZE);
    u64 pn = x->pn_next++;
    sent_t *s;

    if (pn >= x->sent_cap) {
        u64 ncap = x->sent_cap ? x->sent_cap * 2 : 65536;
        x->sent = (sent_t *)xrealloc(x->sent, (size_t)ncap * sizeof(sent_t));
        x->sent_cap = ncap;
    }
    *p++ = PKT_DATA;
    p = put32(p, x->session);
    p = put64(p, pn);
    p = put64(p, chunk);
    p = put16(p, (u16)len);
    if (read_at(x->file_fd, p, len, off) < 0)
        die("read");
    p += len;

    s = &x->sent[pn];
    s->chunk = chunk;
    s->state = ST_INFLIGHT;
    bbr3_on_send(&x->bbr, &s->info, (u32)(p - buf), now);
    send_to_peer(x, buf, p - buf);

    x->pkts_sent++;
    if (is_retx)
        x->retx_sent++;
    x->last_ack_eliciting_sent = now;
}

/* ------------------------------------------------------------------ */
/* RTT, loss detection, PTO (RFC 9002 style)                             */
/* ------------------------------------------------------------------ */
static void update_rtt(xfer_t *x, s64 latest, s64 ack_delay)
{
    s64 adjusted, diff;
    if (!x->have_rtt) {
        x->have_rtt = 1;
        x->min_rtt = latest;
        x->srtt = latest;
        x->rttvar = latest / 2;
        x->latest_rtt = latest;
        return;
    }
    if (latest < x->min_rtt)
        x->min_rtt = latest;
    adjusted = latest;
    if (latest >= x->min_rtt + ack_delay)
        adjusted = latest - ack_delay;
    diff = x->srtt > adjusted ? x->srtt - adjusted : adjusted - x->srtt;
    x->rttvar = (3 * x->rttvar + diff) / 4;
    x->srtt = (7 * x->srtt + adjusted) / 8;
    x->latest_rtt = adjusted;
}

static void advance_first_unacked(xfer_t *x)
{
    while (x->first_unacked < x->pn_next &&
           x->sent[x->first_unacked].state != ST_INFLIGHT)
        x->first_unacked++;
}

/* Must be called between bbr3_ack_begin() and bbr3_ack_end(). */
static void detect_losses(xfer_t *x, s64 now)
{
    s64 base, loss_delay, lost_send_time;
    u64 pn;

    x->loss_time = 0;
    if (x->largest_acked < 0)
        return;
    base = x->have_rtt ? (x->srtt > x->latest_rtt ? x->srtt : x->latest_rtt) : 333000;
    loss_delay = base * 9 / 8;
    if (loss_delay < 1000)
        loss_delay = 1000;
    lost_send_time = now - loss_delay;

    for (pn = x->first_unacked; (s64)pn < x->largest_acked; pn++) {
        sent_t *s = &x->sent[pn];
        if (s->state != ST_INFLIGHT)
            continue;
        if ((u64)x->largest_acked - pn >= 3 || s->info.sent_time <= lost_send_time) {
            s->state = ST_LOST;
            x->lost_pkts++;
            bbr3_on_packet_lost(&x->bbr, &s->info, now);
            queue_retx(x, s->chunk);
        } else {
            s64 t = s->info.sent_time + loss_delay;
            if (x->loss_time == 0 || t < x->loss_time)
                x->loss_time = t;
        }
    }
    advance_first_unacked(x);
}

static s64 pto_deadline(xfer_t *x)
{
    s64 base = x->have_rtt
        ? x->srtt + (4 * x->rttvar > 1000 ? 4 * x->rttvar : 1000) + MAX_ACK_DELAY_US
        : 300000;
    int shift = x->pto_count < 10 ? x->pto_count : 10;
    return x->last_ack_eliciting_sent + (base << shift);
}

static void on_pto(xfer_t *x)
{
    /* Probe with the oldest outstanding data; its ACK reveals what was lost. */
    int queued = 0;
    u64 pn;
    x->pto_count++;
    x->probes = 2;
    for (pn = x->first_unacked; pn < x->pn_next && queued < 2; pn++) {
        if (x->sent[pn].state == ST_INFLIGHT && !x->chunk_acked[x->sent[pn].chunk]) {
            queue_retx(x, x->sent[pn].chunk);
            queued++;
        }
    }
}

/* ------------------------------------------------------------------ */
/* ACK processing                                                        */
/* ------------------------------------------------------------------ */
static void mark_chunk_acked(xfer_t *x, u64 chunk)
{
    if (!x->chunk_acked[chunk]) {
        x->chunk_acked[chunk] = 1;
        x->acked_chunks++;
    }
}

static void on_ack(xfer_t *x, const u8 *b, size_t len, s64 now)
{
    u64 largest;
    s64 ack_delay, largest_newly = -1, rtt = -1;
    unsigned n, i;

    if (len < 1 + 4 + 8 + 4 + 2)
        return;
    largest = get64(b + 5);
    ack_delay = get32(b + 13);
    n = get16(b + 17);
    if (n > MAX_ACK_RANGES || len < 19 + (size_t)n * 16 || largest >= x->pn_next)
        return;

    x->acks_rcvd++;
    bbr3_ack_begin(&x->bbr);

    for (i = 0; i < n; i++) {
        u64 hi = get64(b + 19 + i * 16);
        u64 lo = get64(b + 27 + i * 16);
        u64 from, pn;
        if (lo > hi || hi >= x->pn_next)
            continue;
        from = lo > x->first_unacked ? lo : x->first_unacked;
        for (pn = from; pn <= hi; pn++) {
            sent_t *s = &x->sent[pn];
            if (s->state == ST_INFLIGHT) {
                s->state = ST_ACKED;
                bbr3_on_packet_acked(&x->bbr, &s->info, now);
                mark_chunk_acked(x, s->chunk);
                if ((s64)pn > largest_newly)
                    largest_newly = (s64)pn;
            } else if (s->state == ST_LOST) {
                s->state = ST_ACKED;          /* declared lost too early */
                x->spurious++;
                mark_chunk_acked(x, s->chunk);
            }
        }
    }

    if ((s64)largest > x->largest_acked)
        x->largest_acked = (s64)largest;

    if (largest_newly >= 0) {
        if ((u64)largest_newly == largest) {
            update_rtt(x, now - x->sent[largest].info.sent_time, ack_delay);
            rtt = x->latest_rtt;
        }
        x->pto_count = 0;
        x->last_ack_time = now;
    }

    detect_losses(x, now);
    bbr3_ack_end(&x->bbr, rtt, now);
    advance_first_unacked(x);

    if (x->acked_chunks == x->n_chunks)
        x->done = 1;
}

/* ------------------------------------------------------------------ */
/* Logging                                                               */
/* ------------------------------------------------------------------ */
static double mbps(double bytes_per_s) { return bytes_per_s * 8 / 1e6; }

static double min_rtt_ms(bbr3_t *b)
{
    return b->min_rtt == S64_MAX ? 0.0 : (double)b->min_rtt / 1e3;
}

static void log_state(xfer_t *x, s64 now)
{
    bbr3_t *b = &x->bbr;
    if (g_log)
        fprintf(g_log, "%.3f,%s,%.3f,%.3f,%.3f,%.0f,%.0f,%.3f,%.3f,%.0f,%.0f,%.0f\n",
                (double)(now - x->start) / 1e3, bbr3_state_name(b), mbps(b->bw),
                mbps(b->max_bw), mbps(b->pacing_rate), b->cwnd, (double)b->inflight,
                min_rtt_ms(b), (double)x->srtt / 1e3,
                b->inflight_hi >= HUGE_VAL ? -1.0 : b->inflight_hi,
                (double)x->lost_pkts, (double)x->acked_chunks);
}

static void print_status(xfer_t *x, s64 now)
{
    bbr3_t *b = &x->bbr;
    char hi[32];
    if (b->inflight_hi >= HUGE_VAL)
        strcpy(hi, "inf");
    else
        sprintf(hi, "%.0fK", b->inflight_hi / 1024);
    fprintf(stderr,
            "%6.1fs %-14s bw %7.2f Mb/s pacing %7.2f cwnd %6.0fK inflight %6.0fK "
            "inflight_hi %6s min_rtt %6.1fms srtt %6.1fms lost %.0f  %5.1f%%\n",
            (double)(now - x->start) / 1e6, bbr3_state_name(b), mbps(b->bw),
            mbps(b->pacing_rate), b->cwnd / 1024, (double)b->inflight / 1024.0, hi,
            min_rtt_ms(b), (double)x->srtt / 1e3, (double)x->lost_pkts,
            x->n_chunks ? 100.0 * (double)x->acked_chunks / (double)x->n_chunks : 100.0);
}

/* ------------------------------------------------------------------ */
/* One transfer                                                          */
/* ------------------------------------------------------------------ */
static void serve(int fd, struct sockaddr_in *peer, int file_fd, u64 size)
{
    xfer_t *x = (xfer_t *)xcalloc(1, sizeof(*x));
    s64 next_print, next_log, end;
    u8 buf[MAX_PKT];
    double secs;

    x->fd = fd;
    x->peer = *peer;
    x->session = (u32)(now_us() ^ ((s64)getpid() << 16)) | 1;
    x->file_fd = file_fd;
    x->size = size;
    x->n_chunks = (size + CHUNK_SIZE - 1) / CHUNK_SIZE;
    x->chunk_acked = (u8 *)xcalloc((size_t)x->n_chunks + 1, 1);
    x->chunk_queued = (u8 *)xcalloc((size_t)x->n_chunks + 1, 1);
    x->retx_cap = x->n_chunks + 64;
    x->retx = (u64 *)xcalloc((size_t)x->retx_cap, sizeof(u64));
    x->largest_acked = -1;
    x->start = now_us();
    x->last_ack_time = x->start;
    x->next_send_time = (double)x->start;
    bbr3_init(&x->bbr, PKT_SIZE, x->start);

    send_meta(fd, peer, 0, x->session, size);
    if (x->n_chunks == 0)
        x->done = 1;

    next_print = x->start + 1000000;
    next_log = x->start;

    while (!x->done) {
        s64 now = now_us(), wake;
        int sent_now = 0;

        /* 1. read everything waiting */
        for (;;) {
            struct sockaddr_in from;
            sock_len fl = sizeof(from);
            int same_peer;
            ssize_t r = recvfrom(fd, (char *)buf, sizeof(buf), 0,
                                 (struct sockaddr *)&from, &fl);
            if (r <= 0)
                break;
            same_peer = from.sin_addr.s_addr == peer->sin_addr.s_addr &&
                        from.sin_port == peer->sin_port;
            now = now_us();
            if (buf[0] == PKT_REQ && same_peer) {
                send_meta(fd, peer, 0, x->session, size);   /* META was lost */
            } else if (r >= 5 && same_peer && get32(buf + 1) == x->session) {
                if (buf[0] == PKT_ACK)
                    on_ack(x, buf, (size_t)r, now);
                else if (buf[0] == PKT_DONE)
                    x->done = 1;
            } else if (buf[0] == PKT_REQ) {
                send_meta(fd, &from, 1, 0, 0);              /* busy */
            }
        }
        if (x->done)
            break;

        /* 2. timers */
        now = now_us();
        if (x->loss_time && now >= x->loss_time) {
            bbr3_ack_begin(&x->bbr);
            detect_losses(x, now);
            bbr3_ack_end(&x->bbr, -1, now);
        }
        if (x->bbr.inflight > 0 && now >= pto_deadline(x))
            on_pto(x);
        if (now - x->last_ack_time > IDLE_ABORT_US) {
            fprintf(stderr, "no ACKs for %d s, aborting transfer\n", IDLE_ABORT_US / 1000000);
            x->aborted = 1;
            break;
        }

        /* 3. send as much as cwnd and pacing allow */
        while (sent_now < SEND_BATCH) {
            int over_cwnd, is_retx = 0;
            double rate, burst_us;
            u64 chunk = 0;

            if (!have_data_to_send(x)) {
                x->bbr.is_cwnd_limited = 0;
                bbr3_on_app_limited(&x->bbr);
                break;
            }
            over_cwnd = (double)(x->bbr.inflight + PKT_SIZE) > x->bbr.cwnd;
            if (over_cwnd && x->probes == 0) {
                x->bbr.is_cwnd_limited = 1;
                break;
            }
            rate = x->bbr.pacing_rate > 1000 ? x->bbr.pacing_rate : 1000;
            burst_us = x->bbr.send_quantum / rate * 1e6;
            if (burst_us < 2 * g_timer_gran_us)
                burst_us = 2 * g_timer_gran_us;         /* coarse sleeps */
            if (x->next_send_time < (double)now - burst_us)
                x->next_send_time = (double)now - burst_us; /* don't bank idle time */
            if (x->next_send_time > (double)now)
                break;                                      /* pacing */

            if (!next_chunk(x, &chunk, &is_retx))
                break;
            send_data(x, chunk, is_retx, now);
            if (over_cwnd && x->probes > 0)
                x->probes--;
            x->next_send_time += PKT_SIZE / rate * 1e6;
            x->bbr.is_cwnd_limited = (double)(x->bbr.inflight + PKT_SIZE) > x->bbr.cwnd;
            sent_now++;
        }

        /* 4. status output */
        if (now >= next_print) {
            if (!g_quiet)
                print_status(x, now);
            next_print += 1000000;
        }
        if (g_log && now >= next_log) {
            log_state(x, now);
            next_log = now + 5000;
        }

        /* 5. sleep until something can happen */
        wake = now + 50000;
        if (sent_now == SEND_BATCH)
            wake = now;
        else if (have_data_to_send(x) &&
                 ((double)(x->bbr.inflight + PKT_SIZE) <= x->bbr.cwnd || x->probes))
            wake = (s64)x->next_send_time;
        if (x->loss_time && x->loss_time < wake)
            wake = x->loss_time;
        if (x->bbr.inflight > 0 && pto_deadline(x) < wake)
            wake = pto_deadline(x);
        if (next_print < wake)
            wake = next_print;
        wait_readable(fd, -1, wake - now_us());
    }

    end = now_us();
    secs = (double)(end - x->start) / 1e6;
    if (!g_quiet || x->aborted) {
        print_status(x, end);
        fprintf(stderr,
                "%s: %.0f bytes in %.2f s = %.2f Mb/s goodput | packets %.0f, "
                "retransmitted %.0f (%.2f%%), declared lost %.0f, spurious %.0f | "
                "min_rtt %.1f ms, srtt %.1f ms\n",
                x->aborted ? "ABORTED" : "done", (double)size, secs,
                secs > 0 ? (double)size * 8 / secs / 1e6 : 0.0, (double)x->pkts_sent,
                (double)x->retx_sent,
                x->pkts_sent ? 100.0 * (double)x->retx_sent / (double)x->pkts_sent : 0.0,
                (double)x->lost_pkts, (double)x->spurious,
                (double)x->min_rtt / 1e3, (double)x->srtt / 1e3);
    }
    if (g_log)
        fflush(g_log);
    free(x->sent);
    free(x->chunk_acked);
    free(x->chunk_queued);
    free(x->retx);
    free(x);
}

/* ------------------------------------------------------------------ */
static int safe_name(const char *n)
{
    return n[0] && n[0] != '.' && !strchr(n, '/') && !strchr(n, '\\') && !strstr(n, "..");
}

int main(int argc, char **argv)
{
    int port = 9001, opt, fd, bufsz = 8 << 20;
    const char *dir = ".";
    struct sockaddr_in a, last_peer;
    u8 buf[MAX_PKT];
    char last_name[256];
    s64 last_end = 0;

    while ((opt = getopt(argc, argv, "p:d:l:qh")) != -1) {
        switch (opt) {
        case 'p': port = atoi(optarg); break;
        case 'd': dir = optarg; break;
        case 'l':
            g_log = fopen(optarg, "w");
            if (!g_log)
                die(optarg);
            fprintf(g_log, "t_ms,state,bw_mbps,max_bw_mbps,pacing_mbps,cwnd,inflight,"
                           "min_rtt_ms,srtt_ms,inflight_hi,lost_pkts,acked_chunks\n");
            break;
        case 'q': g_quiet = 1; break;
        default:
            fprintf(stderr, "usage: %s [-p port] [-d dir] [-l log.csv] [-q]\n", argv[0]);
            return 1;
        }
    }
    signal(SIGPIPE, SIG_IGN);
    precise_timers();
    measure_timer_granularity();

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
        die("socket");
    setsockopt(fd, SOL_SOCKET, SO_SNDBUF, (char *)&bufsz, sizeof bufsz);
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char *)&bufsz, sizeof bufsz);
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons((unsigned short)port);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(fd, (struct sockaddr *)&a, sizeof a) < 0)
        die("bind");
    set_nonblock(fd);
    fprintf(stderr, "serving %s on UDP port %d (timer granularity %.0f us)\n",
            dir, port, g_timer_gran_us);

    memset(&last_peer, 0, sizeof last_peer);
    last_name[0] = 0;
    for (;;) {
        struct sockaddr_in from;
        sock_len fl = sizeof(from);
        char name[256], path[4096];
        int ffd;
        struct stat st;
        ssize_t r;

        wait_readable(fd, -1, 1000000);
        r = recvfrom(fd, (char *)buf, sizeof(buf), 0, (struct sockaddr *)&from, &fl);
        if (r < 2 || buf[0] != PKT_REQ || buf[1] == 0 || 2 + (ssize_t)buf[1] > r)
            continue;
        memcpy(name, buf + 2, buf[1]);
        name[buf[1]] = 0;
        if (from.sin_addr.s_addr == last_peer.sin_addr.s_addr &&
            from.sin_port == last_peer.sin_port && !strcmp(name, last_name) &&
            now_us() - last_end < 3000000)
            continue;      /* late duplicate of a request we just finished */
        if (strlen(dir) + strlen(name) + 2 > sizeof path)
            continue;
        strcpy(path, dir);
        strcat(path, "/");
        strcat(path, name);
        ffd = safe_name(name) ? open(path, O_RDONLY) : -1;
        if (ffd < 0 || fstat(ffd, &st) < 0 || !S_ISREG(st.st_mode)) {
            if (ffd >= 0)
                close(ffd);
            fprintf(stderr, "%s:%d requested '%s': not found\n",
                    inet_ntoa(from.sin_addr), ntohs(from.sin_port), name);
            send_meta(fd, &from, 1, 0, 0);
            continue;
        }
        fprintf(stderr, "%s:%d requested '%s' (%.0f bytes)\n",
                inet_ntoa(from.sin_addr), ntohs(from.sin_port), name, (double)st.st_size);
        serve(fd, &from, ffd, (u64)st.st_size);
        close(ffd);
        last_peer = from;
        strcpy(last_name, name);
        last_end = now_us();
    }
}
