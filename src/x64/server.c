/*
 * server.c - UDP file server that sends with BBRv3 congestion control.
 *
 *   ./udpbbr_server [-p port] [-d directory] [-l log.csv] [-q]
 *                   [-D] [-o logfile] [-P pidfile]
 *
 *   -D  run as a daemon: detach from the terminal, log to -o (or discard),
 *       write the PID to -P; SIGTERM or SIGINT stops it cleanly.
 *
 * A client sends REQ <filename>; the server answers META and streams the
 * file in CHUNK_SIZE pieces. Reliability is QUIC-like: monotonically
 * increasing packet numbers, SACK-style ACK ranges, packet- and
 * time-threshold loss detection (RFC 9002), and a probe timeout (PTO) for
 * tail losses. BBRv3 decides how much may be in flight (cwnd) and how fast
 * packets leave (pacing rate).
 *
 * One transfer at a time, to keep the example readable.
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <math.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "bbr3.h"
#include "proto.h"

#define ST_INFLIGHT 0
#define ST_ACKED    1
#define ST_LOST     2

#define MAX_ACK_DELAY_US   2000        /* receiver acks within ~1 ms       */
#define IDLE_ABORT_US      20000000    /* give up after 20 s without ACKs  */
#define PKT_SIZE           (DATA_HDR + CHUNK_SIZE)

typedef struct {
    bbr_pkt_t info;
    uint64_t  chunk;
    uint8_t   state;
} sent_t;

typedef struct {
    int fd;
    struct sockaddr_in peer;
    uint32_t session;

    int      file_fd;
    uint64_t size, n_chunks;
    uint8_t *chunk_acked, *chunk_queued;
    uint64_t acked_chunks, next_new_chunk;
    uint64_t *retx;                     /* ring buffer of chunks to resend */
    uint64_t retx_head, retx_tail, retx_cap;

    sent_t  *sent;                      /* indexed by packet number        */
    uint64_t sent_cap, pn_next, first_unacked;
    int64_t  largest_acked;

    int64_t  srtt, rttvar, min_rtt, latest_rtt;
    int      have_rtt;
    int64_t  loss_time;                 /* time-threshold loss timer       */
    int64_t  last_ack_eliciting_sent, last_ack_time;
    int      pto_count, probes;
    double   next_send_time;

    bbr3_t   bbr;

    uint64_t pkts_sent, retx_sent, lost_pkts, spurious, acks_rcvd;
    int64_t  start;
    int      done, aborted;
} xfer_t;

static int   g_quiet;
static FILE *g_log;
static int   g_daemon;
static const char *g_pidfile;
static volatile sig_atomic_t g_stop;     /* set by SIGTERM / SIGINT */

/* Timestamp prefix for event lines (useful in a daemon's log file). */
static void stamp(void)
{
    char tb[32];
    time_t t = time(NULL);
    strftime(tb, sizeof tb, "%Y-%m-%d %H:%M:%S ", localtime(&t));
    fputs(tb, stderr);
}

/* ------------------------------------------------------------------ */
static void die(const char *m) { perror(m); exit(1); }

static void *xrealloc(void *p, size_t n)
{
    p = realloc(p, n);
    if (!p) { fprintf(stderr, "out of memory\n"); exit(1); }
    return p;
}

static void send_to_peer(xfer_t *x, const uint8_t *buf, size_t len)
{
    if (sendto(x->fd, buf, len, 0, (struct sockaddr *)&x->peer, sizeof(x->peer)) < 0 &&
        errno != EAGAIN && errno != ENOBUFS)
        perror("sendto");
}

static void send_meta(int fd, struct sockaddr_in *to, int status,
                      uint32_t session, uint64_t size)
{
    uint8_t b[32], *p = b;
    *p++ = PKT_META;
    *p++ = (uint8_t)status;
    p = put32(p, session);
    p = put64(p, size);
    p = put16(p, CHUNK_SIZE);
    sendto(fd, b, p - b, 0, (struct sockaddr *)to, sizeof(*to));
}

/* ------------------------------------------------------------------ */
/* Retransmission queue                                                  */
/* ------------------------------------------------------------------ */
static void queue_retx(xfer_t *x, uint64_t chunk)
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
        if (x->retx_head == x->retx_tail)
            return x->next_new_chunk < x->n_chunks;
        uint64_t c = x->retx[x->retx_head % x->retx_cap];
        if (!x->chunk_acked[c])
            return 1;
        x->chunk_queued[c] = 0;
        x->retx_head++;
    }
}

static int next_chunk(xfer_t *x, uint64_t *chunk, int *is_retx)
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
static void send_data(xfer_t *x, uint64_t chunk, int is_retx, int64_t now)
{
    uint8_t buf[MAX_PKT], *p = buf;
    uint64_t off = chunk * CHUNK_SIZE;
    uint32_t len = (uint32_t)((x->size - off) < CHUNK_SIZE ? (x->size - off) : CHUNK_SIZE);
    uint64_t pn = x->pn_next++;

    if (pn >= x->sent_cap) {
        uint64_t ncap = x->sent_cap ? x->sent_cap * 2 : 65536;
        x->sent = xrealloc(x->sent, ncap * sizeof(sent_t));
        x->sent_cap = ncap;
    }
    *p++ = PKT_DATA;
    p = put32(p, x->session);
    p = put64(p, pn);
    p = put64(p, chunk);
    p = put16(p, (uint16_t)len);
    if (pread(x->file_fd, p, len, (off_t)off) != (ssize_t)len)
        die("pread");
    p += len;

    sent_t *s = &x->sent[pn];
    s->chunk = chunk;
    s->state = ST_INFLIGHT;
    bbr3_on_send(&x->bbr, &s->info, (uint32_t)(p - buf), now);
    send_to_peer(x, buf, p - buf);

    x->pkts_sent++;
    if (is_retx) x->retx_sent++;
    x->last_ack_eliciting_sent = now;
}

/* ------------------------------------------------------------------ */
/* RTT, loss detection, PTO (RFC 9002 style)                             */
/* ------------------------------------------------------------------ */
static void update_rtt(xfer_t *x, int64_t latest, int64_t ack_delay)
{
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
    int64_t adjusted = latest;
    if (latest >= x->min_rtt + ack_delay)
        adjusted = latest - ack_delay;
    int64_t diff = x->srtt > adjusted ? x->srtt - adjusted : adjusted - x->srtt;
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
static void detect_losses(xfer_t *x, int64_t now)
{
    x->loss_time = 0;
    if (x->largest_acked < 0)
        return;
    int64_t base = x->have_rtt ? (x->srtt > x->latest_rtt ? x->srtt : x->latest_rtt) : 333000;
    int64_t loss_delay = base * 9 / 8;
    if (loss_delay < 1000) loss_delay = 1000;
    int64_t lost_send_time = now - loss_delay;

    for (uint64_t pn = x->first_unacked; (int64_t)pn < x->largest_acked; pn++) {
        sent_t *s = &x->sent[pn];
        if (s->state != ST_INFLIGHT)
            continue;
        if ((uint64_t)x->largest_acked - pn >= 3 || s->info.sent_time <= lost_send_time) {
            s->state = ST_LOST;
            x->lost_pkts++;
            bbr3_on_packet_lost(&x->bbr, &s->info, now);
            queue_retx(x, s->chunk);
        } else {
            int64_t t = s->info.sent_time + loss_delay;
            if (x->loss_time == 0 || t < x->loss_time)
                x->loss_time = t;
        }
    }
    advance_first_unacked(x);
}

static int64_t pto_deadline(xfer_t *x)
{
    int64_t base = x->have_rtt
        ? x->srtt + (4 * x->rttvar > 1000 ? 4 * x->rttvar : 1000) + MAX_ACK_DELAY_US
        : 300000;
    int shift = x->pto_count < 10 ? x->pto_count : 10;
    return x->last_ack_eliciting_sent + (base << shift);
}

static void on_pto(xfer_t *x)
{
    /* Probe with the oldest outstanding data; its ACK reveals what was lost. */
    x->pto_count++;
    x->probes = 2;
    int queued = 0;
    for (uint64_t pn = x->first_unacked; pn < x->pn_next && queued < 2; pn++) {
        if (x->sent[pn].state == ST_INFLIGHT && !x->chunk_acked[x->sent[pn].chunk]) {
            queue_retx(x, x->sent[pn].chunk);
            queued++;
        }
    }
}

/* ------------------------------------------------------------------ */
/* ACK processing                                                        */
/* ------------------------------------------------------------------ */
static void mark_chunk_acked(xfer_t *x, uint64_t chunk)
{
    if (!x->chunk_acked[chunk]) {
        x->chunk_acked[chunk] = 1;
        x->acked_chunks++;
    }
}

static void on_ack(xfer_t *x, const uint8_t *b, size_t len, int64_t now)
{
    if (len < 1 + 4 + 8 + 4 + 2)
        return;
    uint64_t largest = get64(b + 5);
    int64_t ack_delay = get32(b + 13);
    unsigned n = get16(b + 17);
    if (n > MAX_ACK_RANGES || len < 19 + (size_t)n * 16 || largest >= x->pn_next)
        return;

    x->acks_rcvd++;
    bbr3_ack_begin(&x->bbr);
    int64_t largest_newly = -1;

    for (unsigned i = 0; i < n; i++) {
        uint64_t hi = get64(b + 19 + i * 16);
        uint64_t lo = get64(b + 27 + i * 16);
        if (lo > hi || hi >= x->pn_next)
            continue;
        uint64_t from = lo > x->first_unacked ? lo : x->first_unacked;
        for (uint64_t pn = from; pn <= hi; pn++) {
            sent_t *s = &x->sent[pn];
            if (s->state == ST_INFLIGHT) {
                s->state = ST_ACKED;
                bbr3_on_packet_acked(&x->bbr, &s->info, now);
                mark_chunk_acked(x, s->chunk);
                if ((int64_t)pn > largest_newly)
                    largest_newly = (int64_t)pn;
            } else if (s->state == ST_LOST) {
                s->state = ST_ACKED;          /* declared lost too early */
                x->spurious++;
                mark_chunk_acked(x, s->chunk);
            }
        }
    }

    if ((int64_t)largest > x->largest_acked)
        x->largest_acked = (int64_t)largest;

    int64_t rtt = -1;
    if (largest_newly >= 0) {
        if ((uint64_t)largest_newly == largest) {
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

static void log_state(xfer_t *x, int64_t now)
{
    bbr3_t *b = &x->bbr;
    if (g_log)
        fprintf(g_log, "%.3f,%s,%.3f,%.3f,%.3f,%.0f,%llu,%.3f,%.3f,%.0f,%llu,%llu\n",
                (now - x->start) / 1e3, bbr3_state_name(b), mbps(b->bw), mbps(b->max_bw),
                mbps(b->pacing_rate), b->cwnd, (unsigned long long)b->inflight,
                b->min_rtt == INT64_MAX ? 0 : b->min_rtt / 1e3, x->srtt / 1e3,
                isinf(b->inflight_hi) ? -1 : b->inflight_hi,
                (unsigned long long)x->lost_pkts, (unsigned long long)x->acked_chunks);
}

static void print_status(xfer_t *x, int64_t now)
{
    bbr3_t *b = &x->bbr;
    char hi[32];
    if (isinf(b->inflight_hi)) snprintf(hi, sizeof hi, "inf");
    else snprintf(hi, sizeof hi, "%.0fK", b->inflight_hi / 1024);
    fprintf(stderr,
            "%6.1fs %-14s bw %7.2f Mb/s pacing %7.2f cwnd %6.0fK inflight %6.0fK "
            "inflight_hi %6s min_rtt %6.1fms srtt %6.1fms lost %llu  %5.1f%%\n",
            (now - x->start) / 1e6, bbr3_state_name(b), mbps(b->bw), mbps(b->pacing_rate),
            b->cwnd / 1024, b->inflight / 1024.0, hi,
            b->min_rtt == INT64_MAX ? 0 : b->min_rtt / 1e3, x->srtt / 1e3,
            (unsigned long long)x->lost_pkts, 100.0 * x->acked_chunks / x->n_chunks);
}

/* ------------------------------------------------------------------ */
/* One transfer                                                          */
/* ------------------------------------------------------------------ */
static void serve(int fd, struct sockaddr_in *peer, int file_fd, uint64_t size)
{
    xfer_t *x = calloc(1, sizeof(*x));
    x->fd = fd;
    x->peer = *peer;
    x->session = (uint32_t)(now_us() ^ (getpid() << 16)) | 1;
    x->file_fd = file_fd;
    x->size = size;
    x->n_chunks = (size + CHUNK_SIZE - 1) / CHUNK_SIZE;
    x->chunk_acked = calloc(x->n_chunks + 1, 1);
    x->chunk_queued = calloc(x->n_chunks + 1, 1);
    x->retx_cap = x->n_chunks + 64;
    x->retx = calloc(x->retx_cap, sizeof(uint64_t));
    x->largest_acked = -1;
    x->start = now_us();
    x->last_ack_time = x->start;
    x->next_send_time = (double)x->start;
    bbr3_init(&x->bbr, PKT_SIZE, x->start);

    send_meta(fd, peer, 0, x->session, size);
    if (x->n_chunks == 0)
        x->done = 1;

    int64_t next_print = x->start + 1000000, next_log = x->start;
    uint8_t buf[MAX_PKT];

    while (!x->done && !g_stop) {
        int64_t now = now_us();

        /* 1. read everything waiting */
        for (;;) {
            struct sockaddr_in from;
            socklen_t fl = sizeof(from);
            ssize_t r = recvfrom(fd, buf, sizeof(buf), MSG_DONTWAIT,
                                 (struct sockaddr *)&from, &fl);
            if (r <= 0)
                break;
            int same_peer = from.sin_addr.s_addr == peer->sin_addr.s_addr &&
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
            stamp();
            fprintf(stderr, "no ACKs for %d s, aborting transfer\n", IDLE_ABORT_US / 1000000);
            x->aborted = 1;
            break;
        }

        /* 3. send as much as cwnd and pacing allow */
        int sent_now = 0;
        while (sent_now < 64) {
            if (!have_data_to_send(x)) {
                x->bbr.is_cwnd_limited = 0;
                bbr3_on_app_limited(&x->bbr);
                break;
            }
            int over_cwnd = (double)(x->bbr.inflight + PKT_SIZE) > x->bbr.cwnd;
            if (over_cwnd && x->probes == 0) {
                x->bbr.is_cwnd_limited = 1;
                break;
            }
            double rate = x->bbr.pacing_rate > 1000 ? x->bbr.pacing_rate : 1000;
            double burst_us = x->bbr.send_quantum / rate * 1e6;
            if (x->next_send_time < now - burst_us)
                x->next_send_time = now - burst_us;     /* don't bank idle time */
            if (x->next_send_time > (double)now)
                break;                                  /* pacing */

            uint64_t chunk = 0;
            int is_retx = 0;
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
            if (!g_quiet) print_status(x, now);
            next_print += 1000000;
        }
        if (g_log && now >= next_log) {
            log_state(x, now);
            next_log = now + 5000;
        }

        /* 5. sleep until something can happen */
        int64_t wake = now + 50000;
        if (sent_now == 64)
            wake = now;
        else if (have_data_to_send(x) &&
                 ((double)(x->bbr.inflight + PKT_SIZE) <= x->bbr.cwnd || x->probes))
            wake = (int64_t)x->next_send_time;
        if (x->loss_time && x->loss_time < wake) wake = x->loss_time;
        if (x->bbr.inflight > 0 && pto_deadline(x) < wake) wake = pto_deadline(x);
        if (next_print < wake) wake = next_print;
        int64_t wait = wake - now_us();
        if (wait < 0) wait = 0;
        struct pollfd pfd = { .fd = fd, .events = POLLIN };
        struct timespec ts = { wait / 1000000, (wait % 1000000) * 1000 };
        ppoll(&pfd, 1, &ts, NULL);
    }

    int64_t end = now_us();
    double secs = (end - x->start) / 1e6;
    if (!g_quiet)
        print_status(x, end);
    {
        /* Bytes the client has acknowledged (all of them unless cut short). */
        int complete = x->acked_chunks == x->n_chunks;
        uint64_t got = complete ? size : x->acked_chunks * CHUNK_SIZE;
        if (got > size)
            got = size;                 /* last chunk is short */
        char what[64];
        if (complete)
            snprintf(what, sizeof what, "%llu bytes", (unsigned long long)size);
        else
            snprintf(what, sizeof what, "%llu of %llu bytes",
                     (unsigned long long)got, (unsigned long long)size);
        stamp();
        fprintf(stderr,
                "%s: %s in %.2f s = %.2f Mb/s goodput | packets %llu, "
                "retransmitted %llu (%.2f%%), declared lost %llu, spurious %llu | "
                "min_rtt %.1f ms, srtt %.1f ms\n",
                x->aborted ? "ABORTED" : !complete && g_stop ? "STOPPED" : "done",
                what, secs, secs > 0 ? got * 8 / secs / 1e6 : 0.0,
                (unsigned long long)x->pkts_sent, (unsigned long long)x->retx_sent,
                x->pkts_sent ? 100.0 * x->retx_sent / x->pkts_sent : 0.0,
                (unsigned long long)x->lost_pkts, (unsigned long long)x->spurious,
                x->min_rtt / 1e3, x->srtt / 1e3);
    }
    if (g_log) fflush(g_log);
    free(x->sent); free(x->chunk_acked); free(x->chunk_queued); free(x->retx); free(x);
}

/* ------------------------------------------------------------------ */
static int safe_name(const char *n)
{
    return n[0] && n[0] != '.' && !strchr(n, '/') && !strchr(n, '\\') && !strstr(n, "..");
}

/* ------------------------------------------------------------------ */
/* Daemon support                                                        */
/* ------------------------------------------------------------------ */
static void on_stop_signal(int sig) { (void)sig; g_stop = 1; }

static void remove_pidfile(void)
{
    if (g_pidfile)
        unlink(g_pidfile);
}

/* Refuse to start if the PID file names a process that is still running. */
static void check_pidfile(const char *path)
{
    FILE *f = fopen(path, "r");
    long pid;
    if (!f)
        return;
    if (fscanf(f, "%ld", &pid) == 1 && pid > 0 && kill((pid_t)pid, 0) == 0) {
        fprintf(stderr, "already running as pid %ld (%s)\n", pid, path);
        exit(1);
    }
    fclose(f);                         /* stale: will be overwritten */
}

static void write_pidfile(const char *path)
{
    FILE *f = fopen(path, "w");
    if (!f) {
        stamp();
        fprintf(stderr, "can't write pid file %s: %s\n", path, strerror(errno));
        return;
    }
    fprintf(f, "%ld\n", (long)getpid());
    fclose(f);
}

/* Classic double-fork daemonisation. Output goes to `logpath` (appended)
 * or /dev/null. Paths must already be absolute: we chdir("/"). */
static void daemonize(const char *logpath)
{
    pid_t pid = fork();
    if (pid < 0) die("fork");
    if (pid > 0) _exit(0);             /* parent: back to the shell */
    if (setsid() < 0) die("setsid");   /* new session, no controlling tty */
    signal(SIGHUP, SIG_IGN);
    pid = fork();
    if (pid < 0) die("fork");
    if (pid > 0) _exit(0);             /* not a session leader: can't regain a tty */

    umask(022);
    int nul = open("/dev/null", O_RDWR);
    int out = logpath ? open(logpath, O_WRONLY | O_CREAT | O_APPEND, 0644) : nul;
    if (nul < 0 || out < 0) die(logpath ? logpath : "/dev/null");
    if (chdir("/") < 0) die("chdir");
    dup2(nul, STDIN_FILENO);
    dup2(out, STDOUT_FILENO);
    dup2(out, STDERR_FILENO);
    if (nul > STDERR_FILENO) close(nul);
    if (out > STDERR_FILENO && out != nul) close(out);
}

static char *absolute(const char *path)
{
    char *r = realpath(path, NULL);
    if (!r) die(path);
    return r;
}

/* An absolute path for a file that may not exist yet (its directory must). */
static char *absolute_file(const char *path)
{
    const char *slash = strrchr(path, '/');
    char dirpart[PATH_MAX];
    if (path[0] == '/')
        return strdup(path);
    if (!slash)
        snprintf(dirpart, sizeof dirpart, ".");
    else
        snprintf(dirpart, sizeof dirpart, "%.*s", (int)(slash - path), path);
    char *d = absolute(dirpart);
    size_t n = strlen(d) + strlen(path) + 2;
    char *r = malloc(n);
    snprintf(r, n, "%s/%s", d, slash ? slash + 1 : path);
    free(d);
    return r;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s [-p port] [-d dir] [-l log.csv] [-q] [-D] [-o logfile] [-P pidfile]\n"
            "  -p port     UDP port (default 9001)\n"
            "  -d dir      directory to serve (default .)\n"
            "  -l file     BBR state log, CSV every 5 ms\n"
            "  -q          no per-second status lines\n"
            "  -D          run as a daemon in the background\n"
            "  -o file     with -D: append messages to this file (default: discard)\n"
            "  -P file     write the process ID here (removed on exit)\n",
            argv0);
    exit(1);
}

int main(int argc, char **argv)
{
    int port = 9001, opt;
    const char *dir = ".", *logpath = NULL;
    while ((opt = getopt(argc, argv, "p:d:l:qDo:P:h")) != -1) {
        switch (opt) {
        case 'p': port = atoi(optarg); break;
        case 'd': dir = optarg; break;
        case 'l':
            g_log = fopen(optarg, "w");
            if (!g_log) die(optarg);
            fprintf(g_log, "t_ms,state,bw_mbps,max_bw_mbps,pacing_mbps,cwnd,inflight,"
                           "min_rtt_ms,srtt_ms,inflight_hi,lost_pkts,acked_chunks\n");
            break;
        case 'q': g_quiet = 1; break;
        case 'D': g_daemon = 1; break;
        case 'o': logpath = optarg; break;
        case 'P': g_pidfile = optarg; break;
        default:  usage(argv[0]);
        }
    }
    if (logpath && !g_daemon) {
        fprintf(stderr, "-o only applies with -D (otherwise messages go to stderr)\n");
        return 1;
    }

    /* Resolve paths now: a daemon runs in "/". Fails here, visibly, if the
     * directory doesn't exist. */
    char *root = absolute(dir);
    struct stat dst;
    if (stat(root, &dst) < 0 || !S_ISDIR(dst.st_mode)) {
        fprintf(stderr, "%s: not a directory\n", dir);
        return 1;
    }
    if (logpath) logpath = absolute_file(logpath);
    if (g_pidfile) {
        g_pidfile = absolute_file(g_pidfile);
        check_pidfile(g_pidfile);
    }

    signal(SIGPIPE, SIG_IGN);
    prctl(PR_SET_TIMERSLACK, 1UL);           /* precise sleeps for pacing */

    /* Bind before detaching so "address in use" reaches the terminal. */
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) die("socket");
    int bufsz = 8 << 20;
    setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &bufsz, sizeof bufsz);
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &bufsz, sizeof bufsz);
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(port),
                             .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (bind(fd, (struct sockaddr *)&a, sizeof a) < 0) die("bind");

    if (g_daemon) {
        fprintf(stderr, "serving %s on UDP port %d in the background%s%s\n", root, port,
                logpath ? ", log " : "", logpath ? logpath : "");
        daemonize(logpath);
        g_quiet = 1;                         /* no per-second lines in a log */
    }
    if (g_pidfile) {
        write_pidfile(g_pidfile);
        atexit(remove_pidfile);
    }

    /* No SA_RESTART: a signal interrupts recvfrom()/ppoll() so we can exit. */
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_stop_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);

    stamp();
    fprintf(stderr, "serving %s on UDP port %d (pid %ld)\n", root, port, (long)getpid());

    uint8_t buf[MAX_PKT];
    struct sockaddr_in last_peer = {0};
    char last_name[256] = "";
    int64_t last_end = 0;
    while (!g_stop) {
        struct sockaddr_in from;
        socklen_t fl = sizeof(from);
        ssize_t r = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *)&from, &fl);
        if (r < 2 || buf[0] != PKT_REQ || buf[1] == 0 || 2 + (ssize_t)buf[1] > r)
            continue;
        char name[256];
        memcpy(name, buf + 2, buf[1]);
        name[buf[1]] = 0;
        if (from.sin_addr.s_addr == last_peer.sin_addr.s_addr &&
            from.sin_port == last_peer.sin_port && !strcmp(name, last_name) &&
            now_us() - last_end < 3000000)
            continue;      /* late duplicate of a request we just finished */
        char path[PATH_MAX + 256];
        snprintf(path, sizeof path, "%s/%s", root, name);
        int ffd = safe_name(name) ? open(path, O_RDONLY) : -1;
        struct stat st;
        if (ffd < 0 || fstat(ffd, &st) < 0 || !S_ISREG(st.st_mode)) {
            if (ffd >= 0) close(ffd);
            stamp();
            fprintf(stderr, "%s:%d requested '%s': not found\n",
                    inet_ntoa(from.sin_addr), ntohs(from.sin_port), name);
            send_meta(fd, &from, 1, 0, 0);
            continue;
        }
        stamp();
        fprintf(stderr, "%s:%d requested '%s' (%lld bytes)\n",
                inet_ntoa(from.sin_addr), ntohs(from.sin_port), name, (long long)st.st_size);
        serve(fd, &from, ffd, (uint64_t)st.st_size);
        close(ffd);
        last_peer = from;
        snprintf(last_name, sizeof last_name, "%s", name);
        last_end = now_us();
    }

    stamp();
    fprintf(stderr, "stopping (signal received)\n");
    if (g_log) fclose(g_log);
    free(root);
    return 0;                                /* atexit removes the PID file */
}
