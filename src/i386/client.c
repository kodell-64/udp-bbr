/*
 * client.c - fetch a file from udpbbr_server over UDP.
 *
 *   ./udpbbr_client [-p port] host remote_name [local_file]
 *
 * The receiver writes each chunk where it belongs, remembers which packet
 * numbers arrived as a list of ranges, and acknowledges them: after every
 * second packet, immediately when a gap appears, and at least every
 * millisecond while data is arriving. All congestion control lives in the
 * sender; the receiver's only job is timely, accurate ACKs.
 *
 * Plain C89 + long long: builds with gcc 2.7.2.3 on glibc 2.0 / libc5 as
 * well as with current compilers (see compat.h).
 */
#define _GNU_SOURCE
#define _FILE_OFFSET_BITS 64        /* 64-bit file offsets where supported */
#include <stdio.h>
#include <stdlib.h>

#include "compat.h"
#include "proto.h"

#define ACK_EVERY       2
#define ACK_DELAY_US    1000
#define IDLE_ABORT_US   20000000
#define REQ_RETRY_US    300000

typedef struct { u64 hi, lo; } range_t;

static range_t g_ranges[MAX_ACK_RANGES];   /* received pns, highest first */
static int     g_nranges;

static void die(const char *m)
{
    perror(m);
    exit(1);
}

static void usage(const char *argv0)
{
    fprintf(stderr, "usage: %s [-p port] host remote_name [local_file]\n", argv0);
    exit(1);
}

/* Record packet number pn in the descending range list. */
static void add_pn(u64 pn)
{
    int i;
    for (i = 0; i < g_nranges; i++) {
        range_t *r = &g_ranges[i];
        if (pn >= r->lo && pn <= r->hi)
            return;                                   /* duplicate */
        if (pn == r->hi + 1) {                        /* extend upward */
            r->hi = pn;
            if (i > 0 && g_ranges[i - 1].lo == pn + 1) {   /* merge with above */
                g_ranges[i - 1].lo = r->lo;
                memmove(r, r + 1, (g_nranges - i - 1) * sizeof(*r));
                g_nranges--;
            }
            return;
        }
        if (r->lo > 0 && pn == r->lo - 1) {           /* extend downward */
            r->lo = pn;
            if (i + 1 < g_nranges && g_ranges[i + 1].hi + 1 == pn) {
                r->lo = g_ranges[i + 1].lo;
                memmove(&g_ranges[i + 1], &g_ranges[i + 2],
                        (g_nranges - i - 2) * sizeof(*r));
                g_nranges--;
            }
            return;
        }
        if (pn > r->hi)
            break;                                    /* new range goes here */
    }
    if (g_nranges == MAX_ACK_RANGES) {
        if (i == MAX_ACK_RANGES)
            return;          /* older than everything we track: already acked */
        g_nranges--;         /* forget the oldest range */
    }
    memmove(&g_ranges[i + 1], &g_ranges[i], (g_nranges - i) * sizeof(range_t));
    g_ranges[i].hi = g_ranges[i].lo = pn;
    g_nranges++;
}

static void send_ack(int fd, u32 session, s64 largest_rx_time, s64 now)
{
    u8 b[32 + MAX_ACK_RANGES * 16], *p = b;
    int i;

    if (g_nranges == 0)
        return;
    *p++ = PKT_ACK;
    p = put32(p, session);
    p = put64(p, g_ranges[0].hi);
    p = put32(p, (u32)(now - largest_rx_time));
    p = put16(p, (u16)g_nranges);
    for (i = 0; i < g_nranges; i++) {
        p = put64(p, g_ranges[i].hi);
        p = put64(p, g_ranges[i].lo);
    }
    send(fd, (char *)b, p - b, 0);
}

int main(int argc, char **argv)
{
    int port = 9001, opt, fd, out, unacked = 0, i, bufsz = 8 << 20;
    const char *host, *rname, *lname;
    struct sockaddr_in sa;
    u8 req[260], buf[MAX_PKT], done[5];
    u8 *have;
    u32 session = 0;
    u64 size = 0, n_chunks, got = 0, pkts = 0, dups = 0;
    s64 t0, last_req = 0, start, last_rx, largest_rx_time, ack_deadline = 0;
    s64 next_progress, end, now;
    double secs;

    while ((opt = getopt(argc, argv, "p:h")) != -1) {
        if (opt == 'p')
            port = atoi(optarg);
        else
            usage(argv[0]);
    }
    if (argc - optind < 2)
        usage(argv[0]);
    host = argv[optind];
    rname = argv[optind + 1];
    lname = argc - optind > 2 ? argv[optind + 2] : rname;
    if (strlen(rname) > 255) {
        fprintf(stderr, "name too long\n");
        return 1;
    }

    precise_timers();
    memset(&sa, 0, sizeof sa);
    sa.sin_family = AF_INET;
    sa.sin_port = htons((unsigned short)port);
    if (resolve_ipv4(host, &sa.sin_addr) < 0) {
        fprintf(stderr, "%s: unknown host\n", host);
        return 1;
    }
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
        die("socket");
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char *)&bufsz, sizeof bufsz);
    if (connect(fd, (struct sockaddr *)&sa, sizeof sa) < 0)
        die("connect");
    set_nonblock(fd);

    /* ---- request the file until META arrives ---- */
    req[0] = PKT_REQ;
    req[1] = (u8)strlen(rname);
    memcpy(req + 2, rname, req[1]);
    t0 = now_us();
    for (;;) {
        ssize_t r;
        now = now_us();
        if (now - t0 > 10000000) {
            fprintf(stderr, "no reply from server\n");
            return 1;
        }
        if (now - last_req >= REQ_RETRY_US) {
            send(fd, (char *)req, 2 + req[1], 0);
            last_req = now;
        }
        wait_readable(fd, -1, 50000);
        r = recv(fd, (char *)buf, sizeof buf, 0);
        if (r >= 16 && buf[0] == PKT_META) {
            if (buf[1] != 0) {
                fprintf(stderr, "server: '%s' not found or busy\n", rname);
                return 1;
            }
            session = get32(buf + 2);
            size = get64(buf + 6);
            if (get16(buf + 14) != CHUNK_SIZE) {
                fprintf(stderr, "chunk size mismatch\n");
                return 1;
            }
            break;
        }
    }

    if (sizeof(off_t) < 8 && size > 0x7fffffffULL) {
        fprintf(stderr, "file is %.0f bytes; this system is limited to 2 GB files\n",
                (double)size);
        return 1;
    }
    out = open(lname, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (out < 0)
        die(lname);
    if (ftruncate(out, (off_t)size) < 0)
        die("ftruncate");
    n_chunks = (size + CHUNK_SIZE - 1) / CHUNK_SIZE;
    have = (u8 *)calloc((size_t)n_chunks + 1, 1);
    if (!have)
        die("calloc");
    fprintf(stderr, "receiving '%s' (%.0f bytes) -> %s\n", rname, (double)size, lname);

    /* ---- receive loop ---- */
    start = now_us();
    last_rx = largest_rx_time = start;
    next_progress = start + 1000000;

    while (got < n_chunks) {
        s64 wait = 100000;
        now = now_us();
        if (ack_deadline)
            wait = ack_deadline - now;
        wait_readable(fd, -1, wait);

        for (;;) {
            ssize_t r = recv(fd, (char *)buf, sizeof buf, 0);
            u64 pn, chunk;
            u16 len;
            int gap;

            if (r < 0)
                break;                        /* EAGAIN: drained */
            now = now_us();
            if (r < DATA_HDR || buf[0] != PKT_DATA || get32(buf + 1) != session)
                continue;
            pn = get64(buf + 5);
            chunk = get64(buf + 13);
            len = get16(buf + 21);
            if (chunk >= n_chunks || DATA_HDR + len > r)
                continue;
            pkts++;
            last_rx = now;

            gap = g_nranges > 0 && pn != g_ranges[0].hi + 1;
            if (g_nranges == 0 || pn > g_ranges[0].hi)
                largest_rx_time = now;
            add_pn(pn);

            if (!have[chunk]) {
                if (write_at(out, buf + DATA_HDR, len, chunk * CHUNK_SIZE) < 0)
                    die("write");
                have[chunk] = 1;
                got++;
            } else {
                dups++;
            }

            unacked++;
            if (unacked >= ACK_EVERY || gap || got == n_chunks) {
                send_ack(fd, session, largest_rx_time, now);
                unacked = 0;
                ack_deadline = 0;
            } else if (!ack_deadline) {
                ack_deadline = now + ACK_DELAY_US;
            }
        }

        now = now_us();
        if (ack_deadline && now >= ack_deadline) {
            send_ack(fd, session, largest_rx_time, now);
            unacked = 0;
            ack_deadline = 0;
        }
        if (now >= next_progress) {
            fprintf(stderr, "\r%5.1f%%  %.2f Mb/s   ",
                    100.0 * (double)got / (double)(n_chunks ? n_chunks : 1),
                    (double)got * CHUNK_SIZE * 8 / ((double)(now - start) / 1e6) / 1e6);
            next_progress += 1000000;
        }
        if (now - last_rx > IDLE_ABORT_US) {
            fprintf(stderr, "\nserver went silent, giving up\n");
            return 1;
        }
    }

    /* ---- finished: tell the server (a few times, DONE may be lost) ---- */
    end = now_us();
    done[0] = PKT_DONE;
    put32(done + 1, session);
    for (i = 0; i < 3; i++) {
        send_ack(fd, session, largest_rx_time, now_us());
        send(fd, (char *)done, sizeof done, 0);
        usleep(10000);
    }
    close(out);
    secs = (double)(end - start) / 1e6;
    fprintf(stderr, "\rreceived %.0f bytes in %.2f s = %.2f Mb/s goodput "
            "(%.0f packets, %.0f duplicates)\n",
            (double)size, secs, secs > 0 ? (double)size * 8 / secs / 1e6 : 0.0,
            (double)pkts, (double)dups);
    return 0;
}
