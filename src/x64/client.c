/*
 * client.c - fetch a file from udpbbr_server over UDP.
 *
 *   ./udpbbr_client [-p port] host remote_name [local_file]
 *
 * The receiver writes each chunk where it belongs (pwrite), remembers
 * which packet numbers arrived as a list of ranges, and acknowledges them:
 * after every second packet, immediately when a gap appears, and at least
 * every millisecond while data is arriving. All congestion control lives
 * in the sender; the receiver's only job is timely, accurate ACKs.
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <netdb.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "proto.h"

#define ACK_EVERY       2
#define ACK_DELAY_US    1000
#define IDLE_ABORT_US   20000000
#define REQ_RETRY_US    300000

typedef struct { uint64_t hi, lo; } range_t;

static range_t  g_ranges[MAX_ACK_RANGES];   /* received pns, highest first */
static int      g_nranges;

static void die(const char *m) { perror(m); exit(1); }

/* Record packet number pn in the descending range list. */
static void add_pn(uint64_t pn)
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

static void send_ack(int fd, uint32_t session, int64_t largest_rx_time, int64_t now)
{
    if (g_nranges == 0)
        return;
    uint8_t b[32 + MAX_ACK_RANGES * 16], *p = b;
    *p++ = PKT_ACK;
    p = put32(p, session);
    p = put64(p, g_ranges[0].hi);
    p = put32(p, (uint32_t)(now - largest_rx_time));
    p = put16(p, (uint16_t)g_nranges);
    for (int i = 0; i < g_nranges; i++) {
        p = put64(p, g_ranges[i].hi);
        p = put64(p, g_ranges[i].lo);
    }
    send(fd, b, p - b, 0);
}

int main(int argc, char **argv)
{
    const char *port = "9001";
    int opt;
    while ((opt = getopt(argc, argv, "p:h")) != -1) {
        if (opt == 'p') port = optarg;
        else goto usage;
    }
    if (argc - optind < 2) {
usage:
        fprintf(stderr, "usage: %s [-p port] host remote_name [local_file]\n", argv[0]);
        return 1;
    }
    const char *host = argv[optind], *rname = argv[optind + 1];
    const char *lname = argc - optind > 2 ? argv[optind + 2] : rname;
    if (strlen(rname) > 255) { fprintf(stderr, "name too long\n"); return 1; }

    prctl(PR_SET_TIMERSLACK, 1UL);
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_DGRAM }, *ai;
    int e = getaddrinfo(host, port, &hints, &ai);
    if (e) { fprintf(stderr, "%s: %s\n", host, gai_strerror(e)); return 1; }
    int fd = socket(ai->ai_family, SOCK_DGRAM, 0);
    if (fd < 0) die("socket");
    int bufsz = 8 << 20;
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &bufsz, sizeof bufsz);
    if (connect(fd, ai->ai_addr, ai->ai_addrlen) < 0) die("connect");
    freeaddrinfo(ai);

    /* ---- request the file until META arrives ---- */
    uint8_t req[260], buf[MAX_PKT];
    req[0] = PKT_REQ;
    req[1] = (uint8_t)strlen(rname);
    memcpy(req + 2, rname, req[1]);
    uint32_t session = 0;
    uint64_t size = 0;
    int64_t t0 = now_us(), last_req = 0;
    for (;;) {
        int64_t now = now_us();
        if (now - t0 > 10000000) { fprintf(stderr, "no reply from server\n"); return 1; }
        if (now - last_req >= REQ_RETRY_US) {
            send(fd, req, 2 + req[1], 0);
            last_req = now;
        }
        struct pollfd pfd = { fd, POLLIN, 0 };
        if (poll(&pfd, 1, 50) <= 0) continue;
        ssize_t r = recv(fd, buf, sizeof buf, 0);
        if (r >= 16 && buf[0] == PKT_META) {
            if (buf[1] != 0) { fprintf(stderr, "server: '%s' not found or busy\n", rname); return 1; }
            session = get32(buf + 2);
            size = get64(buf + 6);
            if (get16(buf + 14) != CHUNK_SIZE) { fprintf(stderr, "chunk size mismatch\n"); return 1; }
            break;
        }
    }

    int out = open(lname, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (out < 0) die(lname);
    if (ftruncate(out, (off_t)size) < 0) die("ftruncate");
    uint64_t n_chunks = (size + CHUNK_SIZE - 1) / CHUNK_SIZE, got = 0;
    uint8_t *have = calloc(n_chunks + 1, 1);
    fprintf(stderr, "receiving '%s' (%llu bytes) -> %s\n", rname, (unsigned long long)size, lname);

    /* ---- receive loop ---- */
    int64_t start = now_us(), last_rx = start, largest_rx_time = start, ack_deadline = 0;
    uint64_t pkts = 0, dups = 0;
    int unacked = 0;
    int64_t next_progress = start + 1000000;

    while (got < n_chunks) {
        int64_t now = now_us();
        int64_t wait = 100000;
        if (ack_deadline) wait = ack_deadline - now;
        if (wait < 0) wait = 0;
        struct pollfd pfd = { fd, POLLIN, 0 };
        struct timespec ts = { wait / 1000000, (wait % 1000000) * 1000 };
        ppoll(&pfd, 1, &ts, NULL);

        for (;;) {
            ssize_t r = recv(fd, buf, sizeof buf, MSG_DONTWAIT);
            if (r < 0) break;
            now = now_us();
            if (r < DATA_HDR || buf[0] != PKT_DATA || get32(buf + 1) != session)
                continue;
            uint64_t pn = get64(buf + 5), chunk = get64(buf + 13);
            uint16_t len = get16(buf + 21);
            if (chunk >= n_chunks || DATA_HDR + len > r)
                continue;
            pkts++;
            last_rx = now;

            int gap = g_nranges > 0 && pn != g_ranges[0].hi + 1;
            if (g_nranges == 0 || pn > g_ranges[0].hi)
                largest_rx_time = now;
            add_pn(pn);

            if (!have[chunk]) {
                if (pwrite(out, buf + DATA_HDR, len, (off_t)(chunk * CHUNK_SIZE)) != len)
                    die("pwrite");
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
            fprintf(stderr, "\r%5.1f%%  %.2f Mb/s   ", 100.0 * got / (n_chunks ? n_chunks : 1),
                    (double)got * CHUNK_SIZE * 8 / ((now - start) / 1e6) / 1e6);
            next_progress += 1000000;
        }
        if (now - last_rx > IDLE_ABORT_US) {
            fprintf(stderr, "\nserver went silent, giving up\n");
            return 1;
        }
    }

    /* ---- finished: tell the server (a few times, DONE may be lost) ---- */
    int64_t end = now_us();
    uint8_t done[5] = { PKT_DONE };
    put32(done + 1, session);
    for (int i = 0; i < 3; i++) {
        send_ack(fd, session, largest_rx_time, now_us());
        send(fd, done, sizeof done, 0);
        usleep(10000);
    }
    close(out);
    double secs = (end - start) / 1e6;
    fprintf(stderr, "\rreceived %llu bytes in %.2f s = %.2f Mb/s goodput "
            "(%llu packets, %llu duplicates)\n",
            (unsigned long long)size, secs, secs > 0 ? size * 8 / secs / 1e6 : 0.0,
            (unsigned long long)pkts, (unsigned long long)dups);
    return 0;
}
