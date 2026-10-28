/*
 * linkemu.c - a tiny UDP link emulator for testing congestion control
 * without root (no tc/netem needed).
 *
 *   ./linkemu -l 9000 -s 127.0.0.1:9001 -r 20 -d 20 -q 100 -L 0.01
 *
 * The client talks to the emulator's port; the emulator relays to the
 * server. Server->client traffic (the data) goes through a bottleneck:
 *   -r  rate in Mbit/s (counts 28 bytes of IP+UDP header per packet)
 *   -q  drop-tail queue limit in KB
 *   -d  one-way propagation delay in ms (applied in both directions)
 *   -L  random loss probability on the data direction
 *   -A  random loss probability on the ACK direction
 *   -v  print link statistics every second
 * Prints a summary (drops, queueing delay) on Ctrl-C or SIGTERM.
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <getopt.h>
#include <netdb.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "proto.h"

#define QCAP 32768        /* packets that can be in the emulated link at once */

typedef struct { int64_t t; uint16_t len; uint8_t data[MAX_PKT]; } pkt_t;
typedef struct { pkt_t *p; size_t head, tail; } fifo_t;

static volatile sig_atomic_t g_stop;
static void on_sig(int s) { (void)s; g_stop = 1; }

static double rand01(void) { return (double)random() / ((double)RAND_MAX + 1.0); }

static pkt_t *fifo_push(fifo_t *f)
{
    if (f->tail - f->head >= QCAP) return NULL;
    return &f->p[f->tail++ % QCAP];
}
static pkt_t *fifo_peek(fifo_t *f) { return f->head == f->tail ? NULL : &f->p[f->head % QCAP]; }

int main(int argc, char **argv)
{
    int lport = 9000, opt, verbose = 0;
    char *server = "127.0.0.1:9001";
    double rate_mbps = 20, delay_ms = 20, qkb = 100, loss = 0, ack_loss = 0;
    while ((opt = getopt(argc, argv, "l:s:r:d:q:L:A:vh")) != -1) {
        switch (opt) {
        case 'l': lport = atoi(optarg); break;
        case 's': server = optarg; break;
        case 'r': rate_mbps = atof(optarg); break;
        case 'd': delay_ms = atof(optarg); break;
        case 'q': qkb = atof(optarg); break;
        case 'L': loss = atof(optarg); break;
        case 'A': ack_loss = atof(optarg); break;
        case 'v': verbose = 1; break;
        default:
            fprintf(stderr, "usage: %s [-l port] [-s host:port] [-r Mbps] [-d one_way_ms] "
                            "[-q queue_KB] [-L data_loss] [-A ack_loss] [-v]\n", argv[0]);
            return 1;
        }
    }
    srandom((unsigned)now_us());
    prctl(PR_SET_TIMERSLACK, 1UL);
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    char host[256];
    snprintf(host, sizeof host, "%s", server);
    char *colon = strrchr(host, ':');
    if (!colon) { fprintf(stderr, "-s needs host:port\n"); return 1; }
    *colon = 0;
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_DGRAM }, *ai;
    if (getaddrinfo(host, colon + 1, &hints, &ai)) { fprintf(stderr, "bad server\n"); return 1; }

    int cfd = socket(AF_INET, SOCK_DGRAM, 0);       /* faces the client */
    int sfd = socket(AF_INET, SOCK_DGRAM, 0);       /* faces the server */
    int bufsz = 8 << 20;
    setsockopt(cfd, SOL_SOCKET, SO_RCVBUF, &bufsz, sizeof bufsz);
    setsockopt(sfd, SOL_SOCKET, SO_RCVBUF, &bufsz, sizeof bufsz);
    setsockopt(cfd, SOL_SOCKET, SO_SNDBUF, &bufsz, sizeof bufsz);
    struct sockaddr_in la = { .sin_family = AF_INET, .sin_port = htons(lport),
                              .sin_addr.s_addr = htonl(INADDR_LOOPBACK) };
    if (bind(cfd, (struct sockaddr *)&la, sizeof la) < 0) { perror("bind"); return 1; }
    if (connect(sfd, ai->ai_addr, ai->ai_addrlen) < 0) { perror("connect"); return 1; }
    freeaddrinfo(ai);

    fifo_t down = { calloc(QCAP, sizeof(pkt_t)), 0, 0 };   /* server -> client */
    fifo_t up   = { calloc(QCAP, sizeof(pkt_t)), 0, 0 };   /* client -> server */
    struct sockaddr_in client = {0};
    int have_client = 0;

    double bytes_per_us = rate_mbps * 1e6 / 8 / 1e6;
    double qlimit = qkb * 1024;
    int64_t delay_us = (int64_t)(delay_ms * 1000);
    double link_free_at = 0;                  /* when the bottleneck empties */

    uint64_t in_pkts = 0, out_pkts = 0, q_drops = 0, rnd_drops = 0, ack_drops = 0;
    double qdelay_sum = 0, qdelay_max = 0;
    uint64_t sec_bytes = 0, sec_drops = 0;
    double sec_qdelay = 0; uint64_t sec_pkts = 0;
    int64_t start = now_us(), next_stat = start + 1000000;

    fprintf(stderr, "linkemu: 127.0.0.1:%d <-> %s  %.1f Mb/s, %.1f ms one-way (RTT %.1f ms), "
                    "queue %.0f KB (%.1f ms), loss %.3f%% data / %.3f%% ack\n",
            lport, server, rate_mbps, delay_ms, 2 * delay_ms, qkb,
            qlimit / bytes_per_us / 1000, loss * 100, ack_loss * 100);

    uint8_t buf[MAX_PKT];
    while (!g_stop) {
        int64_t now = now_us();
        int64_t wake = now + 100000;
        pkt_t *p;
        if ((p = fifo_peek(&down)) && p->t < wake) wake = p->t;
        if ((p = fifo_peek(&up)) && p->t < wake) wake = p->t;
        if (verbose && next_stat < wake) wake = next_stat;
        int64_t wait = wake - now;
        if (wait < 0) wait = 0;
        struct pollfd pfd[2] = { { cfd, POLLIN, 0 }, { sfd, POLLIN, 0 } };
        struct timespec ts = { wait / 1000000, (wait % 1000000) * 1000 };
        ppoll(pfd, 2, &ts, NULL);

        /* client -> server: delay (and optional loss) only */
        for (;;) {
            struct sockaddr_in from;
            socklen_t fl = sizeof from;
            ssize_t r = recvfrom(cfd, buf, sizeof buf, MSG_DONTWAIT, (struct sockaddr *)&from, &fl);
            if (r <= 0) break;
            client = from;
            have_client = 1;
            if (ack_loss > 0 && rand01() < ack_loss) { ack_drops++; continue; }
            pkt_t *q = fifo_push(&up);
            if (!q) continue;
            q->t = now_us() + delay_us;
            q->len = (uint16_t)r;
            memcpy(q->data, buf, r);
        }

        /* server -> client: random loss, drop-tail bottleneck queue, delay */
        for (;;) {
            ssize_t r = recv(sfd, buf, sizeof buf, MSG_DONTWAIT);
            if (r <= 0) break;
            in_pkts++;
            now = now_us();
            if (loss > 0 && rand01() < loss) { rnd_drops++; continue; }
            double wire = r + WIRE_OVERHEAD;
            double backlog = link_free_at > now ? (link_free_at - now) * bytes_per_us : 0;
            if (backlog + wire > qlimit) { q_drops++; sec_drops++; continue; }
            double depart = (link_free_at > now ? link_free_at : now) + wire / bytes_per_us;
            double waited = depart - now - wire / bytes_per_us;   /* queueing delay */
            link_free_at = depart;
            pkt_t *q = fifo_push(&down);
            if (!q) { q_drops++; continue; }
            q->t = (int64_t)depart + delay_us;
            q->len = (uint16_t)r;
            memcpy(q->data, buf, r);
            qdelay_sum += waited;
            if (waited > qdelay_max) qdelay_max = waited;
            sec_qdelay += waited; sec_pkts++;
        }

        now = now_us();
        while ((p = fifo_peek(&down)) && p->t <= now) {
            if (have_client)
                sendto(cfd, p->data, p->len, 0, (struct sockaddr *)&client, sizeof client);
            out_pkts++;
            sec_bytes += p->len + WIRE_OVERHEAD;
            down.head++;
        }
        while ((p = fifo_peek(&up)) && p->t <= now) {
            send(sfd, p->data, p->len, 0);
            up.head++;
        }

        if (verbose && now >= next_stat) {
            fprintf(stderr, "linkemu %6.1fs  out %6.2f Mb/s  avg queue delay %6.2f ms  drops %llu\n",
                    (now - start) / 1e6, sec_bytes * 8 / 1e6,
                    sec_pkts ? sec_qdelay / sec_pkts / 1000 : 0.0, (unsigned long long)sec_drops);
            sec_bytes = 0; sec_drops = 0; sec_qdelay = 0; sec_pkts = 0;
            next_stat += 1000000;
        }
    }

    fprintf(stderr, "linkemu summary: data pkts in %llu, delivered %llu, queue drops %llu, "
                    "random drops %llu, ack drops %llu, avg queue delay %.2f ms, max %.2f ms\n",
            (unsigned long long)in_pkts, (unsigned long long)out_pkts,
            (unsigned long long)q_drops, (unsigned long long)rnd_drops,
            (unsigned long long)ack_drops,
            (in_pkts - q_drops - rnd_drops) ? qdelay_sum / (in_pkts - q_drops - rnd_drops) / 1000 : 0.0,
            qdelay_max / 1000);
    return 0;
}
