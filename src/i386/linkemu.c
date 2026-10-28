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
 *   -P  packets the emulated link can hold (memory: ~1.5 KB each, x2)
 *   -v  print link statistics every second
 * Prints a summary (drops, queueing delay) on Ctrl-C or SIGTERM.
 * Plain C89 + long long (see compat.h).
 */
#define _GNU_SOURCE
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include "compat.h"
#include "proto.h"

typedef struct { s64 t; u16 len; u8 data[MAX_PKT]; } pkt_t;
typedef struct { pkt_t *p; size_t head, tail, cap; } fifo_t;

static volatile sig_atomic_t g_stop;
static void on_sig(int s) { (void)s; g_stop = 1; }

static double rand01(void) { return (double)random() / ((double)RAND_MAX + 1.0); }

static pkt_t *fifo_push(fifo_t *f)
{
    if (f->tail - f->head >= f->cap)
        return NULL;
    return &f->p[f->tail++ % f->cap];
}

static pkt_t *fifo_peek(fifo_t *f)
{
    return f->head == f->tail ? NULL : &f->p[f->head % f->cap];
}

static void fifo_init(fifo_t *f, size_t cap)
{
    f->p = (pkt_t *)calloc(cap, sizeof(pkt_t));
    if (!f->p) {
        fprintf(stderr, "out of memory (try a smaller -P)\n");
        exit(1);
    }
    f->head = f->tail = 0;
    f->cap = cap;
}

int main(int argc, char **argv)
{
    int lport = 9000, opt, verbose = 0, have_client = 0, cfd, sfd;
    int bufsz = 8 << 20;
    long qcap = 8192;
    char *server = "127.0.0.1:9001", host[256], *colon;
    double rate_mbps = 20, delay_ms = 20, qkb = 100, loss = 0, ack_loss = 0;
    double bytes_per_us, qlimit, link_free_at = 0;
    double qdelay_sum = 0, qdelay_max = 0, sec_qdelay = 0;
    u64 in_pkts = 0, out_pkts = 0, q_drops = 0, rnd_drops = 0, ack_drops = 0;
    u64 sec_bytes = 0, sec_drops = 0, sec_pkts = 0, queued_ok;
    s64 delay_us, start, next_stat;
    struct sockaddr_in la, sa, client;
    fifo_t down, up;                 /* server -> client, client -> server */
    u8 buf[MAX_PKT];

    while ((opt = getopt(argc, argv, "l:s:r:d:q:L:A:P:vh")) != -1) {
        switch (opt) {
        case 'l': lport = atoi(optarg); break;
        case 's': server = optarg; break;
        case 'r': rate_mbps = atof(optarg); break;
        case 'd': delay_ms = atof(optarg); break;
        case 'q': qkb = atof(optarg); break;
        case 'L': loss = atof(optarg); break;
        case 'A': ack_loss = atof(optarg); break;
        case 'P': qcap = atol(optarg); break;
        case 'v': verbose = 1; break;
        default:
            fprintf(stderr, "usage: %s [-l port] [-s host:port] [-r Mbps] [-d one_way_ms] "
                            "[-q queue_KB] [-L data_loss] [-A ack_loss] [-P packets] [-v]\n",
                    argv[0]);
            return 1;
        }
    }
    if (qcap < 16)
        qcap = 16;
    srandom((unsigned)now_us());
    precise_timers();
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    if (strlen(server) >= sizeof host) {
        fprintf(stderr, "-s too long\n");
        return 1;
    }
    strcpy(host, server);
    colon = strrchr(host, ':');
    if (!colon) {
        fprintf(stderr, "-s needs host:port\n");
        return 1;
    }
    *colon = 0;
    memset(&sa, 0, sizeof sa);
    sa.sin_family = AF_INET;
    sa.sin_port = htons((unsigned short)atoi(colon + 1));
    if (resolve_ipv4(host, &sa.sin_addr) < 0) {
        fprintf(stderr, "%s: unknown host\n", host);
        return 1;
    }

    cfd = socket(AF_INET, SOCK_DGRAM, 0);       /* faces the client */
    sfd = socket(AF_INET, SOCK_DGRAM, 0);       /* faces the server */
    setsockopt(cfd, SOL_SOCKET, SO_RCVBUF, (char *)&bufsz, sizeof bufsz);
    setsockopt(sfd, SOL_SOCKET, SO_RCVBUF, (char *)&bufsz, sizeof bufsz);
    setsockopt(cfd, SOL_SOCKET, SO_SNDBUF, (char *)&bufsz, sizeof bufsz);
    memset(&la, 0, sizeof la);
    la.sin_family = AF_INET;
    la.sin_port = htons((unsigned short)lport);
    la.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(cfd, (struct sockaddr *)&la, sizeof la) < 0) {
        perror("bind");
        return 1;
    }
    if (connect(sfd, (struct sockaddr *)&sa, sizeof sa) < 0) {
        perror("connect");
        return 1;
    }
    set_nonblock(cfd);
    set_nonblock(sfd);

    fifo_init(&down, (size_t)qcap);
    fifo_init(&up, (size_t)qcap);
    memset(&client, 0, sizeof client);

    bytes_per_us = rate_mbps * 1e6 / 8 / 1e6;
    qlimit = qkb * 1024;
    delay_us = (s64)(delay_ms * 1000);
    start = now_us();
    next_stat = start + 1000000;

    fprintf(stderr, "linkemu: 127.0.0.1:%d <-> %s  %.1f Mb/s, %.1f ms one-way (RTT %.1f ms), "
                    "queue %.0f KB (%.1f ms), loss %.3f%% data / %.3f%% ack\n",
            lport, server, rate_mbps, delay_ms, 2 * delay_ms, qkb,
            qlimit / bytes_per_us / 1000, loss * 100, ack_loss * 100);

    while (!g_stop) {
        s64 now = now_us(), wake = now + 100000;
        pkt_t *p;

        if ((p = fifo_peek(&down)) != NULL && p->t < wake) wake = p->t;
        if ((p = fifo_peek(&up)) != NULL && p->t < wake) wake = p->t;
        if (verbose && next_stat < wake) wake = next_stat;
        wait_readable(cfd, sfd, wake - now);

        /* client -> server: delay (and optional loss) only */
        for (;;) {
            struct sockaddr_in from;
            sock_len fl = sizeof from;
            pkt_t *q;
            ssize_t r = recvfrom(cfd, (char *)buf, sizeof buf, 0, (struct sockaddr *)&from, &fl);
            if (r <= 0)
                break;
            client = from;
            have_client = 1;
            if (ack_loss > 0 && rand01() < ack_loss) {
                ack_drops++;
                continue;
            }
            q = fifo_push(&up);
            if (!q)
                continue;
            q->t = now_us() + delay_us;
            q->len = (u16)r;
            memcpy(q->data, buf, r);
        }

        /* server -> client: random loss, drop-tail bottleneck queue, delay */
        for (;;) {
            double wire, backlog, depart, waited;
            pkt_t *q;
            ssize_t r = recv(sfd, (char *)buf, sizeof buf, 0);
            if (r <= 0)
                break;
            in_pkts++;
            now = now_us();
            if (loss > 0 && rand01() < loss) {
                rnd_drops++;
                continue;
            }
            wire = (double)(r + WIRE_OVERHEAD);
            backlog = link_free_at > (double)now ? (link_free_at - (double)now) * bytes_per_us : 0;
            if (backlog + wire > qlimit) {
                q_drops++;
                sec_drops++;
                continue;
            }
            depart = (link_free_at > (double)now ? link_free_at : (double)now) + wire / bytes_per_us;
            waited = depart - (double)now - wire / bytes_per_us;   /* queueing delay */
            q = fifo_push(&down);
            if (!q) {
                q_drops++;
                continue;
            }
            link_free_at = depart;
            q->t = (s64)depart + delay_us;
            q->len = (u16)r;
            memcpy(q->data, buf, r);
            qdelay_sum += waited;
            if (waited > qdelay_max)
                qdelay_max = waited;
            sec_qdelay += waited;
            sec_pkts++;
        }

        now = now_us();
        while ((p = fifo_peek(&down)) != NULL && p->t <= now) {
            if (have_client)
                sendto(cfd, (char *)p->data, p->len, 0, (struct sockaddr *)&client, sizeof client);
            out_pkts++;
            sec_bytes += p->len + WIRE_OVERHEAD;
            down.head++;
        }
        while ((p = fifo_peek(&up)) != NULL && p->t <= now) {
            send(sfd, (char *)p->data, p->len, 0);
            up.head++;
        }

        if (verbose && now >= next_stat) {
            fprintf(stderr, "linkemu %6.1fs  out %6.2f Mb/s  avg queue delay %6.2f ms  drops %.0f\n",
                    (double)(now - start) / 1e6, (double)sec_bytes * 8 / 1e6,
                    sec_pkts ? sec_qdelay / (double)sec_pkts / 1000 : 0.0, (double)sec_drops);
            sec_bytes = 0;
            sec_drops = 0;
            sec_qdelay = 0;
            sec_pkts = 0;
            next_stat += 1000000;
        }
    }

    queued_ok = in_pkts - q_drops - rnd_drops;
    fprintf(stderr, "linkemu summary: data pkts in %.0f, delivered %.0f, queue drops %.0f, "
                    "random drops %.0f, ack drops %.0f, avg queue delay %.2f ms, max %.2f ms\n",
            (double)in_pkts, (double)out_pkts, (double)q_drops, (double)rnd_drops,
            (double)ack_drops, queued_ok ? qdelay_sum / (double)queued_ok / 1000 : 0.0,
            qdelay_max / 1000);
    return 0;
}
