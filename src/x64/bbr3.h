/*
 * bbr3.h - a from-scratch implementation of BBR version 3 congestion control,
 * following the structure and naming of the IETF draft
 * "BBR Congestion Control" (draft-ietf-ccwg-bbr) and the delivery-rate
 * estimation draft (draft-cheng-iccrg-delivery-rate-estimation).
 *
 * It is transport-agnostic: the transport tells it when packets are sent,
 * acknowledged and declared lost, and asks it for the congestion window
 * and pacing rate. All byte quantities are in bytes, times in microseconds,
 * bandwidths in bytes per second.
 */
#ifndef BBR3_H
#define BBR3_H

#include <stdint.h>

typedef enum {
    BBR_STARTUP,
    BBR_DRAIN,
    BBR_PROBE_BW_DOWN,
    BBR_PROBE_BW_CRUISE,
    BBR_PROBE_BW_REFILL,
    BBR_PROBE_BW_UP,
    BBR_PROBE_RTT
} bbr_state_t;

/* Per-packet snapshot taken at send time (delivery-rate estimation). */
typedef struct {
    uint64_t delivered;        /* C.delivered when sent                  */
    int64_t  delivered_time;   /* C.delivered_time when sent             */
    int64_t  first_sent_time;  /* C.first_sent_time when sent            */
    int64_t  sent_time;
    uint64_t tx_in_flight;     /* bytes in flight including this packet  */
    uint64_t lost;             /* C.lost when sent                       */
    uint32_t size;
    int      is_app_limited;
} bbr_pkt_t;

/* Windowed max filter (Kathleen Nichols' algorithm, as in Linux minmax). */
typedef struct { uint64_t t; double v; } bbr_minmax_sample_t;
typedef struct { bbr_minmax_sample_t s[3]; } bbr_minmax_t;

/* Rate sample produced for each ACK. */
typedef struct {
    int      valid;            /* a delivery-rate sample was produced    */
    double   delivery_rate;    /* bytes/s                                */
    uint64_t delivered;        /* bytes delivered over the interval      */
    uint64_t prior_delivered;
    int64_t  prior_time;
    int64_t  interval;         /* us                                     */
    int64_t  send_elapsed, ack_elapsed;
    int      is_app_limited;
    uint64_t tx_in_flight;
    uint64_t lost;             /* bytes lost over the interval           */
    uint64_t newly_acked;      /* bytes acked by this ACK                */
    uint64_t newly_lost;       /* bytes marked lost by this ACK          */
    int64_t  rtt;              /* us, -1 if no sample                    */
    int64_t  last_end_time;    /* send time of the latest acked packet   */
} bbr_rs_t;

typedef struct {
    /* ---- connection / delivery-rate state ("C" in the drafts) ---- */
    uint32_t mss;
    double   cwnd;             /* bytes                                  */
    double   pacing_rate;      /* bytes/s                                */
    double   send_quantum;     /* bytes                                  */
    uint64_t inflight;         /* bytes in flight                        */
    uint64_t delivered;        /* total bytes delivered                  */
    int64_t  delivered_time;
    int64_t  first_sent_time;
    uint64_t lost;             /* total bytes lost                       */
    uint64_t app_limited;      /* 0, or delivered mark ending app-limited*/
    int      is_cwnd_limited;  /* set by the transport                   */
    int64_t  srtt_hint;        /* transport's smoothed RTT, for init     */

    /* ---- BBR state ---- */
    bbr_state_t state;
    double   pacing_gain, cwnd_gain;
    double   bw;               /* bandwidth used by the model            */
    double   max_bw;           /* long-term windowed max                 */
    bbr_minmax_t max_bw_filter;
    uint64_t cycle_count;
    double   bw_lo, inflight_lo;   /* short-term lower bounds (or INF)   */
    double   inflight_hi;          /* long-term upper bound (or INF)     */
    double   bw_latest, inflight_latest;
    int      loss_round_start;
    uint64_t loss_round_delivered;
    int      loss_in_round;
    uint32_t loss_events_in_round;
    double   max_inflight;
    double   prior_cwnd;

    int64_t  min_rtt, min_rtt_stamp;               /* us                 */
    int64_t  probe_rtt_min_delay, probe_rtt_min_stamp;
    int      probe_rtt_expired;
    int64_t  probe_rtt_done_stamp;
    int      probe_rtt_round_done;
    int      idle_restart;

    uint64_t next_round_delivered;
    uint64_t round_count;
    int      round_start;
    uint64_t rounds_since_bw_probe;

    double   full_bw;
    uint32_t full_bw_count;
    int      full_bw_now;
    int      full_bw_reached;

    int64_t  cycle_stamp;
    int64_t  bw_probe_wait;    /* us                                     */
    int      ack_phase;
    int      bw_probe_samples;
    uint32_t bw_probe_up_rounds;
    double   bw_probe_up_cnt;  /* packets                                */
    double   bw_probe_up_acks; /* bytes                                  */

    int64_t  extra_acked_interval_start;
    double   extra_acked_delivered;
    bbr_minmax_t extra_acked_filter;
    double   extra_acked;

    int      in_recovery;
    int      packet_conservation;
    int64_t  recovery_start_time;

    bbr_rs_t rs;               /* sample being built for the current ACK */
    uint64_t rng;

    /* per-ACK scratch */
    int64_t  now;
    int      rs_has_pkt;
    uint64_t rs_pkt_lost;      /* C.lost snapshot of the rs packet       */
    uint64_t pending_lost;     /* losses detected without an ACK         */
    int      entered_recovery;
} bbr3_t;

void  bbr3_init(bbr3_t *b, uint32_t mss, int64_t now);

/* Call before sending a packet of `size` bytes; fills *p. */
void  bbr3_on_send(bbr3_t *b, bbr_pkt_t *p, uint32_t size, int64_t now);

/* Processing one ACK: begin, then report each newly acked / lost packet,
 * then end with the RTT sample (us, -1 if none). */
void  bbr3_ack_begin(bbr3_t *b);
void  bbr3_on_packet_acked(bbr3_t *b, const bbr_pkt_t *p, int64_t now);
void  bbr3_on_packet_lost(bbr3_t *b, const bbr_pkt_t *p, int64_t now);
void  bbr3_ack_end(bbr3_t *b, int64_t rtt, int64_t now);

/* Packet removed from flight without being acked or lost (e.g. timeout
 * probe superseded); rarely needed. */
void  bbr3_on_packet_discarded(bbr3_t *b, const bbr_pkt_t *p);

/* The sender has no data to send right now. */
void  bbr3_on_app_limited(bbr3_t *b);

const char *bbr3_state_name(const bbr3_t *b);

#endif
