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

#include "compat.h"

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
    u64 delivered;        /* C.delivered when sent                  */
    s64  delivered_time;   /* C.delivered_time when sent             */
    s64  first_sent_time;  /* C.first_sent_time when sent            */
    s64  sent_time;
    u64 tx_in_flight;     /* bytes in flight including this packet  */
    u64 lost;             /* C.lost when sent                       */
    u32 size;
    int      is_app_limited;
} bbr_pkt_t;

/* Windowed max filter (Kathleen Nichols' algorithm, as in Linux minmax). */
typedef struct { u64 t; double v; } bbr_minmax_sample_t;
typedef struct { bbr_minmax_sample_t s[3]; } bbr_minmax_t;

/* Rate sample produced for each ACK. */
typedef struct {
    int      valid;            /* a delivery-rate sample was produced    */
    double   delivery_rate;    /* bytes/s                                */
    u64 delivered;        /* bytes delivered over the interval      */
    u64 prior_delivered;
    s64  prior_time;
    s64  interval;         /* us                                     */
    s64  send_elapsed, ack_elapsed;
    int      is_app_limited;
    u64 tx_in_flight;
    u64 lost;             /* bytes lost over the interval           */
    u64 newly_acked;      /* bytes acked by this ACK                */
    u64 newly_lost;       /* bytes marked lost by this ACK          */
    s64  rtt;              /* us, -1 if no sample                    */
    s64  last_end_time;    /* send time of the latest acked packet   */
} bbr_rs_t;

typedef struct {
    /* ---- connection / delivery-rate state ("C" in the drafts) ---- */
    u32 mss;
    double   cwnd;             /* bytes                                  */
    double   pacing_rate;      /* bytes/s                                */
    double   send_quantum;     /* bytes                                  */
    u64 inflight;         /* bytes in flight                        */
    u64 delivered;        /* total bytes delivered                  */
    s64  delivered_time;
    s64  first_sent_time;
    u64 lost;             /* total bytes lost                       */
    u64 app_limited;      /* 0, or delivered mark ending app-limited*/
    int      is_cwnd_limited;  /* set by the transport                   */
    s64  srtt_hint;        /* transport's smoothed RTT, for init     */

    /* ---- BBR state ---- */
    bbr_state_t state;
    double   pacing_gain, cwnd_gain;
    double   bw;               /* bandwidth used by the model            */
    double   max_bw;           /* long-term windowed max                 */
    bbr_minmax_t max_bw_filter;
    u64 cycle_count;
    double   bw_lo, inflight_lo;   /* short-term lower bounds (or INF)   */
    double   inflight_hi;          /* long-term upper bound (or INF)     */
    double   bw_latest, inflight_latest;
    int      loss_round_start;
    u64 loss_round_delivered;
    int      loss_in_round;
    u32 loss_events_in_round;
    double   max_inflight;
    double   prior_cwnd;

    s64  min_rtt, min_rtt_stamp;               /* us                 */
    s64  probe_rtt_min_delay, probe_rtt_min_stamp;
    int      probe_rtt_expired;
    s64  probe_rtt_done_stamp;
    int      probe_rtt_round_done;
    int      idle_restart;

    u64 next_round_delivered;
    u64 round_count;
    int      round_start;
    u64 rounds_since_bw_probe;

    double   full_bw;
    u32 full_bw_count;
    int      full_bw_now;
    int      full_bw_reached;

    s64  cycle_stamp;
    s64  bw_probe_wait;    /* us                                     */
    int      ack_phase;
    int      bw_probe_samples;
    u32 bw_probe_up_rounds;
    double   bw_probe_up_cnt;  /* packets                                */
    double   bw_probe_up_acks; /* bytes                                  */

    s64  extra_acked_interval_start;
    double   extra_acked_delivered;
    bbr_minmax_t extra_acked_filter;
    double   extra_acked;

    int      in_recovery;
    int      packet_conservation;
    s64  recovery_start_time;

    bbr_rs_t rs;               /* sample being built for the current ACK */
    u64 rng;

    /* per-ACK scratch */
    s64  now;
    int      rs_has_pkt;
    u64 rs_pkt_lost;      /* C.lost snapshot of the rs packet       */
    u64 pending_lost;     /* losses detected without an ACK         */
    int      entered_recovery;
} bbr3_t;

void  bbr3_init(bbr3_t *b, u32 mss, s64 now);

/* Call before sending a packet of `size` bytes; fills *p. */
void  bbr3_on_send(bbr3_t *b, bbr_pkt_t *p, u32 size, s64 now);

/* Processing one ACK: begin, then report each newly acked / lost packet,
 * then end with the RTT sample (us, -1 if none). */
void  bbr3_ack_begin(bbr3_t *b);
void  bbr3_on_packet_acked(bbr3_t *b, const bbr_pkt_t *p, s64 now);
void  bbr3_on_packet_lost(bbr3_t *b, const bbr_pkt_t *p, s64 now);
void  bbr3_ack_end(bbr3_t *b, s64 rtt, s64 now);

/* Packet removed from flight without being acked or lost (e.g. timeout
 * probe superseded); rarely needed. */
void  bbr3_on_packet_discarded(bbr3_t *b, const bbr_pkt_t *p);

/* The sender has no data to send right now. */
void  bbr3_on_app_limited(bbr3_t *b);

const char *bbr3_state_name(const bbr3_t *b);

#endif
