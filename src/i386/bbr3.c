/*
 * bbr3.c - BBR version 3 congestion control, written from the algorithm
 * description in the IETF draft "BBR Congestion Control"
 * (draft-ietf-ccwg-bbr) and "Delivery Rate Estimation"
 * (draft-cheng-iccrg-delivery-rate-estimation).
 *
 * Function names mirror the draft's pseudocode (BBRUpdateMaxBw ->
 * update_max_bw, etc.) so the two can be read side by side.
 *
 * What makes this "v3" rather than v1:
 *   - a two-part model: long-term max_bw / inflight_hi and short-term
 *     bw_lo / inflight_lo bounds that react to loss (BBRBeta = 0.7) and
 *     a 2% loss-rate threshold (BBRLossThresh) for "inflight too high";
 *   - the ProbeBW cycle DOWN -> CRUISE -> REFILL -> UP with wall-clock and
 *     Reno-coexistence probe timing, and exponential inflight_hi probing;
 *   - headroom (15%) left below inflight_hi while cruising;
 *   - Startup can also exit on persistent high loss;
 *   - v3 gains: Startup 2.77, Drain 0.35, ProbeBW_DOWN 0.90, UP cwnd 2.25.
 */
#include <math.h>
#include <string.h>
#include "bbr3.h"

/* ------------------------------------------------------------------ */
/* Parameters (names from the draft)                                    */
/* ------------------------------------------------------------------ */
#define BBRStartupPacingGain     2.77    /* 4*ln(2) */
#define BBRStartupCwndGain       2.0
#define BBRDrainPacingGain       0.35
#define BBRDefaultCwndGain       2.0
#define BBRProbeBWDownGain       0.90
#define BBRProbeBWUpGain         1.25
#define BBRProbeBWUpCwndGain     2.25
#define BBRPacingMarginPercent   1.0
#define BBRLossThresh            0.02
#define BBRBeta                  0.7
#define BBRHeadroom              0.15
#define BBRMinPipeCwndPkts       4
#define BBRMaxBwFilterLen        2       /* ProbeBW cycles */
#define BBRExtraAckedFilterLen   10      /* round trips    */
#define BBRStartupFullLossCnt    6
#define BBRStartupFullBwRounds   3
#define BBRFullBwThresh          1.25
#define MinRTTFilterLen          10000000LL  /* 10 s  */
#define ProbeRTTCwndGain         0.5
#define ProbeRTTInterval         5000000LL   /* 5 s   */
#define ProbeRTTDuration         200000LL    /* 200 ms */
#define InitialCwndPkts          10
#define BBRMaxRenoRounds         63

#define INF        HUGE_VAL       /* C89's +infinity */
#define RTT_INF    S64_MAX

enum { ACKS_INIT, ACKS_REFILLING, ACKS_PROBE_STARTING,
       ACKS_PROBE_FEEDBACK, ACKS_PROBE_STOPPING };

static double dmax(double a, double b) { return a > b ? a : b; }
static double dmin(double a, double b) { return a < b ? a : b; }

/* ------------------------------------------------------------------ */
/* Windowed max filter (port of Linux lib/minmax.c)                     */
/* ------------------------------------------------------------------ */
static double minmax_reset(bbr_minmax_t *m, u64 t, double v)
{
    m->s[0].t = m->s[1].t = m->s[2].t = t;
    m->s[0].v = m->s[1].v = m->s[2].v = v;
    return v;
}

static double minmax_subwin_update(bbr_minmax_t *m, u64 win,
                                   bbr_minmax_sample_t val)
{
    u64 dt = val.t - m->s[0].t;
    if (dt > win) {
        m->s[0] = m->s[1];
        m->s[1] = m->s[2];
        m->s[2] = val;
        if (val.t - m->s[0].t > win) {
            m->s[0] = m->s[1];
            m->s[1] = m->s[2];
            m->s[2] = val;
        }
    } else if (m->s[1].t == m->s[0].t && dt > win / 4) {
        m->s[2] = m->s[1] = val;
    } else if (m->s[2].t == m->s[1].t && dt > win / 2) {
        m->s[2] = val;
    }
    return m->s[0].v;
}

static double minmax_running_max(bbr_minmax_t *m, u64 win,
                                 u64 t, double v)
{
    bbr_minmax_sample_t val;
    val.t = t;
    val.v = v;
    if (val.v >= m->s[0].v || val.t - m->s[2].t > win)
        return minmax_reset(m, t, v);
    if (val.v >= m->s[1].v)
        m->s[2] = m->s[1] = val;
    else if (val.v >= m->s[2].v)
        m->s[2] = val;
    return minmax_subwin_update(m, win, val);
}

/* ------------------------------------------------------------------ */
static u64 rng_next(bbr3_t *b)
{
    b->rng ^= b->rng << 13; b->rng ^= b->rng >> 7; b->rng ^= b->rng << 17;
    return b->rng;
}
static double rng_unit(bbr3_t *b) { return (rng_next(b) >> 11) * (1.0 / 9007199254740992.0); }

static double min_pipe_cwnd(bbr3_t *b) { return BBRMinPipeCwndPkts * (double)b->mss; }
static double initial_cwnd(bbr3_t *b)  { return InitialCwndPkts * (double)b->mss; }

/* ------------------------------------------------------------------ */
/* Model helpers                                                         */
/* ------------------------------------------------------------------ */
static double bdp_multiple(bbr3_t *b, double bw, double gain)
{
    if (b->min_rtt == RTT_INF)
        return initial_cwnd(b);           /* no RTT sample yet */
    return gain * bw * (double)b->min_rtt / 1e6;
}

static double quantization_budget(bbr3_t *b, double inflight)
{
    double offload_budget = 3 * b->send_quantum;
    inflight = dmax(inflight, offload_budget);
    inflight = dmax(inflight, min_pipe_cwnd(b));
    if (b->state == BBR_PROBE_BW_UP)
        inflight += 2 * (double)b->mss;
    return inflight;
}

static double bbr_inflight(bbr3_t *b, double bw, double gain)
{
    return quantization_budget(b, bdp_multiple(b, bw, gain));
}

static double inflight_with_headroom(bbr3_t *b)
{
    double headroom;
    if (b->inflight_hi == INF)
        return INF;
    headroom = dmax(b->mss, BBRHeadroom * b->inflight_hi);
    return dmax(b->inflight_hi - headroom, min_pipe_cwnd(b));
}

static double target_inflight(bbr3_t *b)
{
    return dmin(bdp_multiple(b, b->bw, 1.0), b->cwnd);
}

static int is_in_probe_bw_state(bbr3_t *b)
{
    return b->state == BBR_PROBE_BW_DOWN || b->state == BBR_PROBE_BW_CRUISE ||
           b->state == BBR_PROBE_BW_REFILL || b->state == BBR_PROBE_BW_UP;
}

static int is_probing_bw(bbr3_t *b)
{
    return b->state == BBR_STARTUP || b->state == BBR_PROBE_BW_REFILL ||
           b->state == BBR_PROBE_BW_UP;
}

/* ------------------------------------------------------------------ */
/* Pacing rate and send quantum                                          */
/* ------------------------------------------------------------------ */
static void set_pacing_rate_with_gain(bbr3_t *b, double gain)
{
    double rate = gain * b->bw * (100.0 - BBRPacingMarginPercent) / 100.0;
    if (b->full_bw_reached || rate > b->pacing_rate)
        b->pacing_rate = rate;
}

static void init_pacing_rate(bbr3_t *b)
{
    double rtt = b->srtt_hint > 0 ? (double)b->srtt_hint : 1000.0;  /* 1 ms */
    double nominal_bw = initial_cwnd(b) / (rtt / 1e6);
    b->pacing_rate = BBRStartupPacingGain * nominal_bw;
}

static void set_send_quantum(bbr3_t *b)
{
    double floor_q = b->pacing_rate < 1.2e6 / 8 ? b->mss : 2.0 * b->mss;
    double q = dmin(b->pacing_rate * 0.001, 64 * 1024);
    b->send_quantum = dmax(q, floor_q);
}

/* ------------------------------------------------------------------ */
/* Round counting                                                        */
/* ------------------------------------------------------------------ */
static void start_round(bbr3_t *b) { b->next_round_delivered = b->delivered; }

static void update_round(bbr3_t *b)
{
    if (b->rs_has_pkt && b->rs.prior_delivered >= b->next_round_delivered) {
        start_round(b);
        b->round_count++;
        b->rounds_since_bw_probe++;
        b->round_start = 1;
    } else {
        b->round_start = 0;
    }
}

/* ------------------------------------------------------------------ */
/* State entry                                                           */
/* ------------------------------------------------------------------ */
static void enter_startup(bbr3_t *b)
{
    b->state = BBR_STARTUP;
    b->pacing_gain = BBRStartupPacingGain;
    b->cwnd_gain = BBRStartupCwndGain;
}

static void enter_drain(bbr3_t *b)
{
    b->state = BBR_DRAIN;
    b->pacing_gain = BBRDrainPacingGain;
    b->cwnd_gain = BBRStartupCwndGain;
}

static void reset_congestion_signals(bbr3_t *b)
{
    b->loss_in_round = 0;
    b->bw_latest = 0;
    b->inflight_latest = 0;
}

static void reset_lower_bounds(bbr3_t *b)
{
    b->bw_lo = INF;
    b->inflight_lo = INF;
}

static void reset_full_bw(bbr3_t *b)
{
    b->full_bw = 0;
    b->full_bw_count = 0;
    b->full_bw_now = 0;
}

static void pick_probe_wait(bbr3_t *b)
{
    /* Randomised to avoid synchronising flows: 0 or 1 extra round, and
     * a 2..3 second wall-clock wait. */
    b->rounds_since_bw_probe = rng_next(b) & 1;
    b->bw_probe_wait = 2000000 + (s64)(rng_unit(b) * 1000000.0);
}

static void raise_inflight_hi_slope(bbr3_t *b)
{
    double growth_this_round = (double)(1u << b->bw_probe_up_rounds);   /* packets */
    if (b->bw_probe_up_rounds < 30)
        b->bw_probe_up_rounds++;
    b->bw_probe_up_cnt = dmax(b->cwnd / b->mss / growth_this_round, 1.0);
}

static void start_probe_bw_down(bbr3_t *b)
{
    reset_congestion_signals(b);
    b->bw_probe_up_cnt = INF;
    pick_probe_wait(b);
    b->cycle_stamp = b->now;
    b->ack_phase = ACKS_PROBE_STOPPING;
    start_round(b);
    b->state = BBR_PROBE_BW_DOWN;
    b->pacing_gain = BBRProbeBWDownGain;
    b->cwnd_gain = BBRDefaultCwndGain;
}

static void start_probe_bw_cruise(bbr3_t *b)
{
    b->state = BBR_PROBE_BW_CRUISE;
    b->pacing_gain = 1.0;
    b->cwnd_gain = BBRDefaultCwndGain;
}

static void start_probe_bw_refill(bbr3_t *b)
{
    reset_lower_bounds(b);
    b->bw_probe_up_rounds = 0;
    b->bw_probe_up_acks = 0;
    b->ack_phase = ACKS_REFILLING;
    start_round(b);
    b->state = BBR_PROBE_BW_REFILL;
    b->pacing_gain = 1.0;
    b->cwnd_gain = BBRDefaultCwndGain;
}

static void start_probe_bw_up(bbr3_t *b)
{
    b->ack_phase = ACKS_PROBE_STARTING;
    start_round(b);
    reset_full_bw(b);
    b->full_bw = b->rs.valid ? b->rs.delivery_rate : 0;
    b->state = BBR_PROBE_BW_UP;
    b->pacing_gain = BBRProbeBWUpGain;
    b->cwnd_gain = BBRProbeBWUpCwndGain;
    raise_inflight_hi_slope(b);
}

static void enter_probe_bw(bbr3_t *b)
{
    b->cwnd_gain = BBRDefaultCwndGain;
    start_probe_bw_down(b);
}

/* ------------------------------------------------------------------ */
/* Bandwidth model                                                       */
/* ------------------------------------------------------------------ */
static void advance_max_bw_filter(bbr3_t *b) { b->cycle_count++; }

static void update_max_bw(bbr3_t *b)
{
    update_round(b);
    if (b->rs.valid &&
        (b->rs.delivery_rate >= b->max_bw || !b->rs.is_app_limited)) {
        b->max_bw = minmax_running_max(&b->max_bw_filter, BBRMaxBwFilterLen,
                                       b->cycle_count, b->rs.delivery_rate);
    }
}

static void update_latest_delivery_signals(bbr3_t *b)
{
    b->loss_round_start = 0;
    if (b->rs.valid) {
        b->bw_latest = dmax(b->bw_latest, b->rs.delivery_rate);
        b->inflight_latest = dmax(b->inflight_latest, (double)b->rs.delivered);
    }
    if (b->rs_has_pkt && b->rs.prior_delivered >= b->loss_round_delivered) {
        b->loss_round_delivered = b->delivered;
        b->loss_round_start = 1;
    }
}

static void advance_latest_delivery_signals(bbr3_t *b)
{
    if (b->loss_round_start) {
        b->bw_latest = b->rs.valid ? b->rs.delivery_rate : 0;
        b->inflight_latest = (double)b->rs.delivered;
    }
}

static void init_lower_bounds(bbr3_t *b)
{
    if (b->bw_lo == INF)       b->bw_lo = b->max_bw;
    if (b->inflight_lo == INF) b->inflight_lo = b->cwnd;
}

static void loss_lower_bounds(bbr3_t *b)
{
    b->bw_lo = dmax(b->bw_latest, BBRBeta * b->bw_lo);
    b->inflight_lo = dmax(b->inflight_latest, BBRBeta * b->inflight_lo);
}

static void adapt_lower_bounds_from_congestion(bbr3_t *b)
{
    if (is_probing_bw(b))
        return;
    if (b->loss_in_round) {
        init_lower_bounds(b);
        loss_lower_bounds(b);
    }
}

static int is_inflight_too_high(u64 lost, u64 tx_in_flight)
{
    return tx_in_flight > 0 && (double)lost > (double)tx_in_flight * BBRLossThresh;
}

/* Startup also ends if the round saw many loss events and a loss rate
 * above BBRLossThresh: the bottleneck queue is overflowing. */
static void check_startup_high_loss(bbr3_t *b)
{
    if (b->full_bw_reached || b->state != BBR_STARTUP)
        return;
    if (b->loss_round_start && b->loss_events_in_round >= BBRStartupFullLossCnt &&
        is_inflight_too_high(b->rs.lost, b->rs.tx_in_flight)) {
        b->full_bw_reached = 1;
        b->inflight_hi = dmax(bdp_multiple(b, b->max_bw, 1.0), b->inflight_latest);
    }
}

static void update_congestion_signals(bbr3_t *b)
{
    update_max_bw(b);
    if (b->rs.newly_lost > 0) {
        b->loss_in_round = 1;
        if (b->loss_events_in_round < 0xffff)
            b->loss_events_in_round++;
    }
    if (!b->loss_round_start)
        return;
    check_startup_high_loss(b);
    adapt_lower_bounds_from_congestion(b);
    b->loss_in_round = 0;
    b->loss_events_in_round = 0;
}

static void bound_bw_for_model(bbr3_t *b)
{
    b->bw = dmin(b->max_bw, b->bw_lo);
}

/* ------------------------------------------------------------------ */
/* ACK aggregation (extra_acked)                                         */
/* ------------------------------------------------------------------ */
static void update_ack_aggregation(bbr3_t *b)
{
    double interval = (double)(b->now - b->extra_acked_interval_start) / 1e6;
    double expected = b->bw * interval;
    double extra;
    if (b->extra_acked_delivered <= expected) {
        b->extra_acked_delivered = 0;
        b->extra_acked_interval_start = b->now;
        expected = 0;
    }
    b->extra_acked_delivered += (double)b->rs.newly_acked;
    extra = b->extra_acked_delivered - expected;
    extra = dmin(extra, b->cwnd);
    b->extra_acked = minmax_running_max(&b->extra_acked_filter,
                                        BBRExtraAckedFilterLen,
                                        b->round_count, extra);
}

/* ------------------------------------------------------------------ */
/* Startup / Drain                                                       */
/* ------------------------------------------------------------------ */
static void check_full_bw_reached(bbr3_t *b)
{
    if (b->full_bw_now || !b->rs.valid || b->rs.is_app_limited)
        return;
    if (!b->round_start)
        return;
    if (b->max_bw >= b->full_bw * BBRFullBwThresh) {
        reset_full_bw(b);
        b->full_bw = b->max_bw;
        return;
    }
    b->full_bw_count++;
    b->full_bw_now = b->full_bw_count >= BBRStartupFullBwRounds;
    if (b->full_bw_now)
        b->full_bw_reached = 1;
}

static void check_startup_done(bbr3_t *b)
{
    if (b->state == BBR_STARTUP && b->full_bw_reached)
        enter_drain(b);
}

static void check_drain_done(bbr3_t *b)
{
    if (b->state == BBR_DRAIN &&
        (double)b->inflight <= bbr_inflight(b, b->bw, 1.0))
        enter_probe_bw(b);
}

/* ------------------------------------------------------------------ */
/* ProbeBW                                                               */
/* ------------------------------------------------------------------ */
static int has_elapsed_in_phase(bbr3_t *b, s64 interval)
{
    return b->now > b->cycle_stamp + interval;
}

static int is_reno_coexistence_probe_time(bbr3_t *b)
{
    double reno_rounds = target_inflight(b) / b->mss;
    double rounds = dmin(reno_rounds, BBRMaxRenoRounds);
    return (double)b->rounds_since_bw_probe >= rounds;
}

static int is_time_to_probe_bw(bbr3_t *b)
{
    if (has_elapsed_in_phase(b, b->bw_probe_wait) ||
        is_reno_coexistence_probe_time(b)) {
        start_probe_bw_refill(b);
        return 1;
    }
    return 0;
}

static int is_time_to_cruise(bbr3_t *b)
{
    if ((double)b->inflight > inflight_with_headroom(b))
        return 0;                          /* not enough headroom yet */
    if ((double)b->inflight <= bbr_inflight(b, b->max_bw, 1.0))
        return 1;                          /* inflight <= estimated BDP */
    return 0;
}

static int is_time_to_go_down(bbr3_t *b)
{
    if (b->is_cwnd_limited && b->cwnd >= b->inflight_hi) {
        reset_full_bw(b);                  /* bw limited by inflight_hi */
        b->full_bw = b->rs.valid ? b->rs.delivery_rate : 0;
    } else if (b->full_bw_now) {
        return 1;                          /* bw stopped growing: probed */
    }
    return 0;
}

static void handle_inflight_too_high(bbr3_t *b, int is_app_limited, double tx_in_flight)
{
    b->bw_probe_samples = 0;               /* only react once per probe */
    if (!is_app_limited)
        b->inflight_hi = dmax(tx_in_flight, target_inflight(b) * BBRBeta);
    if (b->state == BBR_PROBE_BW_UP)
        start_probe_bw_down(b);
}

static int check_inflight_too_high(bbr3_t *b)
{
    if (b->rs_has_pkt && is_inflight_too_high(b->rs.lost, b->rs.tx_in_flight)) {
        if (b->bw_probe_samples)
            handle_inflight_too_high(b, b->rs.is_app_limited,
                                     (double)b->rs.tx_in_flight);
        return 1;
    }
    return 0;
}

static void probe_inflight_hi_upward(bbr3_t *b)
{
    double cnt_bytes;
    if (!b->is_cwnd_limited || b->cwnd < b->inflight_hi)
        return;                            /* not fully using inflight_hi */
    b->bw_probe_up_acks += (double)b->rs.newly_acked;
    cnt_bytes = b->bw_probe_up_cnt * b->mss;
    if (b->bw_probe_up_acks >= cnt_bytes) {
        double delta = floor(b->bw_probe_up_acks / cnt_bytes);
        b->bw_probe_up_acks -= delta * cnt_bytes;
        b->inflight_hi += delta * b->mss;
    }
    if (b->round_start)
        raise_inflight_hi_slope(b);
}

static void adapt_upper_bounds(bbr3_t *b)
{
    if (b->ack_phase == ACKS_PROBE_STARTING && b->round_start)
        b->ack_phase = ACKS_PROBE_FEEDBACK;      /* first feedback from probe */
    if (b->ack_phase == ACKS_PROBE_STOPPING && b->round_start) {
        /* First round after the probe ended: the probe's samples are in,
         * so start a new max_bw filter cycle (once per ProbeBW cycle). */
        if (is_in_probe_bw_state(b) && !b->rs.is_app_limited)
            advance_max_bw_filter(b);
        b->ack_phase = ACKS_INIT;
    }
    if (!check_inflight_too_high(b)) {
        if (b->inflight_hi == INF)
            return;
        if ((double)b->rs.tx_in_flight > b->inflight_hi)
            b->inflight_hi = (double)b->rs.tx_in_flight;
        if (b->state == BBR_PROBE_BW_UP)
            probe_inflight_hi_upward(b);
    }
}

static void update_probe_bw_cycle_phase(bbr3_t *b)
{
    if (!b->full_bw_reached)
        return;
    adapt_upper_bounds(b);
    if (!is_in_probe_bw_state(b))
        return;
    switch (b->state) {
    case BBR_PROBE_BW_DOWN:
        if (is_time_to_probe_bw(b))
            return;
        if (is_time_to_cruise(b))
            start_probe_bw_cruise(b);
        break;
    case BBR_PROBE_BW_CRUISE:
        is_time_to_probe_bw(b);
        break;
    case BBR_PROBE_BW_REFILL:
        if (b->round_start) {              /* refilled the pipe for a round */
            b->bw_probe_samples = 1;
            start_probe_bw_up(b);
        }
        break;
    case BBR_PROBE_BW_UP:
        if (is_time_to_go_down(b))
            start_probe_bw_down(b);
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* Min RTT and ProbeRTT                                                  */
/* ------------------------------------------------------------------ */
static void save_cwnd(bbr3_t *b)
{
    if (!b->in_recovery && b->state != BBR_PROBE_RTT)
        b->prior_cwnd = b->cwnd;
    else
        b->prior_cwnd = dmax(b->prior_cwnd, b->cwnd);
}

static void restore_cwnd(bbr3_t *b) { b->cwnd = dmax(b->cwnd, b->prior_cwnd); }

static double probe_rtt_cwnd(bbr3_t *b)
{
    return dmax(bdp_multiple(b, b->bw, ProbeRTTCwndGain), min_pipe_cwnd(b));
}

static void update_min_rtt(bbr3_t *b)
{
    int min_rtt_expired;
    b->probe_rtt_expired = b->now > b->probe_rtt_min_stamp + ProbeRTTInterval;
    if (b->rs.rtt >= 0 &&
        (b->rs.rtt < b->probe_rtt_min_delay || b->probe_rtt_expired)) {
        b->probe_rtt_min_delay = b->rs.rtt;
        b->probe_rtt_min_stamp = b->now;
    }
    min_rtt_expired = b->now > b->min_rtt_stamp + MinRTTFilterLen;
    if (b->probe_rtt_min_delay < b->min_rtt || min_rtt_expired) {
        b->min_rtt = b->probe_rtt_min_delay;
        b->min_rtt_stamp = b->probe_rtt_min_stamp;
    }
}

static void enter_probe_rtt(bbr3_t *b)
{
    b->state = BBR_PROBE_RTT;
    b->pacing_gain = 1.0;
    b->cwnd_gain = ProbeRTTCwndGain;
}

static void exit_probe_rtt(bbr3_t *b)
{
    reset_lower_bounds(b);
    if (b->full_bw_reached) {
        start_probe_bw_down(b);
        start_probe_bw_cruise(b);
    } else {
        enter_startup(b);
    }
}

static void check_probe_rtt_done(bbr3_t *b)
{
    if (b->probe_rtt_done_stamp != 0 && b->now > b->probe_rtt_done_stamp) {
        b->probe_rtt_min_stamp = b->now;    /* schedule the next ProbeRTT */
        restore_cwnd(b);
        exit_probe_rtt(b);
    }
}

static void mark_connection_app_limited(bbr3_t *b)
{
    u64 mark = b->delivered + b->inflight;
    b->app_limited = mark ? mark : 1;
}

static void handle_probe_rtt(bbr3_t *b)
{
    mark_connection_app_limited(b);        /* ignore low rate samples */
    if (b->probe_rtt_done_stamp == 0 &&
        (double)b->inflight <= probe_rtt_cwnd(b)) {
        b->probe_rtt_done_stamp = b->now + ProbeRTTDuration;
        b->probe_rtt_round_done = 0;
        start_round(b);
    } else if (b->probe_rtt_done_stamp != 0) {
        if (b->round_start)
            b->probe_rtt_round_done = 1;
        if (b->probe_rtt_round_done)
            check_probe_rtt_done(b);
    }
}

static void check_probe_rtt(bbr3_t *b)
{
    if (b->state != BBR_PROBE_RTT && b->probe_rtt_expired && !b->idle_restart) {
        enter_probe_rtt(b);
        save_cwnd(b);
        b->probe_rtt_done_stamp = 0;
        b->ack_phase = ACKS_PROBE_STOPPING;
        start_round(b);
    }
    if (b->state == BBR_PROBE_RTT)
        handle_probe_rtt(b);
    if (b->rs.delivered > 0)
        b->idle_restart = 0;
}

/* ------------------------------------------------------------------ */
/* Congestion window                                                     */
/* ------------------------------------------------------------------ */
static void update_max_inflight(bbr3_t *b)
{
    double inflight = bdp_multiple(b, b->bw, b->cwnd_gain);
    inflight += b->extra_acked;
    b->max_inflight = quantization_budget(b, inflight);
}

static void modulate_cwnd_for_recovery(bbr3_t *b)
{
    if (b->rs.newly_lost > 0)
        b->cwnd = dmax(b->cwnd - (double)b->rs.newly_lost, b->mss);
    if (b->packet_conservation) {
        if (b->round_start)
            b->packet_conservation = 0;    /* one round of conservation */
        else
            b->cwnd = dmax(b->cwnd, (double)(b->inflight + b->rs.newly_acked));
    }
}

static void bound_cwnd_for_probe_rtt(bbr3_t *b)
{
    if (b->state == BBR_PROBE_RTT)
        b->cwnd = dmin(b->cwnd, probe_rtt_cwnd(b));
}

static void bound_cwnd_for_model(bbr3_t *b)
{
    double cap = INF;
    if (is_in_probe_bw_state(b) && b->state != BBR_PROBE_BW_CRUISE)
        cap = b->inflight_hi;
    else if (b->state == BBR_PROBE_RTT || b->state == BBR_PROBE_BW_CRUISE)
        cap = inflight_with_headroom(b);
    cap = dmin(cap, b->inflight_lo);       /* apply short-term bound */
    cap = dmax(cap, min_pipe_cwnd(b));
    b->cwnd = dmin(b->cwnd, cap);
}

static void set_cwnd(bbr3_t *b)
{
    update_max_inflight(b);
    modulate_cwnd_for_recovery(b);
    if (!b->packet_conservation) {
        if (b->full_bw_reached)
            b->cwnd = dmin(b->cwnd + (double)b->rs.newly_acked, b->max_inflight);
        else if (b->cwnd < b->max_inflight || b->delivered < initial_cwnd(b))
            b->cwnd = b->cwnd + (double)b->rs.newly_acked;
        b->cwnd = dmax(b->cwnd, min_pipe_cwnd(b));
    }
    bound_cwnd_for_probe_rtt(b);
    bound_cwnd_for_model(b);
}

/* ------------------------------------------------------------------ */
/* Per-ACK driver                                                        */
/* ------------------------------------------------------------------ */
static void update_model_and_state(bbr3_t *b)
{
    update_latest_delivery_signals(b);
    update_congestion_signals(b);
    update_ack_aggregation(b);
    check_full_bw_reached(b);
    check_startup_done(b);
    check_drain_done(b);
    update_probe_bw_cycle_phase(b);
    update_min_rtt(b);
    check_probe_rtt(b);
    advance_latest_delivery_signals(b);
    bound_bw_for_model(b);
}

static void update_control_parameters(bbr3_t *b)
{
    set_pacing_rate_with_gain(b, b->pacing_gain);
    set_send_quantum(b);
    set_cwnd(b);
}

/* ------------------------------------------------------------------ */
/* Public API                                                            */
/* ------------------------------------------------------------------ */
void bbr3_init(bbr3_t *b, u32 mss, s64 now)
{
    memset(b, 0, sizeof(*b));
    b->now = now;
    b->mss = mss;
    b->rng = 0x9E3779B97F4A7C15ULL ^ (u64)now;
    b->cwnd = initial_cwnd(b);
    b->min_rtt = RTT_INF;
    b->min_rtt_stamp = now;
    b->probe_rtt_min_delay = RTT_INF;
    b->probe_rtt_min_stamp = now;
    b->inflight_hi = INF;
    b->max_bw = 0;
    minmax_reset(&b->max_bw_filter, 0, 0);
    minmax_reset(&b->extra_acked_filter, 0, 0);
    b->extra_acked_interval_start = now;
    reset_congestion_signals(b);
    reset_lower_bounds(b);
    reset_full_bw(b);
    b->bw_probe_up_cnt = INF;
    init_pacing_rate(b);
    set_send_quantum(b);
    enter_startup(b);
    b->pending_lost = 0;
}

void bbr3_on_send(bbr3_t *b, bbr_pkt_t *p, u32 size, s64 now)
{
    b->now = now;
    if (b->inflight == 0) {
        b->first_sent_time = now;
        b->delivered_time = now;
        if (b->app_limited) {              /* restarting after idle */
            b->idle_restart = 1;
            b->extra_acked_interval_start = now;
            if (is_in_probe_bw_state(b))
                set_pacing_rate_with_gain(b, 1.0);
            else if (b->state == BBR_PROBE_RTT)
                check_probe_rtt_done(b);
        }
    }
    p->delivered = b->delivered;
    p->delivered_time = b->delivered_time;
    p->first_sent_time = b->first_sent_time;
    p->is_app_limited = b->app_limited != 0;
    p->sent_time = now;
    p->size = size;
    b->inflight += size;
    p->tx_in_flight = b->inflight;
    p->lost = b->lost;
}

void bbr3_ack_begin(bbr3_t *b)
{
    memset(&b->rs, 0, sizeof(b->rs));
    b->rs.rtt = -1;
    b->rs_has_pkt = 0;
    b->rs_pkt_lost = 0;
}

void bbr3_on_packet_acked(bbr3_t *b, const bbr_pkt_t *p, s64 now)
{
    b->now = now;
    b->inflight = b->inflight > p->size ? b->inflight - p->size : 0;
    b->delivered += p->size;
    b->delivered_time = now;
    b->rs.newly_acked += p->size;

    /* Use the most recently sent packet as the basis of the rate sample. */
    if (!b->rs_has_pkt || p->sent_time > b->rs.last_end_time) {
        b->rs_has_pkt = 1;
        b->rs.prior_delivered = p->delivered;
        b->rs.prior_time = p->delivered_time;
        b->rs.is_app_limited = p->is_app_limited;
        b->rs.send_elapsed = p->sent_time - p->first_sent_time;
        b->rs.ack_elapsed = b->delivered_time - p->delivered_time;
        b->rs.tx_in_flight = p->tx_in_flight;
        b->rs.last_end_time = p->sent_time;
        b->rs_pkt_lost = p->lost;
        b->first_sent_time = p->sent_time;
    }

    if (b->in_recovery && p->sent_time > b->recovery_start_time) {
        /* a packet sent after the loss was delivered: recovery is over */
        b->in_recovery = 0;
        b->packet_conservation = 0;
        restore_cwnd(b);
    }
}

/* Estimate the inflight level at which the loss rate crossed the
 * threshold, from the packet that pushed it over (draft's
 * BBRInflightHiFromLostPacket). */
static double inflight_hi_from_lost_packet(u64 tx_in_flight, u64 lost,
                                           u32 size)
{
    double inflight_prev = (double)tx_in_flight - size;
    double lost_prev = (double)lost - size;
    double lost_prefix;
    if (inflight_prev < 0) inflight_prev = 0;
    if (lost_prev < 0) lost_prev = 0;
    lost_prefix = (BBRLossThresh * inflight_prev - lost_prev) /
                         (1.0 - BBRLossThresh);
    if (lost_prefix < 0) lost_prefix = 0;
    return inflight_prev + lost_prefix;
}

void bbr3_on_packet_lost(bbr3_t *b, const bbr_pkt_t *p, s64 now)
{
    b->now = now;
    b->inflight = b->inflight > p->size ? b->inflight - p->size : 0;
    b->lost += p->size;
    b->rs.newly_lost += p->size;

    /* BBRHandleLostPacket: react quickly while probing for bandwidth */
    if (b->bw_probe_samples) {
        u64 lost_since = b->lost - p->lost;
        if (is_inflight_too_high(lost_since, p->tx_in_flight)) {
            double tx = inflight_hi_from_lost_packet(p->tx_in_flight,
                                                     lost_since, p->size);
            handle_inflight_too_high(b, p->is_app_limited, tx);
        }
    }

    if (!b->in_recovery) {                 /* BBROnEnterFastRecovery */
        b->in_recovery = 1;
        b->recovery_start_time = now;
        save_cwnd(b);
        b->entered_recovery = 1;
    }
}

void bbr3_on_packet_discarded(bbr3_t *b, const bbr_pkt_t *p)
{
    b->inflight = b->inflight > p->size ? b->inflight - p->size : 0;
}

void bbr3_on_app_limited(bbr3_t *b)
{
    mark_connection_app_limited(b);
}

void bbr3_ack_end(bbr3_t *b, s64 rtt, s64 now)
{
    b->now = now;
    if (b->rs.newly_acked == 0) {
        /* Losses found by a timer, no delivery: apply them on the next ACK. */
        b->pending_lost += b->rs.newly_lost;
        return;
    }
    b->rs.newly_lost += b->pending_lost;
    b->pending_lost = 0;

    if (b->app_limited && b->delivered > b->app_limited)
        b->app_limited = 0;

    /* Generate the delivery-rate sample. */
    b->rs.rtt = rtt;
    if (b->rs_has_pkt) {
        s64 floor_rtt;
        b->rs.delivered = b->delivered - b->rs.prior_delivered;
        b->rs.lost = b->lost - b->rs_pkt_lost;
        b->rs.interval = b->rs.send_elapsed > b->rs.ack_elapsed ?
                         b->rs.send_elapsed : b->rs.ack_elapsed;
        floor_rtt = b->min_rtt != RTT_INF ? b->min_rtt : rtt;
        b->rs.valid = b->rs.interval > 0 && b->rs.delivered > 0 &&
                      !(floor_rtt > 0 && b->rs.interval < floor_rtt);
        if (b->rs.valid)
            b->rs.delivery_rate = (double)b->rs.delivered * 1e6 / (double)b->rs.interval;
    }

    if (b->entered_recovery) {
        b->entered_recovery = 0;
        b->cwnd = (double)b->inflight + dmax((double)b->rs.newly_acked, b->mss);
        b->packet_conservation = 1;
        start_round(b);
    }

    update_model_and_state(b);
    update_control_parameters(b);
}

const char *bbr3_state_name(const bbr3_t *b)
{
    switch (b->state) {
    case BBR_STARTUP:         return "Startup";
    case BBR_DRAIN:           return "Drain";
    case BBR_PROBE_BW_DOWN:   return "ProbeBW_DOWN";
    case BBR_PROBE_BW_CRUISE: return "ProbeBW_CRUISE";
    case BBR_PROBE_BW_REFILL: return "ProbeBW_REFILL";
    case BBR_PROBE_BW_UP:     return "ProbeBW_UP";
    case BBR_PROBE_RTT:       return "ProbeRTT";
    }
    return "?";
}
