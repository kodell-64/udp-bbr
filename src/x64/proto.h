/*
 * proto.h - wire format and helpers shared by the UDP file transfer
 * client and server.
 *
 * All integers are big-endian. Packet numbers increase by one for every
 * packet sent and are never reused (as in QUIC): a retransmission carries
 * the same file chunk under a new packet number, so ACKs are never
 * ambiguous and every ACK yields a clean RTT and delivery-rate sample.
 *
 *  REQ   c->s  u8 type=1 | u8 name_len | name
 *  META  s->c  u8 type=2 | u8 status (0 ok, 1 not found) | u32 session
 *              | u64 file_size | u16 chunk_size
 *  DATA  s->c  u8 type=3 | u32 session | u64 pn | u64 chunk | u16 len | bytes
 *  ACK   c->s  u8 type=4 | u32 session | u64 largest_pn | u32 ack_delay_us
 *              | u16 n | n * (u64 hi, u64 lo)   ranges of received pns,
 *                                               highest first, inclusive
 *  DONE  c->s  u8 type=5 | u32 session        all chunks received
 */
#ifndef PROTO_H
#define PROTO_H

#include <stdint.h>
#include <string.h>
#include <time.h>

#define PKT_REQ   1
#define PKT_META  2
#define PKT_DATA  3
#define PKT_ACK   4
#define PKT_DONE  5

#define CHUNK_SIZE      1200            /* file bytes per DATA packet      */
#define DATA_HDR        (1 + 4 + 8 + 8 + 2)
#define MAX_PKT         1500
#define MAX_ACK_RANGES  32
#define WIRE_OVERHEAD   28              /* IPv4 + UDP headers, for stats   */

static inline int64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

static inline uint8_t *put16(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = v; return p + 2; }
static inline uint8_t *put32(uint8_t *p, uint32_t v) { for (int i = 3; i >= 0; i--) *p++ = v >> (8 * i); return p; }
static inline uint8_t *put64(uint8_t *p, uint64_t v) { for (int i = 7; i >= 0; i--) *p++ = v >> (8 * i); return p; }
static inline uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] << 8 | p[1]); }
static inline uint32_t get32(const uint8_t *p) { uint32_t v = 0; for (int i = 0; i < 4; i++) v = v << 8 | p[i]; return v; }
static inline uint64_t get64(const uint8_t *p) { uint64_t v = 0; for (int i = 0; i < 8; i++) v = v << 8 | p[i]; return v; }

#endif
