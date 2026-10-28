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

#include "compat.h"

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

static INLINE u8 *put16(u8 *p, u16 v)
{
    p[0] = (u8)(v >> 8);
    p[1] = (u8)v;
    return p + 2;
}

static INLINE u8 *put32(u8 *p, u32 v)
{
    int i;
    for (i = 3; i >= 0; i--)
        *p++ = (u8)(v >> (8 * i));
    return p;
}

static INLINE u8 *put64(u8 *p, u64 v)
{
    int i;
    for (i = 7; i >= 0; i--)
        *p++ = (u8)(v >> (8 * i));
    return p;
}

static INLINE u16 get16(const u8 *p)
{
    return (u16)(p[0] << 8 | p[1]);
}

static INLINE u32 get32(const u8 *p)
{
    u32 v = 0;
    int i;
    for (i = 0; i < 4; i++)
        v = v << 8 | p[i];
    return v;
}

static INLINE u64 get64(const u8 *p)
{
    u64 v = 0;
    int i;
    for (i = 0; i < 8; i++)
        v = v << 8 | p[i];
    return v;
}

#endif
