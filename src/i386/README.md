# udpbbr: UDP file transfer with a from-scratch BBRv3

A small client/server in C that transfers files over UDP. The sender's
congestion control is our own implementation of **BBR version 3**, written
from the IETF draft *BBR Congestion Control* (draft-ietf-ccwg-bbr) and
*Delivery Rate Estimation* (draft-cheng-iccrg-delivery-rate-estimation).
A small link emulator is included so you can watch BBR work on an emulated
bottleneck without root access.

```
make
./udpbbr_server -p 9001 -d /path/to/files          # serve a directory
./udpbbr_client -p 9001 server.example.com big.iso  # fetch big.iso
```

## Files

| File | What it is |
|---|---|
| `bbr3.c`, `bbr3.h` | BBRv3 congestion control, transport-agnostic (~960 lines with comments) |
| `server.c` | Sender: reliability, loss detection, PTO, pacing, BBRv3 |
| `client.c` | Receiver: writes chunks, sends range ACKs |
| `proto.h` | Wire format and helpers |
| `compat.h` | Portability layer: types, clock, `select()`, file I/O, name lookup |
| `linkemu.c` | UDP link emulator: bottleneck rate, drop-tail queue, delay, loss |
| `i386_divdi.c` | 64-bit division helpers for 32-bit builds without libgcc |

## Building

The sources are plain C89 (plus `long long`), so the same code builds with
a current gcc or clang and with **gcc 2.7.2.3 on late-1990s Linux**
(glibc 2.0 or libc5, kernel 2.0/2.2). `compat.h` hides the differences:

| Modern system | Old system |
|---|---|
| `clock_gettime(CLOCK_MONOTONIC)` | `gettimeofday()` |
| `socklen_t` | `size_t` (glibc 2.0) or `int` (libc5) |
| `prctl()` for 1 µs timer slack | not used |
| 64-bit file offsets | 32-bit `off_t`: files up to 2 GB |

Everything else (`select()`, `lseek()`+`read()`/`write()`,
`gethostbyname()`) is the same on both.

```
make                    # server, client, linkemu
make CC=gcc CFLAGS="-O2 -g"
```

The Makefile uses only plain `=` assignments, which old GNU make
understands.

If a build fails on an in-between system:

* **Undefined `clock_gettime`.** glibc 2.1 to 2.16 keep it in librt. Use
  `make LDLIBS="-lm -lrt"`, or add `-DNO_CLOCK_GETTIME` to `CFLAGS`.
* **Type errors around `socklen_t`.** Add `-DSOCKLEN_T=int` (or `size_t`, or
  `socklen_t`) to `CFLAGS`.

Old kernels sleep in 10 ms ticks (HZ=100). The server measures its timer
granularity at startup (shown in its first line) and lets the pacer send
correspondingly larger bursts, so pacing stays on target.

`make c89check` uses a modern gcc to check that the sources are still
C89-clean.

### 32-bit and static clients

* **32-bit client on a modern 64-bit system** with gcc-multilib:
  `make client32`.
* **Fully static clients** that run on any x86 Linux 2.6 or later with no
  library dependencies, built against [musl](https://musl.libc.org):

```
# once, from the musl 1.2.5 source:
CC="gcc -m32 -march=i486" ./configure --target=i386-linux-musl \
    --prefix=$HOME/musl32 --disable-shared
make AR=ar RANLIB=ranlib install && make distclean
CC=gcc ./configure --prefix=$HOME/musl64 --disable-shared && make install

# then, here:
make udpbbr_client MUSL64=$HOME/musl64
make client32      MUSL32=$HOME/musl32
```

The 32-bit static build doesn't need gcc's 32-bit runtime (libgcc):
`i386_divdi.c` supplies the 64-bit division routines that a 32-bit CPU
needs. The prebuilt binaries were made this way:

| Binary | Target | Runs on |
|---|---|---|
| `udpbbr_client` | x86-64, static (musl) | any x86-64 Linux |
| `udpbbr_client32` | i386, static (musl), `-march=i486` | 486 or later CPU with Linux 2.6+, including 64-bit kernels with 32-bit support |

musl needs Linux 2.6 or later, so on a 2.0/2.2 kernel build from source
with the system's own gcc instead.

## Usage

```
udpbbr_server [-p port] [-d dir] [-l log.csv] [-q]
    -l  write BBR state every 5 ms as CSV (state, bw, pacing, cwnd,
        inflight, min_rtt, srtt, inflight_hi, losses)
    -q  no per-second status lines

udpbbr_client [-p port] host remote_name [local_file]

linkemu [-l listen_port] [-s server_host:port] [-r Mbps] [-d one_way_ms]
        [-q queue_KB] [-L data_loss] [-A ack_loss] [-P packets] [-v]
    -P  packets the emulated link can hold (default 8192, ~1.5 KB each x 2;
        lower it on small-memory machines)
```

The server prints one line per second:

```
 3.0s ProbeBW_DOWN   bw 19.83 Mb/s pacing 17.67 cwnd 193K inflight 153K inflight_hi 193K min_rtt 40.7ms ...
```

### Trying it on an emulated link

```
./udpbbr_server -p 9001 -d . -l bbr.csv &
./linkemu -l 9000 -s 127.0.0.1:9001 -r 20 -d 20 -q 100 -v &   # 20 Mb/s, 40 ms RTT, 100 KB queue
./udpbbr_client -p 9000 127.0.0.1 somefile out.bin
```

## Protocol

* **REQ / META** – the client asks for a file by name; the server answers
  with its size and a session id. The file name may not contain `/`, `..`,
  or start with `.`.
* **DATA** – 1200-byte chunks. Every packet gets a new, increasing
  **packet number**; a retransmission repeats the chunk under a new number,
  as in QUIC. ACKs are therefore never ambiguous, and every ACK gives a clean
  RTT and delivery-rate sample, which BBR depends on.
* **ACK** – the receiver reports up to 32 ranges of received packet
  numbers, plus its ACK delay. It ACKs every second packet, immediately on a
  gap, and within 1 ms otherwise.
* **DONE** – the receiver has every chunk.

Loss detection follows RFC 9002. A packet is lost when 3 later packets have
been acknowledged, or when it is 9/8 × RTT older than an acknowledged one.
A probe timeout (PTO) resends the oldest outstanding data when ACKs stop,
which recovers tail losses.

## BBRv3 implementation

`bbr3.c` mirrors the draft's pseudocode. `BBRUpdateMaxBw` becomes
`update_max_bw()`, and so on, so the two can be read side by side. Each ACK
runs `update_model_and_state()` followed by `update_control_parameters()`,
in the draft's order. The parts:

* **Delivery-rate sampling.** Each packet records `C.delivered`, the
  delivered time, the first-sent time, the app-limited flag, inflight and
  `C.lost` when sent. Each ACK turns the newest acked packet into a rate
  sample: delivered ÷ max(send interval, ACK interval).
* **Long-term model.** `max_bw` is a windowed max over 2 ProbeBW cycles
  (Kathleen Nichols' filter, as in Linux `minmax`). `inflight_hi` is cut to
  max(inflight at loss, 0.7 × target) when the loss rate exceeds **2%**
  (`BBRLossThresh`), including the per-packet estimate from
  `BBRInflightHiFromLostPacket`.
* **Short-term model.** `bw_lo` and `inflight_lo` drop to
  max(latest, 0.7 × current) in any loss round while not probing, and reset
  when probing starts.
* **States.** Startup (pacing gain 2.77; exits on a bandwidth plateau, or on
  ≥ 6 loss events with > 2% loss) → Drain (0.35) → ProbeBW
  DOWN (0.90) / CRUISE (1.0, 15% headroom below `inflight_hi`) /
  REFILL (1.0, one round) / UP (1.25, cwnd gain 2.25, exponentially
  growing `inflight_hi`) → ProbeRTT (cwnd 0.5 × BDP for 200 ms, every 5 s
  without a lower RTT).
* **Probe timing.** Bandwidth probes start after a random 2–3 s wall-clock
  wait, or after the Reno-coexistence round count (min(BDP in packets, 63)
  rounds), whichever comes first.
* **cwnd.** BDP × gain + `extra_acked` (ACK aggregation, 10-round max
  filter) + a quantization budget, bounded by `inflight_hi`, `inflight_lo`,
  ProbeRTT and a 4-packet floor. One round of packet conservation on
  entering loss recovery.
* **Pacing.** gain × bw × 0.99. The sender allows a burst of one send
  quantum (1 ms worth, up to 64 KB) after an idle period.

## Test results

All runs used `linkemu`. Every transfer was compared byte for byte against
the original.

| Link | File | Goodput | Notes |
|---|---|---|---|
| 20 Mb/s, 40 ms RTT, 100 KB queue | 30 MB | 18.73 Mb/s | ceiling is ~19.25 Mb/s after headers; avg queueing delay 7.8 ms |
| same, 10% ACK loss | 30 MB | 18.72 Mb/s | unaffected: range ACKs are redundant |
| same, 15 KB queue (0.15 BDP) | 30 MB | 16.14 Mb/s | avg queueing delay 0.6 ms |
| same, 1% random data loss | 30 MB | 13.60 Mb/s | see below |
| same, 5% random data loss | 30 MB | 3.19 Mb/s | above the 2% threshold; standard TCP's loss formula predicts ~1.3 Mb/s |
| 100 Mb/s, 100 ms RTT, 1.2 MB queue | 100 MB | 86.81 Mb/s | 98–100% link utilisation after a 1.3 s Startup |
| loopback, no emulator | 100 MB | ~2.5 Gb/s | single-threaded sender |

**Old-system emulation.** A build with the old-system code paths
(`gettimeofday()`, `int` socket lengths), with every `select()` rounded up
to 10 ms ticks as on a HZ=100 kernel:

| Link | File | Goodput |
|---|---|---|
| 20 Mb/s, 40 ms RTT, 100 KB queue | 30 MB | 18.90 Mb/s |
| same, 1% random loss | 30 MB | 16.33 Mb/s |
| 10 Mb/s Ethernet LAN, 1 ms RTT, 32 KB buffer | 10 MB | 9.59 Mb/s (99.7% of the link after headers) |

The code was not compiled with gcc 2.7.2.3 itself; the C89 check and the
emulation above stand in for it.

A typical state sequence on the 20 Mb/s link: Startup ends at 357 ms,
Drain lasts 60 ms, then ProbeBW cycles run every 2–3 s (REFILL → UP → DOWN →
CRUISE), and ProbeRTT runs about every 5–7 s.

**Behaviour under random loss.** With 1% non-congestive loss, `max_bw`
stays at the link rate, but BBRv3's short-term `bw_lo` falls between probes.
Every round with a loss resets it to max(`bw_latest`, 0.7 × `bw_lo`), and
under random loss the measured rate sits slightly below the current
estimate. The next bandwidth probe resets it. This is the specified v3
loss response and the same logic as Linux's `bbr_adapt_lower_bounds`, not
a bug. Real paths have noisier rate samples, which soften it.

## Limitations

This is an example, not a production protocol:

* One transfer at a time; other clients get "busy".
* IPv4 only.
* On systems with a 32-bit `off_t` (glibc 2.0, libc5), files are limited to
  2 GB; the client refuses larger ones with a clear message.
* No encryption or authentication, and no checksum beyond UDP's.
* No ECN.
* The sender keeps ~72 bytes of state per packet sent: about 6 MB for a
  100 MB file. A production version would recycle that memory.
* BBRv3 details that are still moving in the drafts (exact Startup loss-exit
  rule, Drain gain, `bw_hi`) follow the version described above.
