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
| `linkemu.c` | UDP link emulator: bottleneck rate, drop-tail queue, delay, loss |
| `i386_divdi.c` | 64-bit division helpers for 32-bit builds without libgcc |

## Building the client: 64-bit and 32-bit

`make` builds a normal, dynamically linked 64-bit client. Two more options:

**32-bit client**, if your system has 32-bit libraries
(Debian/Ubuntu: `apt install gcc-multilib`):

```
make client32                      # -> udpbbr_client32
```

**Fully static clients** that run on any x86 Linux, with no library
dependencies, built against [musl](https://musl.libc.org) libc:

```
# once, from the musl 1.2.5 source:
CC="gcc -m32 -march=i486" ./configure --target=i386-linux-musl \
    --prefix=$HOME/musl32 --disable-shared
make AR=ar RANLIB=ranlib install && make distclean
CC=gcc ./configure --prefix=$HOME/musl64 --disable-shared && make install

# then, here:
make udpbbr_client MUSL64=$HOME/musl64
make client32      MUSL32=$HOME/musl32 CFLAGS="-std=gnu11 -O2 -Wall -Wextra -march=i486"
```

The 32-bit static build doesn't need gcc's 32-bit runtime (libgcc):
`i386_divdi.c` supplies the 64-bit division routines that a 32-bit CPU
needs. The prebuilt binaries were made this way:

| Binary | Target | Runs on |
|---|---|---|
| `udpbbr_client` | x86-64, static (musl) | any x86-64 Linux |
| `udpbbr_client32` | i386, static (musl), `-march=i486` | 486 or later CPU; also 64-bit kernels with 32-bit support |

Both clients talk to the same server. File offsets are 64-bit throughout,
so 32-bit builds should handle files over 4 GB; the largest file tested
here is 100 MB. The kernel minimum comes from `ppoll()` (Linux 2.6.16).

## Usage

```
udpbbr_server [-p port] [-d dir] [-l log.csv] [-q] [-D] [-o logfile] [-P pidfile]
    -l  write BBR state every 5 ms as CSV (state, bw, pacing, cwnd,
        inflight, min_rtt, srtt, inflight_hi, losses)
    -q  no per-second status lines
    -D  run as a daemon in the background
    -o  with -D: append log messages to this file (default: discard them)
    -P  write the process ID to this file; removed on exit

udpbbr_client [-p port] host remote_name [local_file]

linkemu [-l listen_port] [-s server_host:port] [-r Mbps] [-d one_way_ms]
        [-q queue_KB] [-L data_loss] [-A ack_loss] [-v]
```

The server prints one line per second:

```
 3.0s ProbeBW_DOWN   bw 19.83 Mb/s pacing 17.67 cwnd 193K inflight 153K inflight_hi 193K min_rtt 40.7ms ...
```

### Running as a daemon

```
./udpbbr_server -D -d /srv/files -o /var/log/udpbbr.log -P /run/udpbbr.pid
kill $(cat /run/udpbbr.pid)          # stop it
```

With `-D` the server detaches the classic Unix way: fork, `setsid`, fork
again, `chdir("/")`, `umask 022`, stdin from `/dev/null`, output to the `-o`
file. It ignores SIGHUP. SIGTERM or SIGINT ends the current transfer, logs
how much of it was delivered, removes the PID file and exits with status 0.

Before detaching, it resolves relative paths (`-d`, `-o`, `-P`), checks the
directory exists, and binds the port. A missing directory or a port already
in use is therefore reported on the terminal with exit status 1. It also
refuses to start if the PID file names a process that is still running.

Daemon mode logs one timestamped line per event: startup, each request,
one summary per transfer, and shutdown.

```
2026-10-28 16:59:33 serving /srv/files on UDP port 9953 (pid 391)
2026-10-28 16:59:33 10.0.0.7:51561 requested 'big.iso' (30000000 bytes)
2026-10-28 16:59:36 STOPPED: 6777600 of 30000000 bytes in 2.98 s = 18.20 Mb/s goodput | ...
2026-10-28 16:59:36 stopping (signal received)
```

Under systemd, run it in the foreground instead and let systemd manage it:

```
[Service]
ExecStart=/usr/local/bin/udpbbr_server -q -d /srv/files
Restart=on-failure
```

If the server stops mid-transfer, the client gives up after 20 s without
data ("server went silent").

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
* No encryption or authentication, and no checksum beyond UDP's.
* No ECN.
* The sender keeps ~72 bytes of state per packet sent: about 6 MB for a
  100 MB file. A production version would recycle that memory.
* BBRv3 details that are still moving in the drafts (exact Startup loss-exit
  rule, Drain gain, `bw_hi`) follow the version described above.
