# Lynx Hub Optimization Notes

Notes digested from a Discord conversation between **ThadHouse** (WPILib, working on
SystemCore's Lynx hub support) and **Eeshwar Krishnan** (author of PhotonFTC) on
2026-03-18 about how to get a higher loop rate over the Lynx UART protocol.

These notes are the source of truth for tuning the daemon's outbound packet pipeline.

## How SystemCore gets a higher loop rate than the Control Hub

SystemCore is faster than a stock Control Hub Java OpMode for two reasons, neither
of which is "the protocol is faster":

1. **Each hub gets its own USB port.** Two hubs can be talked to in parallel
   instead of being serialized over a single UART (and even worse, RS485 chained).
2. **It supports far less.** No RS485 daisy-chain, no I2C, no DIO, no Analog Input.
   Only motor, servo, and "sensor port" command paths exist. That keeps the
   command set small enough that the entire pipeline can be handled asynchronously.

Quote (ThadHouse):

> By being much more limited on what is supported, and doing as much as possible
> asynchronously. No RS485 support, no I2C support, no DIO or Analog Input
> support. And then each hub is individually hooked up to its own USB port on
> Systemcore. So I can talk to 2 hubs in parallel.

It still uses the Lynx (RHSP) protocol — the speed comes from how the host
schedules packets, not from a different wire format.

## Hub firmware buffer limits (the numbers that matter)

These are the buffer sizes inside the hub firmware itself. Going beyond them
causes the hub to drop / NACK packets, which is exactly the failure mode the
"Skipping due to outstanding" + recover loop produces in this daemon.

From ThadHouse, who has talked to someone with Lynx firmware source access:

| Buffer                       | Size                          |
| ---                          | ---                           |
| UART RX                      | **128 bytes**                 |
| Global packet queue          | **~15 packets**               |
| Per-subsystem queue          | **~10 packets**               |
| Effective in-flight maximum  | **~10–15 packets**            |

Eeshwar landed on 15 as PhotonFTC's max experimentally; ThadHouse confirmed the
real ceiling is in the 10–15 range once you account for all three layers.

## Outstanding-packet count: more is not faster

Quote (ThadHouse):

> Anything above like 5 doesn't actually make things faster. All you need to do
> is saturate the UART. Any faster than that does nothing.

Implication for `MAX_NUM_OUTSTANDING_MESSAGES` in `LynxUsbDevice.cpp`:

- 4–8 is the right ballpark for a single hub on a single UART.
- Pushing it higher does **not** raise throughput, it just risks tripping the
  hub's internal queues — which manifests as hub-side drops, NACKs, or in our
  case the "Skipping due to outstanding" stall.
- The metric to optimize for is "UART line is saturated for the whole transmit
  window," not "more outstanding packets in flight."

ThadHouse confirmed via scope that with proper ordering the UART line stays
saturated the entire transmit time.

## Packet ordering matters

Quote (Eeshwar):

> iirc you could get faster by ordering the packets in certain ways — i think by
> processing time. There were definitely specific orders that maximized
> utilization.

The win comes from interleaving packets whose hub-side processing time differs,
so that fast packets free up subsystem-queue slots while a slower packet is
still being processed. ThadHouse pointed at PhotonFTC's ordering as a "good
enough" reference. See PhotonFTC source below.

For this daemon, the current per-tick send order in
`main_enhanced.cpp::SendCommands` is roughly: motor power, motor current
request, motor mode, motor enable, servo config, servo PW, servo enable, then
I2C. That is reasonable but worth revisiting — heavy/blocking ops (I2C reads,
RS485-relayed packets) probably want to be issued early so fast servo writes can
flow through their subsystem queue while we wait.

## Packet ID trick

ThadHouse noted that since SystemCore only sends ~36 unique commands, he uses
the *unique values themselves as the packet IDs*. We don't need to mirror that
exactly, but the principle — keep the dispatch table tiny and predictable — is
worth keeping in mind if we ever rewrite the ID-to-handler mapping.

## RS485: head-of-line blocking is real

The Discord didn't dwell on this, but it is the operational consequence for
*us*: SystemCore avoids RS485 entirely. When we daisy-chain a child module via
RS485 from a parent, every child packet is relayed through the parent's UART
queue. A slow child response stalls the parent's whole subsystem queue, which
is exactly what we observe with two modules where the second is RS485-attached.

Practical takeaways for this daemon:
- Treat RS485-relayed packets as expensive — keep their per-tick count low.
- The "two-modules → timeout → recover" loop is consistent with hub buffers
  filling because RS485 round-trip latency holds slots open longer than direct
  UART responses do.

## References

- **PhotonFTC blog post:** <https://blog.eeshwark.com/blog/photonftc-basic-explanation>
- **PhotonFTC source:** <https://github.com/Eeshwar-Krishnan/PhotonFTC>
- **RHSP protocol spec (mirrored in repo):** [RHSP.md](RHSP.md)
- **Python reference impl:** <https://github.com/unofficial-rev-port/REVHubInterface/blob/main/REVHubInterface/REVcomm.py>

## Concrete implications for this codebase

1. `MAX_NUM_OUTSTANDING_MESSAGES` should stay in the 4–8 range. Don't raise it
   chasing throughput; that path leads to hub-side drops.
2. The `CachedCommand::Get()` + `Ack()` pattern (now applied in
   `main_enhanced.cpp`) is the right idea — keep per-tick command count down so
   we never approach the ~10/subsystem queue limit.
3. For the two-module RS485 case, the bottleneck is the parent hub's queue, not
   the wire. Reducing per-tick commands and ordering RS485-bound packets early
   is more productive than raising timeouts or batch sizes.
4. Long-term, the SystemCore-style "one USB port per hub, parallel I/O" approach
   is what actually scales. Anything we do over a chained RS485 link is
   inherently capped by the parent's subsystem queues.
