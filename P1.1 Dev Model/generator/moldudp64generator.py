"""
Python MoldUDP64 Generator — Prototype P1.1
Warm-up & Timed Testing Harness

Warm-up Phase:
  • Send dummy packets to prime caches, branch predictors, and descriptor rings

Timed Phase:
  • Send test packets to capture lowest-latency measurements

Simulation Tests:
  - Overflow burst: 17+ back-to-back packets to exercise RX-ring overflow behavior
  - Wrap-around: Send enough packets to cycle rx_head/rx_tail indices
  - AAPL flood: Only AAPL messages, measure spin-loop delays for free TX descriptors
  - Noise flood: Non-AAPL messages, measure RXREAD and LOGIC cycle counts with branch bypass
  - Cold start vs. warm cache comparison
  - Error injection: malformed length or corrupted header → ensure no hang or crash

Dynamic Volume Patterns:
  • Linear ramp: increase pps from start to end over a fixed duration
  • Burst pause: alternate high-rate bursts and idle periods
  • Random jitter: add variability to inter-packet delays
  • Relevant/noise ratio sweep: uniform mix per run for accurate comparisons
"""

import argparse         #parsing command-line flags
import socket           #raw UDP datagrams to feed QEMU’s plugin
import time             #for sleeps to pace packet emission
import random           #jitter mode to randomize inter-packet delays
from constants import TOTAL_BYTES, MSG_LEN, AAPL_LOCATE, NON_AAPL_LOCATE

def make_raw_packet(session_id, msg_type, stock_locate, price, shares,
                    malformed=False, corrupt=False):
    """
    Construct a raw MoldUDP64 packet:
      - 8-byte session ID (big-endian)
      - 4-byte payload length (big-endian; normally 11)
      - Payload struct <cHII>: 1-byte msg_type, 2-byte stock_locate (LE),
        4-byte price (LE), 4-byte shares (LE)
    Options:
      malformed: set length > TOTAL_BYTES to trigger oversize logic
      corrupt: flip bits in the first payload byte
    """
    # 1) Session ID into an 8-byte big-endian format
    pkt = int(session_id).to_bytes(8, 'big')                      # Verilog FSM reads the session tag MSB first, so the generator must match

    # 2) Length field
    length = 1 + 2 + 4 + 4   # msg_type + stock_locate + price + shares
    if malformed:            # triggers kernel’s drop-oversize path, covering that error-handling 
        length = TOTAL_BYTES + 1
    pkt += int(length).to_bytes(4, 'big')

    # 3) Payload in array format to allow mutation for the corrupt flag
    payload = bytearray()
    payload += msg_type.encode('ascii')                          # 1 byte
    payload += int(stock_locate).to_bytes(2, 'little')
    payload += int(price).to_bytes(4, 'little')
    payload += int(shares).to_bytes(4, 'little')
    if corrupt:                                                  # testing header-corruption recovery 
        payload[0] ^= 0xFF
    pkt += bytes(payload)

    return pkt                                                   # returns a byte object of length 8 + 4 + payload_size


#The function is strucutred in this format so I can get tangiable results for the paper
def parse_args():
    parser = argparse.ArgumentParser(
        description="Python MoldUDP64 Generator with warm-up, timing, and stress-test modes"
    )

    # Destination settings for CLI 
    dest = parser.add_argument_group("Destination")
    dest.add_argument("--dst-ip", type=str, default="127.0.0.1",
                      help="Destination IP for UDP packets")
    dest.add_argument("--dst-port", type=int, default=9000,
                      help="Destination UDP port")

    # Warm-up and timed phases
    phases = parser.add_argument_group("Warm-up & Timed Phases")

    phases.add_argument("--warmup-count", type=int, default=10,         # Convert the CLI string into an integer, erroring out if it’s not valid
                        help="Number of dummy packets for warm-up")     # if omitted args.warmup_count will be 10
    
    phases.add_argument("--warmup-interval", type=float, default=0.1,   # 100ms gap for system to process each packet fully
                        help="Delay between warm-up packets (s)")
    
    phases.add_argument("--test-count", type=int, default=5,            # Measured packets after warm-up
                        help="Number of timed packets to send")
    
    phases.add_argument("--test-interval", type=float, default=0.0,     # For volility in packet sending
                        help="Delay between timed packets (s)")

    # Traffic patterns
    patterns = parser.add_argument_group("Traffic Patterns")
    patterns.add_argument("--mode", choices=["fixed", "ramp", "burst-pause", "jitter"],
                          default="fixed", help="Traffic pattern mode")
    
    # For ramp, linear increase from start pps to end pps
    patterns.add_argument("--ramp-start", type=int, default=10000,
                          help="Start rate for ramp mode (pps)")
    patterns.add_argument("--ramp-end", type=int, default=1000000,
                          help="End rate for ramp mode (pps)")
    patterns.add_argument("--ramp-duration", type=float, default=10.0,
                          help="Duration of ramp mode (s)")
    
    # For burst-pause, 100 000 pps burst
    patterns.add_argument("--burst-rate", type=int, default=100000,
                          help="Burst rate for burst-pause mode (pps)")
    patterns.add_argument("--burst-duration", type=float, default=1.0,
                          help="Burst duration (s)")
    patterns.add_argument("--pause-duration", type=float, default=1.0,
                          help="Pause duration between bursts (s)")
    
    # For jitter, randomness into packet spacing
    patterns.add_argument("--jitter", type=float, default=0.0,
                          help="Max jitter to add/subtract (s)")

    # Error injection, to test the kernel's handleing of oversize or corrupted packets by flipping flags
    errors = parser.add_argument_group("Error Injection")
    errors.add_argument("--malformed-length", action="store_true",
                        help="Set length field > buffer size")
    errors.add_argument("--corrupt-header", action="store_true",
                        help="Flip first payload byte")

    # Payload parameters
    payload = parser.add_argument_group("Payload Parameters")

    #Session ID, lambda to pass hex 
    payload.add_argument("--session-id", type=lambda x: int(x,0), default=0,
                         help="8-byte session ID (hex or int)")
    
    # R = Reset, A = AAPL, X = noise
    payload.add_argument("--msg-type", type=str, choices=["R","A","X"], default="A",
                         help="Message type ('R','A','X')")
    
    # 16-bit symbol id in the payload, lambda for hex
    payload.add_argument("--stock-locate", type=lambda x: int(x,0),
                         default=AAPL_LOCATE, help="Stock locate code")
    
    #32-bit little-endian fields for Price and Shares
    payload.add_argument("--price", type=int, default=10000,
                         help="Price field")
    payload.add_argument("--shares", type=int, default=100,
                         help="Shares field")

    return parser.parse_args()


def main():
    args = parse_args()
    print(f"Configuration: {args}")     # reads all CLI flags into args

    # Build dummy packet for warm-up cache (AAPL message to prefetch AAPL path in L2 cache)
    dummy_packet = make_raw_packet(
        session_id=args.session_id,
        msg_type='A',
        stock_locate=AAPL_LOCATE,
        price=0,
        shares=0,
        malformed=False,
        corrupt=False
    )

    # Creates a new IPv4 UDP socket, socket setup + length print
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = (args.dst_ip, args.dst_port)                               # Packs the target IP and port into a tuple 
    print(f"Warm-up dummy packet length: {len(dummy_packet)} bytes") # Guard rail for debugging

    # 1) Warm-up Phase
    print(f"Starting warm-up: {args.warmup_count} dummy packets")   # Logs how many warm up packets are going to be sent
    for _ in range(args.warmup_count):
        sock.sendto(dummy_packet, dst)                              # Sends the same clean AAPL dummy packets to QEMU plugin
        time.sleep(args.warmup_interval)                            # Prevents back-to-back on warm up




    # 2) First timed packet (optional cold-vs-warm comparison)
    first_packet = make_raw_packet(
        session_id=args.session_id,
        msg_type=args.msg_type,
        stock_locate=args.stock_locate,
        price=args.price,
        shares=args.shares,
        malformed=args.malformed_length,
        corrupt=args.corrupt_header
    )
    print("Sending first timed packet...")
    sock.sendto(first_packet, dst)
    time.sleep(args.test_interval)

    # 3) Fixed-rate Test Loop
    if args.mode == "fixed":
        print(f"Starting fixed-rate test: {args.test_count} packets @ interval {args.test_interval}s")
        for _ in range(args.test_count):
            packet = make_raw_packet(
                session_id=args.session_id,
                msg_type=args.msg_type,
                stock_locate=args.stock_locate,
                price=args.price,
                shares=args.shares,
                malformed=args.malformed_length,
                corrupt=args.corrupt_header
            )
            sock.sendto(packet, dst)
            time.sleep(args.test_interval)
    elif args.mode == "ramp":
        print(f"Starting ramp test: {args.ramp_start}→{args.ramp_end} pps over {args.ramp_duration}s")
        steps = int(args.ramp_duration * 100)
        for i in range(steps):
            rate = args.ramp_start + (args.ramp_end - args.ramp_start) * (i / steps)
            interval = 1.0 / rate
            packet = make_raw_packet(
                session_id=args.session_id,
                msg_type=args.msg_type,
                stock_locate=args.stock_locate,
                price=args.price,
                shares=args.shares,
                malformed=args.malformed_length,
                corrupt=args.corrupt_header
            )
            sock.sendto(packet, dst)
            time.sleep(interval)
        print("Ramp test complete.")
    elif args.mode == "burst-pause":
        total_duration = args.ramp_duration
        end_time = time.time() + total_duration
        print(f"Starting burst-pause test: {args.burst_rate} pps for {args.burst_duration}s, "
              f"pause {args.pause_duration}s, for {total_duration}s")
        while time.time() < end_time:
            burst_count = int(args.burst_rate * args.burst_duration)
            for _ in range(burst_count):
                packet = make_raw_packet(
                    session_id=args.session_id,
                    msg_type=args.msg_type,
                    stock_locate=args.stock_locate,
                    price=args.price,
                    shares=args.shares,
                    malformed=args.malformed_length,
                    corrupt=args.corrupt_header
                )
                sock.sendto(packet, dst)
                time.sleep(1.0 / args.burst_rate)
            print("Burst phase complete, pausing...")
            time.sleep(args.pause_duration)
        print("Burst-pause test complete.")
    elif args.mode == "jitter":
        if args.test_count <= 0:
            print("JITTER mode requires --test-count > 0")
        else:
            print(f"Starting jitter test: {args.test_count} packets, base interval {args.test_interval}s, jitter ±{args.jitter}s")
            for _ in range(args.test_count):
                packet = make_raw_packet(
                    session_id=args.session_id,
                    msg_type=args.msg_type,
                    stock_locate=args.stock_locate,
                    price=args.price,
                    shares=args.shares,
                    malformed=args.malformed_length,
                    corrupt=args.corrupt_header
                )
                sock.sendto(packet, dst)
                delta = random.uniform(-args.jitter, args.jitter)
                sleep_time = max(0.0, args.test_interval + delta)
                time.sleep(sleep_time)
            print("Jitter test complete.")
    else:
        print(f"Unknown mode '{args.mode}'")

    print("All packets sent; exiting.")

if __name__ == "__main__":
    main()
