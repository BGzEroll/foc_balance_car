"""TCP commands + TCP/UDP telemetry benchmark; never enables the motors."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import select
import socket
import struct
import time

WIRE = struct.Struct("<IIQQ15f7I")
MAGIC = b"BCW1"
FIELDS = ["magic", "sequence", "sent_us", "imu_us", "pitch", "roll", "yaw",
          "gx", "gy", "gz", "speed_a", "speed_b", "iq_a", "iq_b", "id_a", "id_b",
          "target_a", "target_b", "bus", "updates", "max_dt_us", "max_calculate_us",
          "active", "fault", "current_age_a_us", "current_age_b_us"]

# BCW2: 16-byte transport header plus packed control-cycle payload.
WIRE2 = struct.Struct("<IIQQQ13I34f")
FIELDS2 = ["magic","sequence","sent_us","sample_us","imu_us","cycle","dt_us","revision",
 "mode","result","fault","flags","saturated","imu_age_us","encoder_age_us",
 "current_age_a_us","current_age_b_us","calculate_us",
 "pitch","roll","pitch_rate","yaw","yaw_rate","speed","distance","target_speed","target_pitch",
 "command_a","command_b","iq_a","iq_b","id_a","id_b","raw_a","raw_b","angle_a","angle_b","bus","current_limit_A","velocity_integral","pitch_integral","yaw_integral",
 "velocity_kp","velocity_ki","pitch_kd","pitch_kp","pitch_ki","yaw_kp","yaw_ki","yaw_kd","position_kp","pitch_trim_rad"]
assert WIRE2.size == 220 and len(WIRE2.unpack(bytes(220))) == len(FIELDS2)


def percentile(values, p):
    if not values:
        return None
    s = sorted(values)
    return s[round((len(s) - 1) * p / 100)]


class Client:
    def __init__(self, host):
        self.host = socket.gethostbyname(host)
        self.tcp = socket.create_connection((self.host, 3333), timeout=3)
        self.tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.tcp.setblocking(False)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        self.udp.bind(("0.0.0.0", 0))
        self.udp.setblocking(False)
        # Outbound probe establishes the Windows firewall/NAT reply path.
        self.udp.sendto(b"HELLO", (self.host, 3334))
        self.buffer = bytearray()
        self.lines, self.records, self.rtts = [], [], []
        self.pings = {}
        self.datagrams = 0

    def send(self, command):
        self.tcp.sendall((command + "\n").encode("ascii"))

    def record(self, data, received):
        wire = WIRE2 if data[:4] == b"BCW2" else WIRE
        if len(data) != wire.size or data[:4] not in (MAGIC,b"BCW2"):
            raise ValueError("Bad telemetry frame")
        self.records.append((received, wire.unpack(data)))

    def pump(self, timeout=.05):
        ready, _, _ = select.select([self.tcp, self.udp], [], [], timeout)
        for sock in ready:
            received = time.perf_counter()
            if sock is self.udp:
                while True:
                    try:
                        data, sender = sock.recvfrom(4096)
                    except BlockingIOError:
                        break
                    if sender[0] == self.host:
                        if data[:4] == b"BCW2":
                            self.datagrams += 1; self.record(data,time.perf_counter()); continue
                        if len(data) % WIRE.size: raise ValueError("Bad datagram size")
                        self.datagrams += 1
                        for offset in range(0, len(data), WIRE.size):
                            self.record(data[offset:offset+WIRE.size], time.perf_counter())
            else:
                data = sock.recv(65536)
                if not data:
                    raise ConnectionError("Car closed TCP connection")
                self.buffer.extend(data)
                while self.buffer:
                    if self.buffer[:4] in (MAGIC,b"BCW2"):
                        size = WIRE2.size if self.buffer[:4] == b"BCW2" else WIRE.size
                        if len(self.buffer) < size:
                            break
                        self.record(self.buffer[:size], received)
                        del self.buffer[:size]
                    else:
                        end = self.buffer.find(b"\n")
                        if end < 0:
                            break
                        line = self.buffer[:end].decode("ascii").strip()
                        del self.buffer[:end + 1]
                        self.lines.append(line)
                        if line.startswith("PONG "):
                            token = line[5:]
                            began = self.pings.pop(token, None)
                            if began is not None:
                                self.rtts.append((received - began) * 1000)

    def command(self, text, prefix=None, timeout=3):
        verb = text.split()[0]
        prefix = prefix or {"TUNEA": "GAINS", "SET":"GAINS", "GAINSSET":"GAINS", "BALANCE": "RUN", "SHADOW": "RUN",
                            "PROBE": "RUN"}.get(verb, verb)
        before = len(self.lines)
        self.send(text)
        end = time.perf_counter() + timeout
        while time.perf_counter() < end:
            self.pump()
            for line in self.lines[before:]:
                if line.startswith(("REJECT", "BUSY")):
                    raise RuntimeError(line)
                if line.startswith(prefix):
                    return line
        raise TimeoutError(f"No reply to {text}")

    def close(self):
        self.tcp.close()
        self.udp.close()


def bench(host, transport, hz, seconds, output):
    c = Client(host)
    try:
        status = c.command("STATUS")
        if "running=0" not in status or "active=0" not in status:
            raise RuntimeError("Benchmark requires motors stopped; will not interrupt running balance")
        c.command("SCHEDRESET")
        c.command(f"STREAM {transport} {hz} {c.udp.getsockname()[1]}", "STREAM")
        began = time.perf_counter()
        next_ping = began
        end = began + seconds
        while time.perf_counter() < end:
            now = time.perf_counter()
            if now >= next_ping:
                token = str(time.perf_counter_ns())
                c.pings[token] = now
                c.send("PING " + token)
                next_ping = now + .1
            c.pump(min(.02, max(0, end - now)))
        net = c.command("NET")
        c.command(f"STREAM {transport} 0", "STREAM")
        until = time.perf_counter() + .3
        while time.perf_counter() < until:
            c.pump(.02)
        final_status = c.command("STATUS")
        records = [(t, p) for t, p in c.records if began + 1 <= t <= end]
        if len(records) < 2:
            raise RuntimeError("Not enough telemetry received")
        seqs = [p[1] for _, p in records]
        unique = set(seqs)
        span = max(unique) - min(unique) + 1
        arrivals = [(b[0] - a[0]) * 1000 for a, b in zip(records, records[1:])]
        sent_span = (records[-1][1][2] - records[0][1][2]) / 1e6
        # No clock synchronization assumed: report excess latency above the
        # fastest observed sample, not an absolute one-way latency.
        offsets = [t * 1000 - p[2] / 1000 for t, p in records]
        floor = min(offsets)
        excess = [x - floor for x in offsets]
        control_hz = (records[-1][1][19] - records[0][1][19]) / sent_span
        report = dict(timestamp_utc=datetime.now(timezone.utc).isoformat(), host=host,
                      transport=transport, requested_hz=hz, duration_s=seconds, batch=1,
                      datagrams_received=c.datagrams,
                      warmup_s=1, bytes_per_packet=WIRE.size, received=len(records),
                      received_hz=len(records) / (seconds - 1),
                      received_datagrams_hz=len({p[1] for _, p in records}) / (seconds - 1) if transport == "UDP" else None,
                      board_attempt_hz=(span - 1) / sent_span,
                      sequence_loss_percent=100 * (span - len(unique)) / span,
                      duplicates=len(seqs) - len(unique),
                      reordered=sum(b < a for a, b in zip(seqs, seqs[1:])),
                      unique_imu_hz=len({p[3] for _, p in records}) / (seconds - 1),
                      arrival_gap_p95_ms=percentile(arrivals, 95),
                      arrival_gap_p99_ms=percentile(arrivals, 99), max_gap_ms=max(arrivals),
                      excess_latency_p95_ms=percentile(excess, 95),
                      excess_latency_p99_ms=percentile(excess, 99),
                      ping_rtt_p50_ms=percentile(c.rtts, 50),
                      ping_rtt_p95_ms=percentile(c.rtts, 95),
                      ping_rtt_max_ms=max(c.rtts) if c.rtts else None,
                      ping_lost=len(c.pings), control_task_hz=control_hz,
                      control_max_dt_us=max(p[20] for _, p in records),
                      nonzero_fault_samples=sum(p[23] != 0 for _, p in records),
                      nonzero_active_samples=sum(p[22] != 0 for _, p in records),
                      status_before=status, status_after=final_status, net=net)
        output.mkdir(parents=True, exist_ok=True)
        name = f"{transport.lower()}_{hz}_{time.time_ns()}"
        (output / (name + ".json")).write_text(json.dumps(report, indent=2), encoding="utf-8")
        with (output / (name + ".jsonl")).open("w", encoding="utf-8") as f:
            for t, p in c.records:
                f.write(json.dumps(dict(received_monotonic=t, **dict(zip(FIELDS, p)))) + "\n")
        print(json.dumps(report), flush=True)
        return report
    except Exception as error:
        output.mkdir(parents=True, exist_ok=True)
        report = dict(transport=transport, requested_hz=hz, duration_s=seconds, batch=1,
                      datagrams_received=c.datagrams,
                      error=f"{type(error).__name__}: {error}", received_total=len(c.records))
        name = f"failed_{transport.lower()}_{hz}_{time.time_ns()}"
        (output / (name + ".json")).write_text(json.dumps(report, indent=2), encoding="utf-8")
        with (output / (name + ".jsonl")).open("w", encoding="utf-8") as f:
            for t, p in c.records:
                f.write(json.dumps(dict(received_monotonic=t, **dict(zip(FIELDS, p)))) + "\n")
        print(json.dumps(report), flush=True)
        return report
    finally:
        c.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host")
    parser.add_argument("--command", help="TCP command (motor commands require deliberate user action)")
    parser.add_argument("--rates", default="200")
    parser.add_argument("--transports", default="UDP")
    parser.add_argument("--seconds", type=float, default=15)
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "results")
    args = parser.parse_args()
    if args.seconds <= 1:
        parser.error("--seconds must exceed the one-second warmup")
    if args.command:
        c = Client(args.host)
        try:
            print(c.command(args.command))
        finally:
            c.close()
        return
    reports = []
    # Alternate transports at each rate to reduce environmental ordering bias.
    for hz in map(int, args.rates.split(",")):
        for transport in args.transports.split(","):
            reports.append(bench(args.host, transport, hz, args.seconds, args.output))
    (args.output / "summary.json").write_text(json.dumps(reports, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
