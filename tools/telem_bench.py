#!/usr/bin/env python3
"""IS09 telemetry benchmark and command sender (laptop side, Buddy 1).

Standard library only (no paho-mqtt). Subscribes to is09/<client>/#, decodes
every frame with telem_decode.py and reports rates, sizes and integrity
against the subsystem 1 pass criteria (testing/subsystem1/TEST_LOG.md).

  # B1/B2: listen 60 s and check rates, decoding and byte budget
  python3 tools/telem_bench.py --host 10.143.10.211 --secs 60

  # B5: send commands, then watch the car's response for 3 s
  python3 tools/telem_bench.py --host 10.143.10.211 --send start 12
  python3 tools/telem_bench.py --host 10.143.10.211 --send start 12  # replay
  python3 tools/telem_bench.py --host 10.143.10.211 --send stop 13
  python3 tools/telem_bench.py --host 10.143.10.211 --send start 14 --bad-crc
  python3 tools/telem_bench.py --host 10.143.10.211 --send start 15 --len 5
  python3 tools/telem_bench.py --host 10.143.10.211 --send start 16 --retain

--retain only means something on a broker that stores retained messages
(Mosquitto); the minimal mqtt_broker.py ignores it.
Exit status: 0 = all checks passed, 1 = a check failed, 2 = no connection.
"""

import argparse
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import telem_decode as td  # noqa: E402  (same folder)

VERDICTS = ["OK", "BAD_LENGTH", "BAD_VERSION", "BAD_CRC", "BAD_TYPE", "REPLAY",
            "RETAINED", "START_GUARD", "START_STATE"]

# Pass criteria (snapshot 200 ms, heartbeat 1 s; Week 6 NFR6).
SNAP_HZ, SNAP_TOL = 5.0, 0.1
HB_HZ, HB_TOL = 1.0, 0.05


# ---- minimal MQTT 3.1.1 client ---------------------------------------------
def _enc_len(n):
    out = bytearray()
    while True:
        b, n = n % 128, n // 128
        out.append(b | (0x80 if n else 0))
        if not n:
            return bytes(out)


def _utf8(s):
    b = s.encode()
    return struct.pack(">H", len(b)) + b


class Mqtt:
    def __init__(self, host, port, client_id):
        self.sock = socket.create_connection((host, port), timeout=5)
        body = _utf8("MQTT") + bytes([4, 0x02]) + struct.pack(">H", 60) + _utf8(client_id)
        self.sock.sendall(bytes([0x10]) + _enc_len(len(body)) + body)
        head, payload = self._read()
        if head >> 4 != 2 or len(payload) < 2 or payload[1] != 0:
            raise ConnectionError("CONNACK refused: %r" % payload)
        self.sock.settimeout(0.2)
        self.last_ping = time.monotonic()

    def _readn(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("broker closed the connection")
            buf += chunk
        return buf

    def _read(self):
        head = self._readn(1)[0]
        mult, length = 1, 0
        while True:
            b = self._readn(1)[0]
            length += (b & 0x7F) * mult
            if not b & 0x80:
                break
            mult *= 128
        return head, (self._readn(length) if length else b"")

    def subscribe(self, topic):
        body = struct.pack(">H", 1) + _utf8(topic) + b"\x00"
        self.sock.sendall(bytes([0x82]) + _enc_len(len(body)) + body)

    def publish(self, topic, payload, retain=False):
        body = _utf8(topic) + payload
        self.sock.sendall(bytes([0x30 | (1 if retain else 0)]) + _enc_len(len(body)) + body)

    def poll(self):
        """Return (topic, payload) or None within ~0.2 s."""
        if time.monotonic() - self.last_ping > 30:
            self.sock.sendall(b"\xc0\x00")
            self.last_ping = time.monotonic()
        try:
            head, body = self._read()
        except socket.timeout:
            return None
        if head >> 4 != 3:
            return None
        n = struct.unpack(">H", body[:2])[0]
        topic = body[2:2 + n].decode("utf-8", "replace")
        off = 2 + n + (2 if (head >> 1) & 3 else 0)
        return topic, body[off:]

    def close(self):
        try:
            self.sock.sendall(b"\xe0\x00")
            self.sock.close()
        except OSError:
            pass


# ---- statistics -------------------------------------------------------------
class Stats:
    def __init__(self):
        self.count = {"snap": 0, "evt": 0, "hb": 0, "obs": 0}
        self.bytes = {"snap": 0, "evt": 0, "hb": 0, "obs": 0}
        self.decode_errors = 0
        self.snap_gaps = 0
        self.hb_gaps = 0
        self.last_snap_seq = None
        self.last_hb_seq = None
        self.last_hb = None
        self.last_snap = None
        self.event_types = {}
        self.rejects = []

    def add(self, kind, payload, verbose):
        if kind not in self.count:
            return
        self.count[kind] += 1
        self.bytes[kind] += len(payload)
        try:
            if kind == "snap":
                s = td.decode_snapshot(payload)
                if self.last_snap_seq is not None and s["seq"] != (self.last_snap_seq + 1) & 0xFF:
                    self.snap_gaps += 1
                self.last_snap_seq, self.last_snap = s["seq"], s
                line = td.pretty_snapshot(s)
            elif kind == "hb":
                h = td.decode_heartbeat(payload)
                if self.last_hb_seq is not None and h["seq"] != (self.last_hb_seq + 1) & 0xFF:
                    self.hb_gaps += 1
                self.last_hb_seq, self.last_hb = h["seq"], h
                line = "HB seq=%d up=%d s conn=%s backoff=%d s dropped=%d" % (
                    h["seq"], h["up"], td.CONN[h["conn"]], h["bo"], h["drop"])
            elif kind == "evt":
                e = td.decode_event(payload)
                name = td.EVENT_TYPES[e["type"]] if e["type"] < len(td.EVENT_TYPES) else "?"
                self.event_types[name] = self.event_types.get(name, 0) + 1
                line = td.pretty_event(e)
                if name == "CMD_REJECTED":
                    reason = e["payload"]
                    if e["src"] == 1:  # controller
                        why = ("not armed (APP_CAL_MOTION_ARMED=0)" if reason == 0x0001
                               else "START not in S1")
                    else:
                        why = VERDICTS[reason] if reason < len(VERDICTS) else "?"
                    self.rejects.append(why)
                    line += "  -> " + why
            else:
                o = td.decode_obstacle(payload)
                line = "OBS seq=%d range=%d cm width=%d cm action=%s" % (
                    o["seq"], o["range_cm"], o["width_cm"], td.ACTIONS[o["action"]])
        except (ValueError, IndexError) as exc:
            self.decode_errors += 1
            line = "UNDECODABLE %s (%d B): %s" % (kind, len(payload), exc)
        if verbose:
            print(time.strftime("%H:%M:%S"), line)


def listen(m, base, secs, verbose):
    st = Stats()
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        msg = m.poll()
        if msg is None:
            continue
        topic, payload = msg
        if topic.startswith(base):
            st.add(topic[len(base):], payload, verbose)
    return st, time.monotonic() - t0


def report(st, secs):
    ok = True
    print("\n=== %.1f s on %s ===" % (secs, time.strftime("%Y-%m-%d %H:%M:%S")))
    for k in ("snap", "hb", "evt", "obs"):
        print("%-4s %5d frames  %6.2f Hz  %6d B  %7.1f B/s" % (
            k, st.count[k], st.count[k] / secs, st.bytes[k], st.bytes[k] / secs))
    total = sum(st.bytes.values())
    print("payload total %d B = %.1f B/s (MQTT/TCP/IP headers not included)" % (
        total, total / secs))
    if st.event_types:
        print("events: " + ", ".join("%s x%d" % kv for kv in sorted(st.event_types.items())))
    if st.last_hb:
        print("last heartbeat: up %d s, conn %s, dropped %d" % (
            st.last_hb["up"], td.CONN[st.last_hb["conn"]], st.last_hb["drop"]))

    def check(name, cond, detail):
        nonlocal ok
        ok = ok and cond
        print("[%s] %s  (%s)" % ("PASS" if cond else "FAIL", name, detail))

    print()
    check("B1a snapshot rate 5.0 +-0.1 Hz", abs(st.count["snap"] / secs - SNAP_HZ) <= SNAP_TOL,
          "%.2f Hz" % (st.count["snap"] / secs))
    check("B1b heartbeat rate 1.0 +-0.05 Hz", abs(st.count["hb"] / secs - HB_HZ) <= HB_TOL,
          "%.2f Hz" % (st.count["hb"] / secs))
    check("B1c every frame decodes", st.decode_errors == 0, "%d errors" % st.decode_errors)
    check("B1d no snapshot or heartbeat lost", st.snap_gaps == 0 and st.hb_gaps == 0,
          "%d snapshot gaps, %d heartbeat gaps" % (st.snap_gaps, st.hb_gaps))
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", required=True)
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--client", default="is09-car", help="topic client segment")
    ap.add_argument("--secs", type=float, default=60.0)
    ap.add_argument("--verbose", "-v", action="store_true")
    ap.add_argument("--send", nargs=2, metavar=("start|stop", "SEQ"))
    ap.add_argument("--bad-crc", action="store_true")
    ap.add_argument("--len", type=int, default=None, help="send a frame of this length")
    ap.add_argument("--retain", action="store_true")
    a = ap.parse_args()

    base = "is09/%s/" % a.client
    try:
        m = Mqtt(a.host, a.port, "is09-bench-%d" % os.getpid())
    except (OSError, ConnectionError) as exc:
        print("cannot connect to %s:%d: %s" % (a.host, a.port, exc))
        return 2
    m.subscribe(base + "#")
    try:
        if a.send:
            frame = bytearray(td.encode_cmd(a.send[0], int(a.send[1])))
            if a.bad_crc:
                frame[2] ^= 0x5A
            if a.len is not None:
                frame = (frame + bytes(8))[:a.len]
            m.publish(base + "cmd", bytes(frame), retain=a.retain)
            print("sent %s on %scmd%s; watching 3 s for the response:" % (
                bytes(frame).hex(), base, " (retained)" if a.retain else ""))
            st, _ = listen(m, base, 3.0, False)
            for name in ("REMOTE_CMD", "CMD_REJECTED", "SAFE_STOP", "SYS_STATE"):
                if name in st.event_types:
                    print("  %s x%d" % (name, st.event_types[name]))
            for why in st.rejects:
                print("  rejected: " + why)
            if st.last_snap:
                print("  last snapshot: %s, stop cause %s" % (
                    td.SYS_STATES[st.last_snap["sys"]], td.STOP_CAUSES[st.last_snap["cause"]]))
            return 0
        st, secs = listen(m, base, a.secs, a.verbose)
        return 0 if report(st, secs) else 1
    finally:
        m.close()


if __name__ == "__main__":
    sys.exit(main())
