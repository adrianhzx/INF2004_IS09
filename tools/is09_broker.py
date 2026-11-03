"""Minimal MQTT 3.1.1 broker for the IS09 car that prints DECODED telemetry.

Based on mqtt_broker.py (a friend's minimal broker; that file is unchanged).
The broker part is the same: CONNECT, PUBLISH (QoS 0/1), SUBSCRIBE,
UNSUBSCRIBE, PINGREQ and DISCONNECT, and every PUBLISH is forwarded at QoS 0
to clients whose filter matches ('+' and '#'), so tools/telem_bench.py and
command senders keep working against it.

What is different: instead of the raw payload, each frame on is09/<car>/...
is decoded with tools/telem_decode.py (the laptop-side copy of the frame
layouts in app_program/shared/app_telemetry.h) and printed as one line:

    10:24:32.226 SNAP seq=22 S2_LINE_FOLLOWING FOLLOWING cause=NONE ...
    10:24:32.231 HB   seq=56 up=57 s ONLINE backoff=0 s dropped=0
    10:24:32.640 GAP  snap seq 24 -> 26 (1 missed)

Frames that do not decode are printed as UNDECODABLE with their hex bytes.
Other topics are printed as text when printable, otherwise as hex.

Run it from anywhere; it finds telem_decode.py in its own folder:

    python -I tools\\is09_broker.py [--log path\\to\\telemetry.log] [--secs 600]
"""
import argparse
import asyncio
import datetime
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import telem_decode as td  # noqa: E402  (same folder; works with python -I)

SEQ_MOD = 256  # snapshot / heartbeat / obstacle seq are 8-bit

clients = {}   # writer -> list of topic filters
last_seq = {}  # (topic, kind) -> last seq seen, for gap detection
log_file = None


def log(line):
    stamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    text = f"{stamp} {line}"
    print(text, flush=True)
    if log_file:
        log_file.write(text + "\n")
        log_file.flush()


def matches(filt, topic):
    f, t = filt.split("/"), topic.split("/")
    for i, part in enumerate(f):
        if part == "#":
            return True
        if i >= len(t) or (part != "+" and part != t[i]):
            return False
    return len(f) == len(t)


def encode_len(n):
    out = bytearray()
    while True:
        b, n = n % 128, n // 128
        out.append(b | (0x80 if n else 0))
        if not n:
            return bytes(out)


def utf8(b, i):
    n = int.from_bytes(b[i:i + 2], "big")
    return b[i + 2:i + 2 + n].decode("utf-8", "replace"), i + 2 + n


async def read_packet(reader):
    head = await reader.readexactly(1)
    mult, length = 1, 0
    while True:
        b = (await reader.readexactly(1))[0]
        length += (b & 0x7F) * mult
        if not b & 0x80:
            break
        mult *= 128
    body = await reader.readexactly(length) if length else b""
    return head[0], body


# ---- decoding ---------------------------------------------------------------
def gap_note(topic, kind, seq):
    """Return a GAP line if seq did not follow the previous one, else None."""
    key = (topic, kind)
    prev = last_seq.get(key)
    last_seq[key] = seq
    if prev is None:
        return None
    expected = (prev + 1) % SEQ_MOD
    if seq == expected:
        return None
    missed = (seq - expected) % SEQ_MOD
    return f"GAP  {kind} seq {prev} -> {seq} ({missed} missed or out of order)"


def describe(topic, payload):
    """Decode one PUBLISH payload into (line, gap_line_or_None)."""
    parts = topic.split("/")
    kind = parts[-1] if len(parts) == 3 and parts[0] == "is09" else None
    try:
        if kind == "snap":
            s = td.decode_snapshot(payload)
            return td.pretty_snapshot(s), gap_note(topic, "snap", s["seq"])
        if kind == "hb":
            h = td.decode_heartbeat(payload)
            line = "HB   seq=%d up=%d s %s backoff=%d s dropped=%d" % (
                h["seq"], h["up"], td.CONN[h["conn"]], h["bo"], h["drop"])
            return line, gap_note(topic, "hb", h["seq"])
        if kind == "obs":
            o = td.decode_obstacle(payload)
            line = ("OBS  seq=%d range=%d cm width=%d cm y=%d..%d closest=%d cm "
                    "action=%s scan_state=%d" % (
                        o["seq"], o["range_cm"], o["width_cm"], o["y_min"],
                        o["y_max"], o["closest_cm"], td.ACTIONS[o["action"]],
                        o["scan_state"]))
            return line, gap_note(topic, "obs", o["seq"])
        if kind == "evt":
            return td.pretty_event(td.decode_event(payload)), None
        if kind == "cmd":
            c = td.decode_cmd(payload)
            if isinstance(c, str):
                return f"CMD  rejected by decoder: {c} ({payload.hex()})", None
            name = {1: "START", 2: "STOP"}[c["cmd"]]
            return f"CMD  {name} seq={c['seq']}", None
    except (ValueError, IndexError, KeyError) as exc:
        return f"UNDECODABLE {topic} ({len(payload)} B): {exc} [{payload.hex()}]", None
    # any other topic (for example the template's text heartbeat)
    text = payload.decode("utf-8", "replace")
    if payload and all(32 <= b < 127 for b in payload):
        return f"{topic} {text}", None
    return f"{topic} ({len(payload)} B) {payload.hex()}", None


# ---- broker -----------------------------------------------------------------
async def forward(topic, payload, sender):
    tb = topic.encode()
    pkt_body = len(tb).to_bytes(2, "big") + tb + payload
    pkt = bytes([0x30]) + encode_len(len(pkt_body)) + pkt_body
    for w, filters in list(clients.items()):
        if w is not sender and any(matches(f, topic) for f in filters):
            try:
                w.write(pkt)
                await w.drain()
            except (ConnectionError, OSError):
                pass


async def handle(reader, writer):
    peer = writer.get_extra_info("peername")
    clients[writer] = []
    log(f"[broker] connection from {peer[0]}:{peer[1]}")
    try:
        while True:
            first, body = await read_packet(reader)
            ptype = first >> 4
            if ptype == 1:                                   # CONNECT
                cid = "?"
                try:
                    _, i = utf8(body, 0)                     # "MQTT"
                    cid, _ = utf8(body, i + 4)               # level, flags, keepalive
                except Exception:
                    pass
                log(f"[broker] CONNECT client_id={cid}")
                writer.write(b"\x20\x02\x00\x00")
            elif ptype == 3:                                 # PUBLISH
                qos = (first >> 1) & 3
                topic, i = utf8(body, 0)
                if qos:
                    pid = body[i:i + 2]
                    i += 2
                    writer.write(b"\x40\x02" + pid)          # PUBACK
                payload = body[i:]
                line, gap = describe(topic, payload)
                if gap:
                    log(gap)
                log(line)
                await forward(topic, payload, writer)
            elif ptype == 8:                                 # SUBSCRIBE
                pid, i, granted = body[0:2], 2, bytearray()
                while i < len(body):
                    filt, i = utf8(body, i)
                    i += 1
                    clients[writer].append(filt)
                    granted.append(0)
                    log(f"[broker] SUBSCRIBE {filt}")
                writer.write(bytes([0x90]) + encode_len(2 + len(granted)) + pid + granted)
            elif ptype == 10:                                # UNSUBSCRIBE
                writer.write(b"\xb0\x02" + body[0:2])
            elif ptype == 12:                                # PINGREQ
                writer.write(b"\xd0\x00")
            elif ptype == 14:                                # DISCONNECT
                break
            await writer.drain()
    except (asyncio.IncompleteReadError, ConnectionError, OSError):
        pass
    finally:
        clients.pop(writer, None)
        log(f"[broker] {peer[0]} disconnected")
        writer.close()


async def main(args):
    server = await asyncio.start_server(handle, "0.0.0.0", args.port)
    log(f"[broker] listening on 0.0.0.0:{args.port} (decoded telemetry)")
    async with server:
        if args.secs > 0:
            await asyncio.sleep(args.secs)
            log("[broker] time limit reached, stopping")
        else:
            await server.serve_forever()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--log", default=None)
    ap.add_argument("--secs", type=int, default=0, help="stop after N s (0 = run forever)")
    a = ap.parse_args()
    if a.log:
        log_file = open(a.log, "a", encoding="utf-8")
    try:
        asyncio.run(main(a))
    except KeyboardInterrupt:
        sys.exit(0)
