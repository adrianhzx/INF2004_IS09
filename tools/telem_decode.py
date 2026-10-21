#!/usr/bin/env python3
"""IS09 telemetry decoder / command encoder (laptop side).

Implements exactly the frame layouts of app_program/shared/app_telemetry.h
(frame version 1).  Usage:

  telem_decode.py --selftest [vectors.txt]      parity check against the C
                                                test vectors (testing/host)
  telem_decode.py --decode snap <hex>           decode one frame by hand
  telem_decode.py --encode-cmd start <seq>      print a 3-byte command frame
  telem_decode.py --mqtt host [--client is09-car]   subscribe and print
                                                (needs paho-mqtt; Phase 6)

Bit streams are LSB-first: stream bit n is bit (n % 8) of byte (n // 8).
"""

import sys

FRAME_VERSION = 1
SNAP_BYTES = 18
HB_BYTES = 6
OBS_BYTES = 10
EVT_BYTES = 4
CMD_BYTES = 3
BACKOFF_MAX = 10

STOP_CAUSES = ["NONE", "OPERATOR_STOP", "COLLISION", "STALL", "STALE_LINE",
               "STALE_MOTION", "CTRL_HEARTBEAT", "SCAN_TIMEOUT", "NO_CLEARANCE",
               "LINE_SEARCH_TIMEOUT", "CRITICAL_RUNTIME", "REMOTE_STOP(unadopted)",
               "?12", "?13", "?14", "?15"]
SYS_STATES = ["S0_BOOT_BRAKED", "S1_IDLE", "S2_LINE_FOLLOWING", "S3_EXECUTE_NAV",
              "S4_OBSTACLE_BYPASS", "S5_LINE_SEARCH", "S6_RUN_COMPLETE", "S7_SAFE_STOP"]
MOTION_STATES = ["BRAKED", "FOLLOWING", "MOVING", "TURNING", "STOP_LATCHED", "?5", "?6", "?7"]
FAULTS = ["MOTOR_STALL", "ENCODER_FAULT", "IMU_FAULT", "NO_ECHO", "WIFI_LOSS",
          "BROKER_LOSS", "IR_CONTRAST", "STALE_LINE", "STALE_MOTION", "CTRL_HEARTBEAT",
          "SCAN_TIMEOUT", "TELEM_BUF_FULL", "BARCODE_OVERFLOW", "rsv13", "rsv14", "rsv15"]
BARCODES = ["NONE", "LEFT(A)", "RIGHT(B)", "STRAIGHT(C)", "UTURN(D)", "INV_NO_MATCH",
            "INV_OVERFLOW", "?7"]
CLASSES = ["STATIONARY", "ACCELERATING", "TURNING", "CLIMBING", "DESCENDING", "IMPACT",
           "CRUISING", "?7"]
ACTIONS = ["STOP", "CONTINUE", "TURN_LEFT", "TURN_RIGHT", "REVERSE", "?5", "?6", "?7"]
CONN = ["WIFI_CONNECTING", "MQTT_CONNECTING", "ONLINE", "BACKOFF"]
EVENT_TYPES = ["NONE", "SYS_STATE", "SAFE_STOP", "FAULT_SET", "FAULT_CLR", "MOTION_DONE",
               "MOTION_REJECT", "LINE_EVENT", "BARCODE", "HUMP_START", "HUMP_END",
               "OBSTACLE", "SCAN_DONE", "REMOTE_CMD", "CMD_REJECTED", "CONN_STATE",
               "DROPPED", "BOOT", "RECOVERY", "MOTION_CLASS"]
SOURCES = ["MOTION", "CONTROLLER", "LINE", "IMU", "SCAN", "BARCODE", "COMMS", "SHARED"]

# (name, bit offset, width, signed)
SNAP_FIELDS = [
    ("ver", 0, 8, False), ("seq", 8, 8, False), ("sys", 16, 3, False),
    ("mot", 19, 3, False), ("cause", 22, 4, False), ("faults", 26, 16, False),
    ("sl", 42, 7, False), ("sr", 49, 7, False), ("el", 56, 16, False),
    ("er", 72, 16, False), ("dist", 88, 16, True), ("lerr", 104, 8, True),
    ("onl", 112, 1, False), ("bc", 113, 3, False), ("hmax", 116, 7, False),
    ("hact", 123, 1, False), ("cls", 124, 3, False), ("rng", 127, 9, False),
    ("act", 136, 3, False),
]


def bits_get(buf, off, width):
    value = 0
    for i in range(width):
        pos = off + i
        if (buf[pos // 8] >> (pos % 8)) & 1:
            value |= 1 << i
    return value


def to_signed(raw, width):
    if raw & (1 << (width - 1)):
        return raw - (1 << width)
    return raw


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def decode_snapshot(buf):
    if len(buf) != SNAP_BYTES:
        raise ValueError("snapshot must be %d bytes, got %d" % (SNAP_BYTES, len(buf)))
    out = {}
    for name, off, width, signed in SNAP_FIELDS:
        raw = bits_get(buf, off, width)
        out[name] = to_signed(raw, width) if signed else raw
    if out["ver"] != FRAME_VERSION:
        raise ValueError("unknown frame version %d" % out["ver"])
    if bits_get(buf, 139, 5) != 0:
        raise ValueError("reserved bits 139-143 not zero")
    return out


def decode_heartbeat(buf):
    if len(buf) != HB_BYTES or buf[0] != FRAME_VERSION:
        raise ValueError("bad heartbeat")
    return {"ver": buf[0], "seq": buf[1], "up": buf[2] | (buf[3] << 8),
            "conn": buf[4] & 3, "bo": (buf[4] >> 2) & 0xF, "drop": buf[5]}


def decode_obstacle(buf):
    if len(buf) != OBS_BYTES or buf[0] != FRAME_VERSION:
        raise ValueError("bad obstacle frame")
    w0 = int.from_bytes(buf[2:6], "little")
    w1 = int.from_bytes(buf[6:10], "little")
    return {"ver": buf[0], "seq": buf[1], "w0": w0, "w1": w1,
            "range_cm": w0 & 0x1FF, "angle_idx": (w0 >> 9) & 0x3F,
            "width_cm": (w0 >> 15) & 0x3F, "y_min": to_signed((w0 >> 21) & 0x7F, 7),
            "y_max": to_signed(w1 & 0x7F, 7), "closest_cm": (w1 >> 7) & 0x1FF,
            "action": (w1 >> 16) & 7, "scan_state": (w1 >> 19) & 7,
            "pseq": (w1 >> 22) & 0xF}


def decode_event(buf):
    if len(buf) != EVT_BYTES:
        raise ValueError("bad event frame")
    r = int.from_bytes(buf, "little")
    return {"type": r & 0x1F, "src": (r >> 5) & 7, "payload": (r >> 8) & 0xFFFF,
            "stamp": (r >> 24) & 0xFF}


def encode_cmd(cmd, seq):
    codes = {"start": 1, "stop": 2}
    if cmd not in codes or not 0 <= seq <= 63:
        raise ValueError("cmd must be start|stop, seq 0..63")
    frame = bytes([FRAME_VERSION, (codes[cmd] << 6) | seq])
    return frame + bytes([crc8(frame)])


def decode_cmd(buf):
    if len(buf) != CMD_BYTES:
        return "BAD_LENGTH"
    if buf[0] != FRAME_VERSION:
        return "BAD_VERSION"
    if crc8(buf[:2]) != buf[2]:
        return "BAD_CRC"
    code = buf[1] >> 6
    if code not in (1, 2):
        return "BAD_TYPE"
    return {"cmd": code, "seq": buf[1] & 0x3F}


def pretty_snapshot(s):
    faults = [FAULTS[i] for i in range(16) if s["faults"] & (1 << i)]
    return ("SNAP seq=%d %s %s cause=%s faults=[%s] speed=%d/%d cm/s enc=%d/%d dist=%d mm "
            "line_err=%d on_line=%d barcode=%s hump_max=%d mm hump_active=%d class=%s "
            "range=%s action=%s" % (
                s["seq"], SYS_STATES[s["sys"]], MOTION_STATES[s["mot"]], STOP_CAUSES[s["cause"]],
                ",".join(faults), s["sl"], s["sr"], s["el"], s["er"], s["dist"], s["lerr"],
                s["onl"], BARCODES[s["bc"]], s["hmax"], s["hact"], CLASSES[s["cls"]],
                ("no-echo/>=511" if s["rng"] == 511 else "%d cm" % s["rng"]), ACTIONS[s["act"]]))


def pretty_event(e):
    t = EVENT_TYPES[e["type"]] if e["type"] < len(EVENT_TYPES) else "?%d" % e["type"]
    return "EVT %s from %s payload=0x%04x t=%d ms(mod 256)" % (
        t, SOURCES[e["src"]], e["payload"], e["stamp"])


def selftest(path):
    """Parity check: decode each C vector and compare every field."""
    ok = 0
    bad = 0
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) < 2:
                continue
            tag, hexs, fields = parts[0], parts[1], parts[2:]
            buf = bytes.fromhex(hexs)
            expect = dict(kv.split("=") for kv in fields)
            expect = {k: int(v) for k, v in expect.items()}
            if tag == "SNAP":
                got = decode_snapshot(buf)
            elif tag == "HB":
                got = decode_heartbeat(buf)
            elif tag == "OBS":
                got = decode_obstacle(buf)
            elif tag == "EVT":
                got = decode_event(buf)
            elif tag == "CMD":
                got = decode_cmd(buf)
                if encode_cmd("start" if got["cmd"] == 1 else "stop", got["seq"]) != buf:
                    bad += 1
                    print("FAIL CMD re-encode mismatch", hexs)
                    continue
            else:
                continue
            mism = {k: (expect[k], got.get(k)) for k in expect if got.get(k) != expect[k]}
            if mism:
                bad += 1
                print("FAIL", tag, hexs, mism)
            else:
                ok += 1
    print("telem_decode selftest: %d vectors ok, %d failed" % (ok, bad))
    return 0 if bad == 0 else 1


def main(argv):
    if len(argv) >= 2 and argv[1] == "--selftest":
        return selftest(argv[2] if len(argv) > 2 else "testing/host/vectors.txt")
    if len(argv) >= 4 and argv[1] == "--decode":
        kind, buf = argv[2], bytes.fromhex(argv[3])
        if kind == "snap":
            print(pretty_snapshot(decode_snapshot(buf)))
        elif kind == "hb":
            h = decode_heartbeat(buf)
            print("HB seq=%d uptime=%d s %s backoff=%d s dropped=%d" % (
                h["seq"], h["up"], CONN[h["conn"]], h["bo"], h["drop"]))
        elif kind == "obs":
            print(decode_obstacle(buf))
        elif kind == "evt":
            print(pretty_event(decode_event(buf)))
        elif kind == "cmd":
            print(decode_cmd(buf))
        return 0
    if len(argv) >= 4 and argv[1] == "--encode-cmd":
        print(encode_cmd(argv[2], int(argv[3])).hex())
        return 0
    if len(argv) >= 3 and argv[1] == "--mqtt":
        return mqtt_main(argv[2], argv[3] if len(argv) > 3 else "is09-car")
    print(__doc__)
    return 2


def mqtt_main(host, client):
    try:
        import paho.mqtt.client as mqtt  # noqa
    except ImportError:
        print("paho-mqtt not installed: pip install paho-mqtt")
        return 1
    base = "is09/%s/" % client

    def on_msg(_c, _u, m):
        kind = m.topic.rsplit("/", 1)[-1]
        try:
            if kind == "snap":
                print(pretty_snapshot(decode_snapshot(m.payload)))
            elif kind == "evt":
                print(pretty_event(decode_event(m.payload)))
            elif kind == "hb":
                h = decode_heartbeat(m.payload)
                print("HB seq=%d uptime=%d s %s backoff=%d dropped=%d" % (
                    h["seq"], h["up"], CONN[h["conn"]], h["bo"], h["drop"]))
            elif kind == "obs":
                print("OBS", decode_obstacle(m.payload))
        except ValueError as exc:
            print("undecodable %s (%d B): %s" % (m.topic, len(m.payload), exc))

    c = mqtt.Client()
    c.on_message = on_msg
    c.connect(host, 1883, 60)
    for t in ("snap", "evt", "hb", "obs"):
        c.subscribe(base + t)
    print("subscribed to", base + "{snap,evt,hb,obs}; Ctrl-C to stop")
    c.loop_forever()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
