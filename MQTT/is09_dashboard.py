"""Live dashboard for the IS09 car running this software.

One program does everything, with the Python standard library only:
  - an MQTT 3.1.1 broker on port 1883 (the car connects to it as before;
    stop is09_broker.py / Mosquitto first, they use the same port);
  - a web page on http://localhost:8080 that shows every telemetry frame,
    event and reply live, with charts, and sends commands to the car;
  - a log file of every message (logs/dashboard_<date>_<time>.log), in the
    same "time topic payload" format as before, to attach when asking for
    help;
  - the UDP broker beacon ("IS09_BROKER 1883" on port 47002, as
    telemetry_monitor.py), so the car finds this laptop if its IP changed.

Run (Windows):   py is09_dashboard.py
Options:         --http-port 8080 --mqtt-port 1883 --no-browser --no-beacon
                 --quiet (do not print every message in this window)
"""
import argparse
import asyncio
import collections
import datetime
import json
import os
import socket
import sys
import threading
import time
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(HERE, "is09_dashboard.html")
HISTORY = 3000          # messages replayed to a page that (re)connects

clients = {}            # MQTT writer -> list of topic filters
retained = {}           # topic -> payload bytes
history = collections.deque(maxlen=HISTORY)
sse_queues = set()
log_file = None
quiet = False
stats = {"robot_seen": 0.0, "robot_peer": "", "msgs": 0}


def now_text():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def log(line, echo=True):
    text = f"{now_text()} {line}"
    if echo and not quiet:
        print(text, flush=True)
    if log_file:
        log_file.write(text + "\n")
        log_file.flush()


# ---- MQTT broker (same packet handling as tools/is09_broker.py) ---------------
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


def publish_packet(topic, payload, retain=False):
    tb = topic.encode()
    body = len(tb).to_bytes(2, "big") + tb + payload
    return bytes([0x31 if retain else 0x30]) + encode_len(len(body)) + body


async def forward(topic, payload, sender=None):
    pkt = publish_packet(topic, payload)
    for w, filters in list(clients.items()):
        if w is not sender and any(matches(f, topic) for f in filters):
            try:
                w.write(pkt)
                await w.drain()
            except (ConnectionError, OSError):
                pass


def to_page(kind, topic, text):
    """Record a message and hand it to every open page."""
    msg = {"ts": time.time() * 1000.0, "kind": kind, "topic": topic, "payload": text}
    history.append(msg)
    data = json.dumps(msg)
    for q in list(sse_queues):
        if q.qsize() < 2000:
            q.put_nowait(data)


async def mqtt_handle(reader, writer):
    peer = writer.get_extra_info("peername")
    clients[writer] = []
    log(f"[broker] connection from {peer[0]}:{peer[1]}")
    to_page("sys", "broker", f"connection from {peer[0]}")
    try:
        while True:
            first, body = await read_packet(reader)
            ptype = first >> 4
            if ptype == 1:                                   # CONNECT
                cid = "?"
                try:
                    _, i = utf8(body, 0)
                    cid, _ = utf8(body, i + 4)
                except Exception:
                    pass
                log(f"[broker] CONNECT client_id={cid}")
                to_page("sys", "broker", f"CONNECT {cid} from {peer[0]}")
                writer.write(b"\x20\x02\x00\x00")
            elif ptype == 3:                                 # PUBLISH
                qos = (first >> 1) & 3
                retain = bool(first & 1)
                topic, i = utf8(body, 0)
                if qos:
                    pid = body[i:i + 2]
                    i += 2
                    writer.write(b"\x40\x02" + pid)          # PUBACK
                payload = body[i:]
                if retain:
                    retained[topic] = payload
                text = payload.decode("utf-8", "replace")
                if topic.startswith("is09/"):
                    stats["robot_seen"] = time.time()
                    stats["robot_peer"] = peer[0]
                stats["msgs"] += 1
                log(f"{topic} {text}", echo=not topic.endswith("telemetry"))
                to_page("mqtt", topic, text)
                await forward(topic, payload, writer)
            elif ptype == 8:                                 # SUBSCRIBE
                pid, i, granted = body[0:2], 2, bytearray()
                new = []
                while i < len(body):
                    filt, i = utf8(body, i)
                    i += 1
                    clients[writer].append(filt)
                    new.append(filt)
                    granted.append(0)
                    log(f"[broker] SUBSCRIBE {filt}")
                writer.write(bytes([0x90]) + encode_len(2 + len(granted)) + pid + granted)
                for t, p in retained.items():                # retained messages, as Mosquitto
                    if any(matches(f, t) for f in new):
                        writer.write(publish_packet(t, p, retain=True))
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
        to_page("sys", "broker", f"{peer[0]} disconnected")
        writer.close()


# ---- web page (HTTP + server-sent events) --------------------------------------
async def http_handle(reader, writer):
    try:
        head = await reader.readuntil(b"\r\n\r\n")
    except (asyncio.IncompleteReadError, asyncio.LimitOverrunError, ConnectionError):
        writer.close()
        return
    lines = head.decode("latin-1").split("\r\n")
    try:
        method, path, _ = lines[0].split(" ", 2)
    except ValueError:
        writer.close()
        return
    headers = {}
    for ln in lines[1:]:
        if ":" in ln:
            k, v = ln.split(":", 1)
            headers[k.strip().lower()] = v.strip()
    body = b""
    n = int(headers.get("content-length", "0") or 0)
    if n:
        body = await reader.readexactly(n)

    def reply(code, ctype, data):
        writer.write(f"HTTP/1.1 {code}\r\nContent-Type: {ctype}\r\n"
                     f"Content-Length: {len(data)}\r\nCache-Control: no-cache\r\n"
                     f"Connection: close\r\n\r\n".encode() + data)

    try:
        if method == "GET" and path in ("/", "/index.html"):
            with open(PAGE, "rb") as f:
                reply("200 OK", "text/html; charset=utf-8", f.read())
        elif method == "GET" and path == "/events":
            writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\n"
                         b"Cache-Control: no-cache\r\nConnection: keep-alive\r\n\r\n")
            q = asyncio.Queue()
            for m in list(history):
                q.put_nowait(json.dumps(m))
            sse_queues.add(q)
            try:
                while True:
                    try:
                        data = await asyncio.wait_for(q.get(), timeout=1.0)
                        writer.write(f"data: {data}\n\n".encode())
                    except asyncio.TimeoutError:
                        info = {"kind": "info", "ts": time.time() * 1000.0,
                                "robot_age": (time.time() - stats["robot_seen"]) if stats["robot_seen"] else -1,
                                "robot_peer": stats["robot_peer"], "msgs": stats["msgs"],
                                "log": log_file.name if log_file else ""}
                        writer.write(f"data: {json.dumps(info)}\n\n".encode())
                    await writer.drain()
            finally:
                sse_queues.discard(q)
            return
        elif method == "POST" and path == "/publish":
            req = json.loads(body.decode("utf-8") or "{}")
            topic = str(req.get("topic", "is09/cmd"))
            payload = str(req.get("payload", "")).encode()
            if req.get("retain"):
                retained[topic] = payload
            log(f"[dashboard] -> {topic} {payload.decode()}")
            to_page("sent", topic, payload.decode())
            await forward(topic, payload)
            reply("200 OK", "application/json", b'{"ok":true}')
        else:
            reply("404 Not Found", "text/plain", b"not found")
        await writer.drain()
    except (ConnectionError, OSError, ValueError):
        pass
    finally:
        try:
            writer.close()
        except Exception:
            pass


# ---- broker beacon --------------------------------------------------------------
def beacon(port, stop):
    msg = f"IS09_BROKER {port}".encode()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    except OSError as e:
        print(f"(broker beacon off: {e})", flush=True)
        return
    while not stop.is_set():
        try:
            s.sendto(msg, ("255.255.255.255", 47002))
        except OSError:
            pass
        stop.wait(2.0)


def lan_ips():
    ips = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    return sorted(ip for ip in ips if not ip.startswith("127."))


async def main(args):
    mqtt = await asyncio.start_server(mqtt_handle, "0.0.0.0", args.mqtt_port)
    web = await asyncio.start_server(http_handle, "0.0.0.0", args.http_port)
    url = f"http://localhost:{args.http_port}"
    print(f"MQTT broker on port {args.mqtt_port}; laptop IPs: {', '.join(lan_ips()) or '?'}", flush=True)
    print(f"Dashboard: {url}   (log: {log_file.name})", flush=True)
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    async with mqtt, web:
        await asyncio.gather(mqtt.serve_forever(), web.serve_forever())


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--no-beacon", action="store_true")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--log", default=None, help="log file (default logs/dashboard_<date>_<time>.log)")
    a = ap.parse_args()
    quiet = a.quiet
    path = a.log or os.path.join(HERE, "logs", datetime.datetime.now().strftime("dashboard_%Y%m%d_%H%M%S.log"))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    log_file = open(path, "a", encoding="utf-8")
    stop = threading.Event()
    if not a.no_beacon:
        threading.Thread(target=beacon, args=(a.mqtt_port, stop), daemon=True).start()
    try:
        asyncio.run(main(a))
    except KeyboardInterrupt:
        pass
    except OSError as e:
        print(f"Cannot open a port: {e}\nIs is09_broker.py, Mosquitto or another dashboard still running?", flush=True)
        sys.exit(1)
    finally:
        stop.set()
