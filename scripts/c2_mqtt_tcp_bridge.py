#!/usr/bin/env python3
"""Cầu nối C2 hai chiều: TCP Server (cổng 5760 cho QGC / Mission Planner) <-> MQTT (Raw Binary MAVLink).

Chỉ dùng thư viện chuẩn Python 3 (không cần cài thêm package ngoài).

Cách dùng:
  1. Chạy script:
       python3 scripts/c2_mqtt_tcp_bridge.py
  2. Trên QGroundControl (Application Settings -> Comm Links -> Add):
       - Type: TCP
       - Server Address: 127.0.0.1
       - Port: 5760
     Rồi nhấn Connect.
"""

import argparse
import errno
import select
import socket
import struct
import sys
import time

# Giới hạn kích thước mỗi gói PUBLISH gửi xuống ESP32 để luôn nhỏ hơn
# TDM2421_MQTT_MAX_RX_PKT (1600 B) và TDM2421_MQTT_RX_RING_SIZE (2048 B).
MAX_UPLINK_CHUNK = 1024


def encode_rem_len(val: int) -> bytes:
    """Mã hóa Remaining Length theo chuẩn MQTT 3.1.1."""
    out = bytearray()
    while True:
        b = val % 128
        val //= 128
        if val > 0:
            b |= 0x80
        out.append(b)
        if val == 0:
            break
    return bytes(out)


def encode_utf8(s: str) -> bytes:
    raw = s.encode("utf-8")
    return struct.pack("!H", len(raw)) + raw


def build_connect(client_id: str, user: str, password: str, keepalive: int = 30) -> bytes:
    vh = b"\x00\x04MQTT\x04"
    flags = 0x02  # Clean Session
    if user:
        flags |= 0x80
    if password:
        flags |= 0x40
    body = bytearray(vh + bytes([flags]) + struct.pack("!H", keepalive))
    body += encode_utf8(client_id)
    if user:
        body += encode_utf8(user)
    if password:
        body += encode_utf8(password)
    return bytes([0x10]) + encode_rem_len(len(body)) + bytes(body)


def build_subscribe(packet_id: int, topic: str) -> bytes:
    body = struct.pack("!H", packet_id) + encode_utf8(topic) + b"\x00"
    return bytes([0x82]) + encode_rem_len(len(body)) + body


def build_publish(topic: str, payload: bytes) -> bytes:
    # QoS 0, retain=0 (tuyệt đối không bật retain cho lệnh C2).
    body = encode_utf8(topic) + payload
    return bytes([0x30]) + encode_rem_len(len(body)) + body


def parse_mqtt_packets(buf: bytearray):
    """Tách các gói MQTT hoàn chỉnh từ luồng byte TCP."""
    packets = []
    while len(buf) >= 2:
        b0 = buf[0]
        mult = 1
        rem_len = 0
        hdr_len = 0
        need_more = False
        for i in range(1, 5):
            if len(buf) <= i:
                need_more = True
                break
            enc = buf[i]
            rem_len += (enc & 0x7F) * mult
            if (enc & 0x80) == 0:
                hdr_len = i + 1
                break
            mult *= 128
        else:
            raise ValueError("Malformed MQTT remaining length")
        if need_more:
            break
        total = hdr_len + rem_len
        if len(buf) < total:
            break
        pkt_body = bytes(buf[hdr_len:total])
        del buf[:total]
        packets.append((b0, pkt_body))
    return packets


def inspect_mavlink_payload(payload: bytes):
    """Đếm số frame MAVLink và số gói ATTITUDE (#30) / HEARTBEAT (#0) trong 1 payload."""
    pos = 0
    total_frames = 0
    att_frames = 0
    hb_frames = 0
    while pos < len(payload):
        stx = payload[pos]
        if stx == 0xFD and pos + 12 <= len(payload):
            plen = payload[pos + 1]
            incompat = payload[pos + 2]
            msgid = payload[pos + 7] | (payload[pos + 8] << 8) | (payload[pos + 9] << 16)
            tlen = 10 + plen + 2 + (13 if (incompat & 0x01) else 0)
            if pos + tlen > len(payload):
                break
            total_frames += 1
            if msgid == 30:
                att_frames += 1
            elif msgid == 0:
                hb_frames += 1
            pos += tlen
        elif stx == 0xFE and pos + 8 <= len(payload):
            plen = payload[pos + 1]
            msgid = payload[pos + 5]
            tlen = 6 + plen + 2
            if pos + tlen > len(payload):
                break
            total_frames += 1
            if msgid == 30:
                att_frames += 1
            elif msgid == 0:
                hb_frames += 1
            pos += tlen
        else:
            break
    return total_frames, att_frames, hb_frames


def open_tcp_server(host: str, port: int) -> socket.socket:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(2)
    srv.setblocking(False)
    return srv


def tune_tcp_socket(sock: socket.socket):
    """Bật TCP_NODELAY và TCP_QUICKACK (nếu HĐH hỗ trợ) để giảm độ trễ."""
    try:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except OSError:
        pass
    if hasattr(socket, "TCP_QUICKACK"):
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
        except OSError:
            pass


def drain_nonblocking(sock: socket.socket) -> tuple[bytes, bool]:
    """Đọc hết toàn bộ byte đang có trong socket non-blocking. Trả về (data, closed)."""
    chunks = []
    while True:
        try:
            data = sock.recv(8192)
        except BlockingIOError:
            break
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                break
            return b"".join(chunks), True
        if not data:
            return b"".join(chunks), True
        chunks.append(data)
    return b"".join(chunks), False


def flush_tx_buf(sock: socket.socket, tx_buf: bytearray) -> bool:
    """Đẩy dữ liệu từ buffer ghi ra socket non-blocking. Trả về False nếu socket lỗi/đóng."""
    while tx_buf:
        try:
            sent = sock.send(tx_buf)
            if sent <= 0:
                return False
            del tx_buf[:sent]
        except BlockingIOError:
            break
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                break
            return False
    return True


def close_quiet(sock: socket.socket | None):
    if sock is not None:
        try:
            sock.close()
        except OSError:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(description="TDM2421 C2 TCP Server (5760) <-> MQTT Bridge for QGC")
    ap.add_argument("--tcp-host", default="0.0.0.0", help="IP lắng nghe cho QGC (mặc định 0.0.0.0)")
    ap.add_argument("--tcp-port", type=int, default=5760, help="Cổng TCP cho QGC (mặc định 5760)")
    ap.add_argument("--mqtt-host", default="171.251.89.98")
    ap.add_argument("--mqtt-port", type=int, default=8903)
    ap.add_argument("--mqtt-user", default="pilot")
    ap.add_argument("--mqtt-pass", default="pilot123")
    ap.add_argument("--client-id", default="TDM2421_QGC_TCP_BRIDGE")
    ap.add_argument("--remote-id", default="1866ASDD0000VT5", help="Remote ID của drone cần kết nối C2")
    ap.add_argument("--tx-topic", default=None, help="Topic nhận binary MAVLink từ ESP32/FC (mặc định p3000m/c2/<remote-id>/tx)")
    ap.add_argument("--rx-topic", default=None, help="Topic gửi lệnh binary MAVLink xuống ESP32/FC (mặc định p3000m/c2/<remote-id>/rx)")
    args = ap.parse_args()
    if args.tx_topic is None:
        args.tx_topic = f"p3000m/c2/{args.remote_id}/tx"
    if args.rx_topic is None:
        args.rx_topic = f"p3000m/c2/{args.remote_id}/rx"

    tcp_srv = open_tcp_server(args.tcp_host, args.tcp_port)
    qgc_sock: socket.socket | None = None
    qgc_addr = None
    qgc_tx_buf = bytearray()
    print(f"[TCP] Listening on {args.tcp_host}:{args.tcp_port} (connect QGC to 127.0.0.1:{args.tcp_port})")

    try:
        while True:
            mqtt_sock = None
            mqtt_tx_buf = bytearray()
            try:
                print(f"[MQTT] Connecting to {args.mqtt_host}:{args.mqtt_port} ...")
                mqtt_sock = socket.create_connection((args.mqtt_host, args.mqtt_port), timeout=10)
                tune_tcp_socket(mqtt_sock)
                mqtt_sock.settimeout(5)
                mqtt_sock.sendall(build_connect(args.client_id, args.mqtt_user, args.mqtt_pass))

                mqtt_rx_buf = bytearray()
                connack_ok = False
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline and not connack_ok:
                    chunk = mqtt_sock.recv(4096)
                    if not chunk:
                        raise ConnectionError("MQTT broker closed before CONNACK")
                    mqtt_rx_buf.extend(chunk)
                    for b0, body in parse_mqtt_packets(mqtt_rx_buf):
                        if (b0 >> 4) == 2 and len(body) >= 2:
                            if body[1] != 0:
                                raise ConnectionError(f"CONNACK rejected rc={body[1]}")
                            connack_ok = True
                            break
                if not connack_ok:
                    raise TimeoutError("CONNACK timeout")

                mqtt_sock.sendall(build_subscribe(1, args.tx_topic))
                mqtt_sock.setblocking(False)
                print(f"[MQTT] Ready: sub='{args.tx_topic}' -> QGC | QGC -> pub='{args.rx_topic}'")

                last_ping = time.monotonic()
                last_stat = time.monotonic()
                last_dl_pkt_t = 0.0

                # Bộ đếm trong cửa sổ 2 giây để đo chính xác tần số thực tế.
                win_dl_pkts = 0
                win_dl_bytes = 0
                win_dl_frames = 0
                win_att_frames = 0
                win_ul_pkts = 0
                win_ul_bytes = 0
                win_max_dt_ms = 0.0

                while True:
                    rwatch = [mqtt_sock, tcp_srv]
                    wwatch = []
                    if mqtt_tx_buf:
                        wwatch.append(mqtt_sock)
                    if qgc_sock is not None:
                        rwatch.append(qgc_sock)
                        if qgc_tx_buf:
                            wwatch.append(qgc_sock)

                    rlist, wlist, _ = select.select(rwatch, wwatch, [], 0.2)
                    now = time.monotonic()

                    # 1. Chấp nhận kết nối mới từ QGC trên cổng 5760.
                    if tcp_srv in rlist:
                        client, addr = tcp_srv.accept()
                        tune_tcp_socket(client)
                        client.setblocking(False)
                        qgc_tx_buf.clear()
                        if qgc_sock is not None:
                            print(f"[TCP] Replacing previous QGC client {qgc_addr} with {addr}")
                            close_quiet(qgc_sock)
                        else:
                            print(f"[TCP] QGC connected from {addr}")
                        qgc_sock = client
                        qgc_addr = addr

                    # 2. Nhận toàn bộ dữ liệu đang chờ từ MQTT (FC -> ESP32 -> Broker) đẩy sang QGC.
                    if mqtt_sock in rlist:
                        chunk, closed = drain_nonblocking(mqtt_sock)
                        tune_tcp_socket(mqtt_sock)
                        if chunk:
                            mqtt_rx_buf.extend(chunk)
                            for b0, body in parse_mqtt_packets(mqtt_rx_buf):
                                ptype = (b0 >> 4) & 0x0F
                                if ptype == 3 and len(body) >= 2:
                                    tlen = struct.unpack("!H", body[:2])[0]
                                    pos = 2 + tlen
                                    if pos <= len(body):
                                        topic = body[2:pos].decode("utf-8", errors="replace")
                                        if ((b0 >> 1) & 0x03) > 0:
                                            pos += 2
                                        payload = body[pos:]
                                        if topic == args.tx_topic and payload:
                                            if last_dl_pkt_t > 0.0:
                                                dt_ms = (now - last_dl_pkt_t) * 1000.0
                                                if dt_ms > win_max_dt_ms:
                                                    win_max_dt_ms = dt_ms
                                            last_dl_pkt_t = now

                                            nf, natt, _ = inspect_mavlink_payload(payload)
                                            win_dl_pkts += 1
                                            win_dl_bytes += len(payload)
                                            win_dl_frames += nf
                                            win_att_frames += natt

                                            if qgc_sock is not None:
                                                qgc_tx_buf.extend(payload)
                        if closed:
                            raise ConnectionError("MQTT broker closed connection")

                    # 3. Xả buffer ghi ra QGC ngay (không để BlockingIOError làm rớt kết nối).
                    if qgc_sock is not None and (qgc_tx_buf or qgc_sock in wlist):
                        if not flush_tx_buf(qgc_sock, qgc_tx_buf):
                            print(f"[TCP] QGC write failed; closing client {qgc_addr}")
                            close_quiet(qgc_sock)
                            qgc_sock = None
                            qgc_addr = None
                            qgc_tx_buf.clear()

                    # 4. Nhận lệnh MAVLink từ QGC trên cổng 5760 bắn lên MQTT cho ESP32 nhận.
                    if qgc_sock is not None and qgc_sock in rlist:
                        data, closed = drain_nonblocking(qgc_sock)
                        if data:
                            for off in range(0, len(data), MAX_UPLINK_CHUNK):
                                sub = data[off:off + MAX_UPLINK_CHUNK]
                                mqtt_tx_buf.extend(build_publish(args.rx_topic, sub))
                                win_ul_pkts += 1
                                win_ul_bytes += len(sub)
                            last_ping = now
                        if closed:
                            print(f"[TCP] QGC disconnected ({qgc_addr})")
                            close_quiet(qgc_sock)
                            qgc_sock = None
                            qgc_addr = None
                            qgc_tx_buf.clear()

                    # 5. Keepalive MQTT.
                    if now - last_ping >= 20.0:
                        mqtt_tx_buf.extend(b"\xC0\x00")
                        last_ping = now

                    # 6. Xả buffer ghi lên MQTT ngay.
                    if mqtt_tx_buf or mqtt_sock in wlist:
                        if not flush_tx_buf(mqtt_sock, mqtt_tx_buf):
                            raise ConnectionError("MQTT socket write failed")

                    # 7. In thống kê chẩn đoán mỗi 2 giây: tách rõ MQTT Hz vs MAVLink frames/s vs ATTITUDE Hz.
                    dt_stat = now - last_stat
                    if dt_stat >= 2.0:
                        last_stat = now
                        qgc_state = f"connected({qgc_addr[0]}:{qgc_addr[1]})" if qgc_sock else "waiting"
                        print(
                            f"[STAT] QGC={qgc_state} | "
                            f"MQTT_RX={win_dl_pkts/dt_stat:.1f}Hz ({win_dl_bytes/dt_stat:.0f}B/s, maxGap={win_max_dt_ms:.0f}ms) | "
                            f"MAV={win_dl_frames/dt_stat:.1f}f/s ATT={win_att_frames/dt_stat:.1f}Hz | "
                            f"UL={win_ul_pkts/dt_stat:.1f}Hz/{win_ul_bytes}B"
                        )
                        win_dl_pkts = 0
                        win_dl_bytes = 0
                        win_dl_frames = 0
                        win_att_frames = 0
                        win_ul_pkts = 0
                        win_ul_bytes = 0
                        win_max_dt_ms = 0.0

            except KeyboardInterrupt:
                print("\n[EXIT] Stopping bridge.")
                return 0
            except Exception as exc:
                print(f"[WARN] MQTT error: {exc}; reconnecting in 3s...", file=sys.stderr)
                time.sleep(3)
            finally:
                close_quiet(mqtt_sock)
    finally:
        close_quiet(qgc_sock)
        close_quiet(tcp_srv)


if __name__ == "__main__":
    sys.exit(main())
