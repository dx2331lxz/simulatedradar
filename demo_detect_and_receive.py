#!/usr/bin/env python3
"""
演示：发送 CMD_SEARCH 到雷达仿真器，接收 ACK 与紧随其后的 TRACK 报文。
用途：简单验证“目标检测 -> 立即上传航迹”。
"""
import argparse
import socket
import struct
import time

MAGIC = b'HRGK'
CMD_SEARCH = 0x1004
MSG_ACK = 0xF000
MSG_TRACK = 0x3001


def now_millis():
    return int(time.time() * 1000)


def u8(x):
    return x & 0xFF


def u16(x):
    return x & 0xFFFF


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_header(msg_id: int, ext_msg_id: int, total_len: int, device_model: int = 6000,
                 device_id: int = 0x8001, check_mode: int = 2, seq: int = 9, counter: int = 1) -> bytes:
    utc_ms = now_millis()
    return struct.pack('<4sHHQHHHHHBBI',
                       MAGIC,
                       total_len & 0xFFFF,
                       device_model & 0xFFFF,
                       (utc_ms & 0xFFFFFFFF) | ((utc_ms & 0xFFFFFFFF) << 0),
                       u16(msg_id),
                       u16(ext_msg_id & 0xFFFF),
                       u16(device_id & 0xFFFF),
                       0, 0,
                       u8(check_mode),
                       u8(seq),
                       int(counter & 0xFFFFFFFF))


def build_packet(msg_id: int, body: bytes = b'', check_mode: int = 2, seq: int = 9) -> bytes:
    total_len = 32 + len(body) + 2
    header = build_header(msg_id, 0, total_len, check_mode=check_mode, seq=seq)
    packet_no_crc = header + body
    if check_mode == 2:
        chk = crc16_modbus(packet_no_crc)
    elif check_mode == 1:
        chk = sum(packet_no_crc) & 0xFFFF
    else:
        chk = 0
    return packet_no_crc + struct.pack('<H', u16(chk))


def parse_header(data: bytes):
    return struct.unpack('<4sHHQHHHHHBBI', data[:32])


def parse_track_body(data: bytes):
    # data 为去掉帧头与CRC后的 body
    # body = uint8 inu + double radar_lon + double radar_lat + float radar_alt + track_info + 16B reserved
    inu_valid, radar_lon, radar_lat, radar_alt = struct.unpack('<Bddf', data[:1 + 8 + 8 + 4])
    off = 1 + 8 + 8 + 4
    # track_info:
    # uint16 id; dd f; fff; fff; 4B; 4B; 2B; fff; 3B
    track_id = struct.unpack('<H', data[off:off + 2])[0]
    off += 2
    tgt_lon, tgt_lat, tgt_alt = struct.unpack('<ddf', data[off:off + 8 + 8 + 4])
    off += 8 + 8 + 4
    dist, az, el = struct.unpack('<fff', data[off:off + 12])
    off += 12
    spd, course, intensity = struct.unpack('<fff', data[off:off + 12])
    # 其余字段此处不再展开
    return {
        'inu_valid': inu_valid,
        'radar_lon': radar_lon,
        'radar_lat': radar_lat,
        'radar_alt': radar_alt,
        'track_id': track_id,
        'tgt_lon': tgt_lon,
        'tgt_lat': tgt_lat,
        'tgt_alt': tgt_alt,
        'dist': dist,
        'az': az,
        'el': el,
        'spd': spd,
        'course': course,
        'intensity': intensity,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--radar-ip', default='127.0.0.1')
    ap.add_argument('--radar-port', type=lambda x: int(x, 0), default=0x1888)
    ap.add_argument('--timeout', type=float, default=2.0)
    args = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(args.timeout)
    pkt = build_packet(CMD_SEARCH, b'', check_mode=2, seq=7)
    print(f'[demo] Send CMD_SEARCH to {args.radar_ip}:{args.radar_port}, len={len(pkt)}')
    s.sendto(pkt, (args.radar_ip, args.radar_port))

    # 期望在超时窗口内收到 ACK 与紧随其后的 TRACK
    recv_cnt = 0
    t_end = time.time() + args.timeout
    while time.time() < t_end and recv_cnt < 2:
        try:
            data, addr = s.recvfrom(8192)
        except socket.timeout:
            break
        recv_cnt += 1
        magic, total_len, dev_model, utc_ms, msg_id, ext_id, dev_id, ext_dev_id, reserved, chk_mode, seq, counter = parse_header(data)
        if magic != MAGIC:
            print('[demo] Unknown magic, skip')
            continue
        if msg_id == MSG_ACK:
            print(f'[demo] <- ACK from {addr}, len={len(data)}, seq={seq}')
        elif msg_id == MSG_TRACK:
            body = data[32:-2]
            info = parse_track_body(body)
            print('[demo] <- TRACK from {} len={} track_id={} inu={} dist(m)={:.1f} az(deg)={:.1f} el(deg)={:.1f} tgt=({:.6f},{:.6f})'.format(
                addr, len(data), info['track_id'], info['inu_valid'], info['dist'], info['az'], info['el'], info['tgt_lat'], info['tgt_lon']))
        else:
            print(f'[demo] <- Other msg_id=0x{msg_id:04X} from {addr}, len={len(data)}')

    if recv_cnt == 0:
        print('[demo] No response within timeout. 请确认仿真器已运行并监听端口。')


if __name__ == '__main__':
    main()
