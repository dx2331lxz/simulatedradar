#!/usr/bin/env python3
"""简单 UDP 测试客户端：构造协议帧（CMD_SEARCH）发送到雷达仿真器并等待 ACK
用法示例：
  python udp_test_client.py --dest-ip 127.0.0.1 --dest-port 0x1888
"""
import argparse
import socket
import struct
import time

MAGIC = b'HRGK'
CMD_SEARCH = 0x1004
CMD_STANDBY = 0x1003


def now_millis():
    return int(time.time() * 1000)


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
                 device_id: int = 0x8001, check_mode: int = 2, seq: int = 1, counter: int = 1) -> bytes:
    # '<4sHHQHHHHHBBI' 32 bytes
    utc_ms = now_millis()
    # pack fields; NOTE total_len must be u16
    return struct.pack('<4sHHQHHHHHBBI',
                       MAGIC,
                       total_len & 0xFFFF,
                       device_model & 0xFFFF,
                       (utc_ms & 0xFFFFFFFF) | ((utc_ms & 0xFFFFFFFF) << 0),
                       u16(msg_id),
                       u16(ext_msg_id & 0xFFFF),
                       u16(device_id & 0xFFFF),
                       0,
                       0,
                       u8(check_mode),
                       u8(seq),
                       int(counter & 0xFFFFFFFF)
                       )


def u8(x):
    return x & 0xFF


def u16(x):
    return x & 0xFFFF


def build_packet(msg_id: int, body: bytes = b'', check_mode: int = 2, seq: int = 1) -> bytes:
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


def hexd(b: bytes) -> str:
    return ' '.join(f'{x:02X}' for x in b)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dest-ip', default='127.0.0.1')
    ap.add_argument('--dest-port', type=lambda x: int(x, 0), default=0x1888)
    ap.add_argument('--timeout', type=float, default=2.0)
    ap.add_argument('--seq', type=int, default=5)
    ap.add_argument('--msg-id', type=lambda x: int(x, 0), default=CMD_SEARCH, help='要发送的消息ID（例如 0x1004 搜索，0x1003 待机）')
    ap.add_argument('--task-type', type=lambda x: int(x, 0), help='任务类型：搜索1，待机0；不传则 body 为空用于兼容测试')
    args = ap.parse_args()

    if args.task_type is None:
        body = b''  # 兼容仅有命令头的情况
    else:
        # 1字节 task_type + 16字节保留
        body = bytes([args.task_type & 0xFF]) + bytes(16)

    pkt = build_packet(args.msg_id, body=body, check_mode=2, seq=args.seq)
    print(f'Sending msg_id=0x{args.msg_id:04X} (seq={args.seq}, body_len={len(body)}) to {args.dest_ip}:{args.dest_port}, packet len={len(pkt)}')

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(args.timeout)
    try:
        s.sendto(pkt, (args.dest_ip, args.dest_port))
        try:
            data, addr = s.recvfrom(4096)
            print(f'Received {len(data)} bytes from {addr}:')
            print(hexd(data[:32]) + (' ...' if len(data) > 32 else ''))
        except socket.timeout:
            print('No reply within timeout. 仿真器可能未运行或绑定地址/端口不匹配。')
    finally:
        s.close()


if __name__ == '__main__':
    main()
