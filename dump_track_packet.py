#!/usr/bin/env python3
import struct
import time

from track_packet import build_track_body

# Constants copied to match main.py
MAGIC = b'HRGK'
DEVICE_MODEL = 6000
DEVICE_ID = 0x8001
MSG_TRACK = 0x3001


def u8(x): return x & 0xFF
def u16(x): return x & 0xFFFF
def u32(x): return x & 0xFFFFFFFF


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


class Target:
    def __init__(self):
        # default target similar to simulator
        self.track_id = 1
        self.lat = 39.9075 + 0.002
        self.lon = 116.3913 + 0.002
        self.alt = 50.0 + 120.0
        self.speed = 15.0
        self.heading = 0.0
        self.target_type = 1
        self.size = 0
        self.intensity = 25.0


def build_header(total_len: int, check_mode: int = 2, seq: int = 1, counter: int = 1):
    utc_ms = int(time.time() * 1000)
    return struct.pack(
        '<4sHHQHHHHHBBI',
        MAGIC,
        u16(total_len),
        u16(DEVICE_MODEL),
        (utc_ms & 0xFFFFFFFF) | ((utc_ms & 0xFFFFFFFF) << 0),
        u16(MSG_TRACK),
        u16(0),  # ext id
        u16(DEVICE_ID),
        0,  # ext dev id
        0,  # reserved
        u8(check_mode),
        u8(seq),
        u32(counter),
    )


def main():
    # radar pose
    radar_lon = 116.3913
    radar_lat = 39.9075
    radar_alt = 50.0
    inu_valid = 1

    t = Target()
    body = build_track_body(inu_valid, radar_lon, radar_lat, radar_alt, t)
    # header depends on total length (header + body + 2 bytes CRC)
    total_len = 32 + len(body) + 2
    header = build_header(total_len, check_mode=2, seq=1, counter=1)

    packet_no_crc = header + body
    crc = crc16_modbus(packet_no_crc)
    packet = packet_no_crc + struct.pack('<H', u16(crc))

    # print hex
    hx = ' '.join(f'{b:02X}' for b in packet)
    print(hx)
    print(f'len={len(packet)} bytes, crc=0x{crc:04X}')


if __name__ == '__main__':
    main()
