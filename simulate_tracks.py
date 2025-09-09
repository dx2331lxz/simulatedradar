#!/usr/bin/env python3
"""
simulate_tracks.py

生成并打包“航迹信息结构体”（表23）。

功能：
- TrackInfo dataclass，支持 pack()/unpack()
- 随机模拟目标（类型/尺寸/坐标/角度/速度等）
- CLI：生成 N 条记录，可保存为二进制文件或通过 UDP 发送

注意：使用小端字节序（'<') 打包，字段顺序严格按照表23。
"""
from dataclasses import dataclass
import struct
import random
import argparse
import socket
from typing import List


# 定义 struct 格式（小端）
# 字段顺序：见表23
# H: uint16
# d: double
# f: float
# B: uint8
FORMAT = '<Hddfffffff4B6Bfff3B'
SIZE = struct.calcsize(FORMAT)


@dataclass
class TrackInfo:
    batch_no: int = 0  # uint16
    lon: float = 0.0  # double (deg)
    lat: float = 0.0  # double (deg)
    altitude: float = 0.0  # float (m)
    distance: float = 0.0  # float (m)
    azimuth: float = 0.0  # float (deg)
    pitch: float = 0.0  # float (deg)
    speed: float = 0.0  # float (m/s)
    heading: float = 0.0  # float (deg)
    intensity: float = 0.0  # float (dB)
    reserved4_0: int = 0
    reserved4_1: int = 0
    reserved4_2: int = 0
    reserved4_3: int = 0
    target_type: int = 0  # uint8
    target_size: int = 0  # uint8
    point_type: int = 0  # uint8
    track_type: int = 0  # uint8
    lost_times: int = 0  # uint8
    track_quality: int = 0  # uint8 (0-100)
    raw_distance: float = 0.0  # float (m)
    raw_azimuth: float = 0.0  # float (deg)
    raw_pitch: float = 0.0  # float (deg)
    reserved3_0: int = 0
    reserved3_1: int = 0
    reserved3_2: int = 0

    def pack(self) -> bytes:
        """Pack the TrackInfo into bytes according to FORMAT."""
        values = [
            self.batch_no,
            self.lon,
            self.lat,
            self.altitude,
            self.distance,
            self.azimuth,
            self.pitch,
            self.speed,
            self.heading,
            self.intensity,
            self.reserved4_0,
            self.reserved4_1,
            self.reserved4_2,
            self.reserved4_3,
            self.target_type,
            self.target_size,
            self.point_type,
            self.track_type,
            self.lost_times,
            self.track_quality,
            self.raw_distance,
            self.raw_azimuth,
            self.raw_pitch,
            self.reserved3_0,
            self.reserved3_1,
            self.reserved3_2,
        ]
        return struct.pack(FORMAT, *values)

    @staticmethod
    def unpack(data: bytes) -> 'TrackInfo':
        """Unpack bytes into a TrackInfo instance."""
        if len(data) < SIZE:
            raise ValueError(f"data too short: {len(data)} < {SIZE}")
        vals = struct.unpack(FORMAT, data[:SIZE])
        return TrackInfo(
            batch_no=vals[0],
            lon=vals[1],
            lat=vals[2],
            altitude=vals[3],
            distance=vals[4],
            azimuth=vals[5],
            pitch=vals[6],
            speed=vals[7],
            heading=vals[8],
            intensity=vals[9],
            reserved4_0=vals[10],
            reserved4_1=vals[11],
            reserved4_2=vals[12],
            reserved4_3=vals[13],
            target_type=vals[14],
            target_size=vals[15],
            point_type=vals[16],
            track_type=vals[17],
            lost_times=vals[18],
            track_quality=vals[19],
            raw_distance=vals[20],
            raw_azimuth=vals[21],
            raw_pitch=vals[22],
            reserved3_0=vals[23],
            reserved3_1=vals[24],
            reserved3_2=vals[25],
        )

    @staticmethod
    def random(batch_no: int = 1) -> 'TrackInfo':
        """Generate a random TrackInfo within reasonable ranges."""
        lon = random.uniform(-180.0, 180.0)
        lat = random.uniform(-90.0, 90.0)
        altitude = random.uniform(0.0, 12000.0)  # meters
        distance = random.uniform(0.0, 200000.0)
        azimuth = random.uniform(0.0, 360.0)
        pitch = random.uniform(-90.0, 90.0)
        speed = random.uniform(0.0, 400.0)
        heading = random.uniform(0.0, 360.0)
        intensity = random.uniform(-40.0, 40.0)
        # reserved4 default 0
        target_type = random.choice([0x00, 0x01, 0x02, 0x03, 0x04, 0x05])
        target_size = random.choice([0x00, 0x01, 0x02, 0x03])
        point_type = random.choice([0x00, 0x01])
        track_type = random.choice([0x00, 0x01, 0x02])
        lost_times = random.randint(0, 10)
        track_quality = random.randint(0, 100)
        raw_distance = distance + random.uniform(-5.0, 5.0)
        raw_azimuth = (azimuth + random.uniform(-0.5, 0.5)) % 360.0
        raw_pitch = pitch + random.uniform(-0.2, 0.2)
        reserved3 = (0, 0, 0)
        return TrackInfo(
            batch_no=batch_no,
            lon=lon,
            lat=lat,
            altitude=altitude,
            distance=distance,
            azimuth=azimuth,
            pitch=pitch,
            speed=speed,
            heading=heading,
            intensity=intensity,
            reserved4_0=0,
            reserved4_1=0,
            reserved4_2=0,
            reserved4_3=0,
            target_type=target_type,
            target_size=target_size,
            point_type=point_type,
            track_type=track_type,
            lost_times=lost_times,
            track_quality=track_quality,
            raw_distance=raw_distance,
            raw_azimuth=raw_azimuth,
            raw_pitch=raw_pitch,
            reserved3_0=reserved3[0],
            reserved3_1=reserved3[1],
            reserved3_2=reserved3[2],
        )


def generate_tracks(count: int, start_batch: int = 1) -> List[TrackInfo]:
    return [TrackInfo.random(batch_no=(start_batch + i)) for i in range(count)]


def send_udp(host: str, port: int, data: bytes):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(data, (host, port))


def main():
    parser = argparse.ArgumentParser(description='生成并打包航迹信息结构体（表23）')
    parser.add_argument('--count', '-n', type=int, default=1, help='生成航迹条数')
    parser.add_argument('--out', '-o', type=str, default=None, help='输出到二进制文件（例如 tracks.bin）')
    parser.add_argument('--udp', type=str, default=None, help='以 host:port 形式发送 UDP')
    args = parser.parse_args()

    tracks = generate_tracks(args.count)
    data = b''.join(t.pack() for t in tracks)

    if args.out:
        with open(args.out, 'wb') as f:
            f.write(data)
        print(f'Wrote {len(tracks)} tracks ({len(data)} bytes) to {args.out}')

    if args.udp:
        host, port_s = args.udp.split(':')
        port = int(port_s)
        send_udp(host, port, data)
        print(f'Sent {len(data)} bytes to {host}:{port} via UDP')

    if not args.out and not args.udp:
        # 默认打印每条结构体的 hex preview
        for i, t in enumerate(tracks, 1):
            print(f'Track {i}: batch={t.batch_no} lon={t.lon:.6f} lat={t.lat:.6f} type=0x{t.target_type:02X} size=0x{t.target_size:02X} quality={t.track_quality}%')


if __name__ == '__main__':
    main()
