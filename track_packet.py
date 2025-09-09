"""
构建“雷达航迹报文”体与航迹信息结构体，遵循表22/表23。

提供：
- build_track_body(inu_valid, radar_lon, radar_lat, radar_alt, target) -> bytes
  返回： uint8 惯导有效 + 雷达经纬高 + 航迹信息*1 + 预留16 字节

target 需具备属性：
  track_id, lon, lat, alt, speed, heading, intensity, target_type, size
"""
from __future__ import annotations

import math
import struct
from typing import Protocol, List


class _TargetLike(Protocol):
    track_id: int
    lon: float
    lat: float
    alt: float
    speed: float
    heading: float
    intensity: float
    target_type: int
    size: int


EARTH_R = 6371000.0


def _deg2rad(d: float) -> float:
    return d * math.pi / 180.0


def _rad2deg(r: float) -> float:
    return r * 180.0 / math.pi


def _normalize_angle_deg(a: float) -> float:
    a = a % 360.0
    return a if a >= 0 else a + 360.0


def _bearing_distance(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float):
    """
    从点1到点2的真北顺时针方位角与大圆距离（米）
    """
    lat1 = _deg2rad(lat1_deg)
    lon1 = _deg2rad(lon1_deg)
    lat2 = _deg2rad(lat2_deg)
    lon2 = _deg2rad(lon2_deg)
    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    dist = EARTH_R * c

    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    brg = math.atan2(y, x)
    brg_deg = (_rad2deg(brg) + 360.0) % 360.0
    return brg_deg, dist


def _u8(x: int) -> int:
    return x & 0xFF


def _u16(x: int) -> int:
    return x & 0xFFFF


def pack_track_info(
    radar_lat: float,
    radar_lon: float,
    radar_alt: float,
    t: _TargetLike,
) -> bytes:
    """
    打包表23 航迹信息结构体。
    字段顺序：
      uint16 track_id
      double tgt_lon, double tgt_lat, float tgt_alt
      float dist, float az, float el
      float speed, float course, float intensity
      uint8[4] reserved
      uint8 type, uint8 size, uint8 point_type, uint8 track_type
      uint8 lost_cnt, uint8 quality
      float raw_dist, float raw_az, float raw_el
      uint8[3] reserved
    """
    az_deg, dist_m = _bearing_distance(radar_lat, radar_lon, t.lat, t.lon)
    # 俯仰角
    horiz = max(0.01, math.sqrt(max(dist_m ** 2 - (t.alt - radar_alt) ** 2, 0.0)))
    el_deg = math.degrees(math.atan2(t.alt - radar_alt, horiz))

    parts = []
    parts.append(struct.pack('<H', _u16(t.track_id)))
    parts.append(struct.pack('<ddf', float(t.lon), float(t.lat), float(t.alt)))
    parts.append(struct.pack('<fff', float(dist_m), float(az_deg), float(el_deg)))
    parts.append(struct.pack('<fff', float(t.speed), float(_normalize_angle_deg(t.heading)), float(t.intensity)))
    parts.append(bytes(4))  # 预留
    # point_type=0 检测点, track_type=1 搜索航迹（可按需调整）
    parts.append(struct.pack('<BBBB', _u8(t.target_type), _u8(t.size), 0, 1))
    parts.append(struct.pack('<BB', 0, 90))  # 连续丢失次数、航迹质量
    parts.append(struct.pack('<fff', float(dist_m), float(az_deg), float(el_deg)))
    parts.append(bytes(3))
    return b''.join(parts)


def build_track_body(
    inu_valid: int,
    radar_lon: float,
    radar_lat: float,
    radar_alt: float,
    t: _TargetLike,
) -> bytes:
    """
    构建表22的航迹报文体（不含帧头与最终校验位）：
      uint8 惯导有效
      double 雷达经度
      double 雷达纬度
      float  雷达海拔
      航迹信息*1
      预留 uint8[16]
    """
    body = struct.pack('<Bddf', _u8(inu_valid), float(radar_lon), float(radar_lat), float(radar_alt))
    body += pack_track_info(radar_lat, radar_lon, radar_alt, t)
    body += bytes(16)
    return body


def build_tracks_body(
    inu_valid: int,
    radar_lon: float,
    radar_lat: float,
    radar_alt: float,
    targets: List[_TargetLike],
) -> bytes:
    """
    构建包含多个航迹信息（表23）的航迹报文体：
      uint8 惯导有效
      double 雷达经度
      double 雷达纬度
      float  雷达海拔
      航迹信息 * N
      预留 uint8[16]
    """
    body = struct.pack('<Bddf', _u8(inu_valid), float(radar_lon), float(radar_lat), float(radar_alt))
    for t in targets:
        body += pack_track_info(radar_lat, radar_lon, radar_alt, t)
    body += bytes(16)
    return body
