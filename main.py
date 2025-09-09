import argparse
import math
import socket
import struct
import threading
import time
import random
from typing import Tuple, List, Optional
import logging
from track_packet import build_track_body

# =========================
# 协议与常量
# =========================

MAGIC = b'HRGK'  # 0x48 0x52 0x47 0x4B
DEVICE_MODEL = 6000
DEFAULT_DEVICE_ID = 0x8001  # 文档给出 80001, 与 uint16 冲突，这里采用 0x8001
DEFAULT_CHECK_MODE = 2  # 0:不校验 1:和校验 2:CRC16-MODBUS

# 工作状态
RADAR_STATE_INIT = 0x11
RADAR_STATE_STANDBY = 0x13
RADAR_STATE_SEARCH = 0x14
RADAR_STATE_TRACK = 0x15

# 报文ID
MSG_ACK = 0xF000
MSG_QUERY = 0xF001

CMD_STANDBY = 0x1003
CMD_SEARCH = 0x1004
CMD_TRACK = 0x1005
CMD_DEPLOY = 0x1008  # 展开/撤收任务

CFG_SILENT_SET = 0x2091
CFG_SILENT_FEEDBACK = 0x2092

MSG_TRACK = 0x3001
MSG_STATUS = 0x3002

# =========================
# 工具函数
# =========================

def now_millis() -> int:
    return int(time.time() * 1000)

def u8(x): return x & 0xFF
def u16(x): return x & 0xFFFF
def u32(x): return x & 0xFFFFFFFF

def checksum16_sum(data: bytes) -> int:
    # 按字节求和取低16位
    return sum(data) & 0xFFFF

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

def deg2rad(d): return d * math.pi / 180.0
def rad2deg(r): return r * 180.0 / math.pi

def normalize_angle_deg(a):
    # 0~360
    a = a % 360.0
    return a if a >= 0 else a + 360.0

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def speed_range_for_type(t: int):
    """返回目标类型对应的速度区间 (min, max) m/s。"""
    mapping = {
        0: (0.0, 400.0),
        1: (0.0, 25.0),
        2: (10.0, 60.0),
        3: (30.0, 120.0),
        4: (10.0, 80.0),
        5: (100.0, 250.0),
    }
    return mapping.get(t, (0.0, 400.0))


def sample_speed_for_type(t: int) -> float:
    lo, hi = speed_range_for_type(t)
    return random.uniform(lo, hi)

# 地球半径
EARTH_R = 6371000.0

def dest_from_bearing(lat_deg, lon_deg, bearing_deg, dist_m):
    # 根据起点、方位角、距离计算终点
    lat1 = deg2rad(lat_deg)
    lon1 = deg2rad(lon_deg)
    brg = deg2rad(bearing_deg)
    dr = dist_m / EARTH_R
    sin_lat1 = math.sin(lat1)
    cos_lat1 = math.cos(lat1)
    sin_dr = math.sin(dr)
    cos_dr = math.cos(dr)
    sin_lat2 = sin_lat1 * cos_dr + cos_lat1 * sin_dr * math.cos(brg)
    lat2 = math.asin(clamp(sin_lat2, -1.0, 1.0))
    y = math.sin(brg) * sin_dr * cos_lat1
    x = cos_dr - sin_lat1 * math.sin(lat2)
    lon2 = lon1 + math.atan2(y, x)
    return rad2deg(lat2), rad2deg(lon2)

def bearing_distance(lat1_deg, lon1_deg, lat2_deg, lon2_deg) -> Tuple[float, float]:
    # 从点1到点2的真北顺时针方位角与大圆距离
    lat1 = deg2rad(lat1_deg)
    lon1 = deg2rad(lon1_deg)
    lat2 = deg2rad(lat2_deg)
    lon2 = deg2rad(lon2_deg)
    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    dist = EARTH_R * c

    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
    brg = math.atan2(y, x)
    brg_deg = (rad2deg(brg) + 360.0) % 360.0
    return brg_deg, dist

# =========================
# 数据模型
# =========================

class SimTarget:
    def __init__(self, track_id: int, lat: float, lon: float, alt_m: float, speed_mps: float, heading_deg: float,
                 target_type: int = 1, size: int = 0, intensity_db: float = 20.0):
        self.track_id = track_id
        self.lat = lat
        self.lon = lon
        self.alt = alt_m
        self.base_alt = alt_m
        self.speed = speed_mps
        self.heading = heading_deg
        self.target_type = target_type  # 0未知,1旋翼无人机,2固定翼无人机...
        self.size = size  # 0小,1中,2大,3特大
        self.intensity = intensity_db
        # 内部相位用于平滑起伏
        self._phase = random.uniform(0.0, 6.28318)
        # 根据类型决定速度上下限，便于后续 step 中约束
        def _speed_range_for_type(t: int):
            # 返回 (min, max) m/s
            mapping = {
                0: (0.0, 400.0),
                1: (0.0, 25.0),
                2: (10.0, 60.0),
                3: (30.0, 120.0),
                4: (10.0, 80.0),
                5: (100.0, 250.0),
            }
            return mapping.get(t, (0.0, 400.0))
        self._speed_min, self._speed_max = _speed_range_for_type(self.target_type)

    def step(self, dt: float, center_lat: float, center_lon: float, leash_m: float):
        """不规则运动：随机游走 + 超出半径时朝中心回摆，保持高度轻微起伏。
        - dt: 步长（秒）
        - center_*: 雷达中心经纬
        - leash_m: 目标允许离中心的最大半径（米）
        """
        # 速度轻微扰动并限幅：扰动幅度与类型相关（无人机更小，飞机更大）
        type_scale = 1.0
        if self.target_type == 1:
            type_scale = 0.4
        elif self.target_type == 2:
            type_scale = 0.8
        elif self.target_type == 3:
            type_scale = 1.5
        elif self.target_type == 4:
            type_scale = 1.0
        elif self.target_type == 5:
            type_scale = 2.0

        self.speed += random.uniform(-0.8, 0.8) * max(dt, 0.05) * type_scale
        # 最小速度不应小于0，且尊重类型定义的上下限
        self.speed = clamp(self.speed, max(0.0, self._speed_min), self._speed_max)

        # 当前到中心的方位与距离（从目标->中心）
        brg_to_center, dist_to_center = bearing_distance(self.lat, self.lon, center_lat, center_lon)

        # 随机转向（度/秒），配合dt —— 自由游走，不在边缘强制回摆
        drift = random.uniform(-15.0, 15.0) * dt
        self.heading = normalize_angle_deg(self.heading + drift)

        # 高度轻微起伏并限幅
        self._phase += dt * 0.6
        alt_jitter = math.sin(self._phase) * 0.5 + random.uniform(-0.3, 0.3)
        self.alt = clamp(self.alt + alt_jitter, self.base_alt - 30.0, self.base_alt + 60.0)

        # 按当前航向推进
        dist = self.speed * dt
        self.lat, self.lon = dest_from_bearing(self.lat, self.lon, self.heading, dist)

# =========================
# 雷达仿真器
# =========================

class RadarSimulator:
    def __init__(
        self,
        dest_ip: str,
        dest_port: int,
        bind_ip: str,
        bind_port: int,
        checksum_mode: int = DEFAULT_CHECK_MODE,
        device_model: int = DEVICE_MODEL,
        device_id: int = DEFAULT_DEVICE_ID,
        status_hz: float = 50.0,
        send_tracks: bool = True,
    track_hz: float = 5.0,
    targets_count: int = 1,
    startup_state: str = 'retract',
    leash_m: float = 2000.0,
    ):
        self.dest = (dest_ip, dest_port)
        self.bind = (bind_ip, bind_port)
        self.checksum_mode = checksum_mode
        self.device_model = device_model
        self.device_id = device_id & 0xFFFF
        self.seq = 0
        self.counter = 0
        self.lock = threading.Lock()

        # logger
        self.logger = logging.getLogger('RadarSimulator')

        # 雷达状态
        state_map = {
            'standby': RADAR_STATE_STANDBY,
            'search' : RADAR_STATE_SEARCH,
            'track'  : RADAR_STATE_TRACK,
            'retract': RADAR_STATE_STANDBY,
        }
        self.work_state = state_map.get(startup_state, RADAR_STATE_STANDBY)
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.freq_ghz = 15.80
        self.antenna_power_mode = 1  # 0待机,1只收,2只发,3收发
        self.detect_range_m = 5000
        self.inu_valid = 1
        self.sim_mode = 0
        self.retract_state = 0
        self.drive_state = 0
        # 电子锁（True=锁定，False=解锁）
        self.electronic_lock = True

        # 如果以 'retract' 启动，则设置为撤收态（回转台0、上锁）
        if startup_state == 'retract':
            self.retract_state = 1
            self.electronic_lock = True
            self.yaw = 0.0

        # 地理位置（可根据需要设置）
        self.radar_lon = 116.3913
        self.radar_lat = 39.9075
        self.radar_alt = 50.0

        # 静默区
        self.silent_start = 0
        self.silent_end = 0

        # 网络
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(self.bind)
            self.sock.settimeout(0.5)
            self.logger.info(f'Bound UDP socket to {self.bind[0]}:{self.bind[1]}')
        except Exception as e:
            self.logger.exception(f'Failed to bind UDP socket to {self.bind}: {e}')
            raise

        # 已知对端集合（用于提示新连接）
        self.known_peers = set()

        # 发送线程配置
        self.status_interval = 1.0 / status_hz if status_hz > 0 else 0
        self.track_interval = 1.0 / track_hz if track_hz > 0 else 0
        self.send_tracks = send_tracks
        self.bulk_report = False
        self.leash_m = float(leash_m)

        # 目标
        self.targets: List[SimTarget] = []
        # \u76ee\u6807 (\u521d\u59cb\u4f4d\u7f6e\u6216\u8005\u663e\u793a\u4f4d\u7f6e\u53ef\u4e3a\u968f\u673a)
        # always create targets when requested (even if we might not send track packets)
        if targets_count > 0:
            for i in range(targets_count):
                # place targets near the 5km edge (or leash limit if smaller)
                ang = random.uniform(0.0, 360.0)
                edge_dist = min(float(self.leash_m), 5000.0)
                # put target close to the edge within ~50m inward jitter (4950-5000m)
                dist = random.uniform(max(0.0, edge_dist - 50.0), edge_dist)
                lat, lon = dest_from_bearing(self.radar_lat, self.radar_lon, ang, dist)
                # initial heading: point toward radar center with small jitter
                heading_to_center = bearing_distance(lat, lon, self.radar_lat, self.radar_lon)[0]
                init_heading = normalize_angle_deg(heading_to_center + random.uniform(-10.0, 10.0))
                ttype = random.choice([0x00, 0x01, 0x02, 0x03, 0x04, 0x05])
                init_speed = sample_speed_for_type(ttype)
                self.targets.append(
                    SimTarget(
                        track_id=i + 1,
                        lat=lat,
                        lon=lon,
                        alt_m=self.radar_alt + 100.0 + random.uniform(-5.0, 5.0) + 5.0 * i,
                        speed_mps=init_speed,
                        heading_deg=init_heading,
                        target_type=ttype,
                        size=random.choice([0x00, 0x01, 0x02, 0x03]),
                        intensity_db=25.0,
                    )
                )

        # 下一个可用 track id
        self.next_track_id = targets_count + 1

        # continuous movement thread (targets move regardless of SEARCH/TRACK state)
        self.t_move = threading.Thread(target=self._move_loop, daemon=True)

        # 线程
        self._stop = threading.Event()
        self.t_rx = threading.Thread(target=self._rx_loop, daemon=True)
        self.t_status = threading.Thread(target=self._status_loop, daemon=True)
        self.t_track = threading.Thread(target=self._track_loop, daemon=True)

    def enable_bulk_report(self, enable: bool):
        self.bulk_report = bool(enable)

    # ============== 打包帧头与报文 ==============

    def _build_header(self, msg_id: int, ext_msg_id: int, total_len: int, seq: Optional[int] = None) -> bytes:
        if seq is None:
            with self.lock:
                seq = self.seq
                self.seq = (self.seq + 1) & 0xFF
        with self.lock:
            self.counter = (self.counter + 1) & 0xFFFFFFFF
            counter = self.counter
        # frame_head_t 32 bytes
        # 4s H H Q H H H H H B B I
        # magic, total_len, device_model, utc_ms, msg_id, ext_id, dev_id, ext_dev_id,
        # reserved, check_mode, seq, counter
        return struct.pack(
            '<4sHHQHHHHHBBI',
            MAGIC,
            u16(total_len),
            u16(self.device_model),
            u32(now_millis()) | ((now_millis() & 0xFFFFFFFF) << 0),  # Q, 使用毫秒；简易
            u16(msg_id),
            u16(ext_msg_id & 0xFFFF),
            u16(self.device_id),
            0,             # 外部设备ID
            0,             # 保留
            u8(self.checksum_mode),
            u8(seq),
            u32(counter)
        )

    def _finalize_packet(self, header: bytes, body_without_checksum: bytes) -> bytes:
        # 计算总长（含校验位2字节）
        total_len = len(header) + len(body_without_checksum) + 2
        # 重建header以写入total_len
        # 解析当前header除total_len外的字段
        # header结构: 4s H H Q H H H H H B B I = 32 bytes
        fields = list(struct.unpack('<4sHHQHHHHHBBI', header))
        fields[1] = u16(total_len)
        header = struct.pack('<4sHHQHHHHHBBI', *fields)

        packet_no_crc = header + body_without_checksum
        if self.checksum_mode == 2:
            chk = crc16_modbus(packet_no_crc)
        elif self.checksum_mode == 1:
            chk = checksum16_sum(packet_no_crc)
        else:
            chk = 0
        return packet_no_crc + struct.pack('<H', u16(chk))

    # ============== 报文体构造 ==============

    def _build_ack(self, resp_to_msg_id: int, result: int, seq_from_cmd: int) -> bytes:
        # body: uint16 resp_msg_id, int8 result, uint8[16] reserved
        body = struct.pack('<Hb', u16(resp_to_msg_id), result) + bytes(16)
        header = self._build_header(MSG_ACK, 0, total_len=32 + len(body) + 2, seq=seq_from_cmd)
        return self._finalize_packet(header, body)

    def _build_status_body(self) -> bytes:
        # 根据协议表构造，省略未指定字段逻辑，填0
        # 异常代码4B
        hw_sw_fault = bytes([0x00, 0x00, 0x00, 0x00])
        # uint8 工作状态
        # uint8 预留
        # uint16 探测范围
        # uint8 惯导有效
        # uint8 模拟状态
        # uint8 撤收状态
        # uint8 行驶状态
        part1 = struct.pack('<4sBBHBBBB',
                            hw_sw_fault,
                            u8(self.work_state),
                            0,
                            u16(self.detect_range_m),
                            u8(self.inu_valid),
                            u8(self.sim_mode),
                            u8(self.retract_state),
                            u8(self.drive_state))
        # 经度、纬度、海拔、偏航、俯仰、滚转
        part2 = struct.pack('<ddffff',
                            float(self.radar_lon),
                            float(self.radar_lat),
                            float(self.radar_alt),
                            float(normalize_angle_deg(self.yaw)),
                            float(self.pitch),
                            float(self.roll))
        # 预留8 + 版本预留21 + 信处预留32
        part3 = bytes(8) + bytes(21) + bytes(32)
        # 频点 float
        # 天线上电模式 uint8
        part4 = struct.pack('<fB', float(self.freq_ghz), u8(self.antenna_power_mode))
        # 天线预留16 + 信道预留8 + 伺服预留16
        part5 = bytes(16 + 8 + 16)
        # 静默区起止
        part6 = struct.pack('<HH', u16(self.silent_start), u16(self.silent_end))
        # 预留12
        part7 = bytes(12)
        return part1 + part2 + part3 + part4 + part5 + part6 + part7

    def _build_status_packet(self) -> bytes:
        body = self._build_status_body()
        header = self._build_header(MSG_STATUS, 0, total_len=32 + len(body) + 2)
        return self._finalize_packet(header, body)

    def _pack_track_info(self, t: SimTarget) -> bytes:
        # 航迹信息结构体
        # uint16 track_id
        # double tgt_lon, double tgt_lat, float tgt_alt
        # float dist, float az, float el
        # float speed, float course, float intensity
        # uint8[4] reserved
        # uint8 type, uint8 size, uint8 point_type, uint8 track_type
        # uint8 lost_cnt, uint8 quality
        # float raw_dist, float raw_az, float raw_el
        # uint8[3] reserved
        # 地理换算
        az_deg, dist_m = bearing_distance(self.radar_lat, self.radar_lon, t.lat, t.lon)
        horiz = max(0.01, math.sqrt(max(dist_m**2 - (t.alt - self.radar_alt)**2, 0)))
        el_deg = math.degrees(math.atan2(t.alt - self.radar_alt, horiz))
        packed = []
        packed.append(struct.pack('<H', u16(t.track_id)))
        packed.append(struct.pack('<ddf', float(t.lon), float(t.lat), float(t.alt)))
        packed.append(struct.pack('<fff', float(dist_m), float(az_deg), float(el_deg)))
        packed.append(struct.pack('<fff', float(t.speed), float(normalize_angle_deg(t.heading)), float(t.intensity)))
        packed.append(bytes(4))
        packed.append(struct.pack('<BBBB', u8(t.target_type), u8(t.size), 0, 1))  # 检测点/搜索航迹
        packed.append(struct.pack('<BB', 0, 90))  # 丢失次数、质量%
        packed.append(struct.pack('<fff', float(dist_m), float(az_deg), float(el_deg)))
        packed.append(bytes(3))
        return b''.join(packed)

    def _build_track_packet(self, t: SimTarget) -> bytes:
        # 头部 + 航迹报文体（见表22/表23）
        body = build_track_body(self.inu_valid, self.radar_lon, self.radar_lat, self.radar_alt, t)
        header = self._build_header(MSG_TRACK, 0, total_len=32 + len(body) + 2)
        return self._finalize_packet(header, body)

    def _build_silent_feedback_packet(self) -> bytes:
        # 复用静默区配置结构作为反馈
        # 帧头 + uint16 start + uint16 end + 预留16
        body = struct.pack('<HH', u16(self.silent_start), u16(self.silent_end)) + bytes(16)
        header = self._build_header(CFG_SILENT_FEEDBACK, 0, total_len=32 + len(body) + 2)
        return self._finalize_packet(header, body)

    # ============== 发送/接收线程 ==============

    def start(self):
        self.t_rx.start()
        self.logger.info('RX thread started')
        if self.status_interval > 0:
            self.t_status.start()
            self.logger.info('Status thread started')
        if self.send_tracks and self.track_interval > 0:
            self.t_track.start()
            self.logger.info('Track thread started')
        # always start movement thread so targets move from program start
        if self.targets:
            self.t_move.start()
            self.logger.info('Move thread started')

    def stop(self):
        self._stop.set()
        try:
            self.sock.close()
        except Exception:
            pass
        # join move thread briefly if running
        try:
            if hasattr(self, 't_move') and self.t_move.is_alive():
                self.t_move.join(timeout=0.5)
        except Exception:
            pass


    def _status_loop(self):
        next_t = time.time()
        while not self._stop.is_set():
            # 简单姿态变化模拟
            self.yaw = normalize_angle_deg(self.yaw + 0.5)
            self.pitch = clamp(self.pitch + random.uniform(-0.02, 0.02), -5.0, 5.0)
            self.roll = clamp(self.roll + random.uniform(-0.03, 0.03), -10.0, 10.0)

            pkt = self._build_status_packet()
            try:
                self.sock.sendto(pkt, self.dest)
                self.logger.info(f'Sent STATUS packet {len(pkt)} bytes to {self.dest}')
            except Exception:
                self.logger.exception('Failed to send STATUS packet')
                pass

            next_t += self.status_interval
            sleep_t = next_t - time.time()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_t = time.time()

    def _track_loop(self):
        next_t = time.time()
        while not self._stop.is_set():
            if self.work_state in (RADAR_STATE_SEARCH, RADAR_STATE_TRACK):
                # 目标推进
                for t in self.targets:
                    t.step(self.track_interval, self.radar_lat, self.radar_lon, self.leash_m)
                    # 只要在静默区内则不发送
                    az_deg, dist_m = bearing_distance(self.radar_lat, self.radar_lon, t.lat, t.lon)
                    az100 = int(round(az_deg * 100.0))
                    if self._in_silent_zone(az100):
                        continue
                    # 如果启用批量上报，则在循环外统一发送
                    # 单目标发送留到后续处理
                    pass

                if self.bulk_report:
                    # 收集非静默目标
                    from track_packet import build_tracks_body
                    visible = []
                    for t in self.targets:
                        az_deg, dist_m = bearing_distance(self.radar_lat, self.radar_lon, t.lat, t.lon)
                        az100 = int(round(az_deg * 100.0))
                        if not self._in_silent_zone(az100):
                            visible.append(t)
                    if visible:
                        body = build_tracks_body(self.inu_valid, self.radar_lon, self.radar_lat, self.radar_alt, visible)
                        header = self._build_header(MSG_TRACK, 0, total_len=32 + len(body) + 2)
                        pkt = self._finalize_packet(header, body)
                        try:
                            self.sock.sendto(pkt, self.dest)
                            self.logger.info(f'Sent BULK TRACK packet with {len(visible)} tracks {len(pkt)} bytes to {self.dest}')
                        except Exception:
                            self.logger.exception('Failed to send BULK TRACK packet')
                            pass
                else:
                    # 单目标逐个发送（保持原行为）
                    for t in self.targets:
                        az_deg, dist_m = bearing_distance(self.radar_lat, self.radar_lon, t.lat, t.lon)
                        az100 = int(round(az_deg * 100.0))
                        if self._in_silent_zone(az100):
                            continue
                        pkt = self._build_track_packet(t)
                        try:
                            self.sock.sendto(pkt, self.dest)
                            self.logger.info(f'Sent TRACK packet for track_id={t.track_id} {len(pkt)} bytes to {self.dest}')
                        except Exception:
                            self.logger.exception('Failed to send TRACK packet')
                            pass
            next_t += self.track_interval
            sleep_t = next_t - time.time()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_t = time.time()

    def _move_loop(self):
        """Continuously advance all targets regardless of radar state.
        Runs at a fixed small timestep to make motion smooth.
        """
        move_hz = 10.0
        move_dt = 1.0 / move_hz
        next_t = time.time()
        while not self._stop.is_set():
            if self.targets:
                for t in self.targets:
                    try:
                        t.step(move_dt, self.radar_lat, self.radar_lon, self.leash_m)
                    except Exception:
                        self.logger.exception('Error while moving target')
            next_t += move_dt
            sleep_t = next_t - time.time()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_t = time.time()

    def _in_silent_zone(self, az_centideg: int) -> bool:
        # [start, end] 顺时针范围。若 start <= end: [start, end]
        # 若 start > end: [start, 36000] U [0, end]
        s = self.silent_start % 36000
        e = self.silent_end % 36000
        a = az_centideg % 36000
        if s <= e:
            return s <= a <= e
        else:
            return a >= s or a <= e

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                data, addr = self.sock.recvfrom(4096)
                self.logger.info(f'Received {len(data)} bytes from {addr}')
            except socket.timeout:
                continue
            except Exception:
                self.logger.exception('Socket recv failed')
                break

            # 新对端检测（UDP 无连接，但可提示首次出现的 addr）
            if addr not in self.known_peers:
                self.known_peers.add(addr)
                self.logger.info(f'New peer seen: {addr}')
            # 打印前 32 字节的十六进制摘要，帮助调试接收到的内容
            try:
                short = data[:32]
                hx = ' '.join(f'{b:02X}' for b in short)
                self.logger.info(f'Payload head ({len(short)} bytes): {hx}{" ..." if len(data)>32 else ""}')
            except Exception:
                pass
            if len(data) < 32:
                continue
            # 校验帧头
            try:
                magic, total_len, dev_model, utc_ms, msg_id, ext_id, dev_id, ext_dev_id, reserved, chk_mode, seq, counter = \
                    struct.unpack('<4sHHQHHHHHBBI', data[:32])
            except struct.error:
                continue
            if magic != MAGIC:
                continue
            if total_len != len(data):
                # 长度字段与实际收到的数据长度不一致，之前策略是直接丢弃。
                # 改为记录警告并继续处理（在许多测试客户端中 total_len 计算/填充不严格，
                # 导致仿真器无响应），以便能更宽容地兼容测试工具并输出诊断信息。
                self.logger.warning(f'Header total_len mismatch from {addr}: header={total_len} actual={len(data)} -- will continue processing for diagnostics')
            # 验证校验
            if chk_mode == 2:
                expect = struct.unpack('<H', data[-2:])[0]
                calc = crc16_modbus(data[:-2])
                if expect != calc:
                    self.logger.warning(f'CRC mismatch from {addr}: expect=0x{expect:04X} calc=0x{calc:04X}')
                    continue
            elif chk_mode == 1:
                expect = struct.unpack('<H', data[-2:])[0]
                calc = checksum16_sum(data[:-2])
                if expect != calc:
                    self.logger.warning(f'Checksum mismatch from {addr}: expect=0x{expect:04X} calc=0x{calc:04X}')
                    continue
            # log parsed header
            self.logger.debug(f'Parsed header from {addr}: msg_id=0x{msg_id:04X} seq={seq} chk_mode={chk_mode} total_len={total_len}')
            # 处理命令
            body = data[32:-2] if len(data) >= 34 else b''

            if msg_id == CMD_STANDBY:
                # 表13：body 首字节为任务类型(uint8)，默认0；后续16字节保留
                self.logger.debug(f'STANDBY body len={len(body)} from {addr} data={body.hex()}')
                if len(body) == 0:
                    task_type = 0
                    self.logger.info(f'Empty STANDBY body received from {addr}, defaulting task_type=0')
                elif len(body) >= 1:
                    task_type = body[0]
                else:
                    self.logger.warning(f'SHORT STANDBY body from {addr}, len={len(body)}')
                    self._send_ack(msg_id, addr, seq, result=0)
                    continue

                if task_type == 0:
                    self.work_state = RADAR_STATE_STANDBY
                    self._send_ack(msg_id, addr, seq, result=1)
                    self.logger.info(f'Enter STANDBY mode (task_type=0) from {addr}; track sending will stop')
                else:
                    self.logger.warning(f'Unsupported STANDBY task_type={task_type} from {addr}')
                    self._send_ack(msg_id, addr, seq, result=0)
            elif msg_id == CMD_SEARCH:
                # 按照表4.1.2: body 首字节为任务类型(uint8)，后续16字节保留
                self.logger.debug(f'SEARCH body len={len(body)} from {addr} data={body.hex()}')
                # 先解析或推断 task_type（兼容性：空 body 视作 task_type=1）
                if len(body) == 0:
                    task_type = 1
                    self.logger.info(f'Empty SEARCH body received from {addr}, defaulting task_type=1')
                elif len(body) >= 1:
                    task_type = body[0]
                else:
                    self.logger.warning(f'SHORT SEARCH body from {addr}, len={len(body)}')
                    self._send_ack(msg_id, addr, seq, result=0)
                    continue

                # 仅支持任务类型1作为有效的搜索任务
                if task_type == 1:
                    self.work_state = RADAR_STATE_SEARCH
                    self._send_ack(msg_id, addr, seq, result=1)
                    self.logger.info(f'Enter SEARCH mode (task_type=1) from {addr}')
                    # 进入搜索态后立即上传一次航迹（如有目标）
                    if self.targets:
                        try:
                            pkt = self._build_track_packet(self.targets[0])
                            self.sock.sendto(pkt, addr)
                            self.logger.info(f'Immediate TRACK packet sent (SEARCH) for track_id={self.targets[0].track_id} to {addr}')
                        except Exception:
                            self.logger.exception('Failed to send immediate TRACK packet on SEARCH')
                else:
                    # 不支持的 task_type，回复失败 ACK
                    self.logger.warning(f'Unsupported SEARCH task_type={task_type} from {addr}')
                    self._send_ack(msg_id, addr, seq, result=0)
            elif msg_id == CMD_TRACK:
                # 简化处理：切换状态为跟踪
                self.work_state = RADAR_STATE_TRACK
                self._send_ack(msg_id, addr, seq, result=1)
                # 切入跟踪态立刻上传一次航迹
                if self.targets:
                    try:
                        pkt = self._build_track_packet(self.targets[0])
                        self.sock.sendto(pkt, addr)
                        self.logger.info(f'Immediate TRACK packet sent (TRACK) for track_id={self.targets[0].track_id} to {addr}')
                    except Exception:
                        self.logger.exception('Failed to send immediate TRACK packet on TRACK')
            elif msg_id == CFG_SILENT_SET:
                # body: uint16 start, uint16 end, reserved...
                if len(body) >= 4:
                    self.silent_start, self.silent_end = struct.unpack('<HH', body[:4])
                self._send_ack(msg_id, addr, seq, result=1)
                # 亦可立即反馈CFG_SILENT_FEEDBACK
                fb = self._build_silent_feedback_packet()
                try:
                    self.sock.sendto(fb, addr)
                    self.logger.info(f'Sent CFG_SILENT_FEEDBACK to {addr} len={len(fb)}')
                except Exception:
                    self.logger.exception('Failed to send CFG_SILENT_FEEDBACK')
                    pass
            elif msg_id == CMD_DEPLOY:
                # 表17：body 首字节为任务类型(uint8)，0x01 展开，0x00 撤收，后续16字节保留
                self.logger.debug(f'DEPLOY body len={len(body)} from {addr} data={body.hex()}')
                if len(body) == 0:
                    task_type = 0
                    self.logger.info(f'Empty DEPLOY body received from {addr}, defaulting task_type=0 (retract)')
                elif len(body) >= 1:
                    task_type = body[0]
                else:
                    self.logger.warning(f'SHORT DEPLOY body from {addr}, len={len(body)}')
                    self._send_ack(msg_id, addr, seq, result=0)
                    continue

                if task_type == 1:
                    # 展开：解锁转台电子锁
                    self.electronic_lock = False
                    self.retract_state = 0
                    self._send_ack(msg_id, addr, seq, result=1)
                    self.logger.info(f'DEPLOY: expand received from {addr}, electronic_lock=UNLOCKED')
                    # 发送一次状态包以反馈当前状态
                    try:
                        pkt = self._build_status_packet()
                        self.sock.sendto(pkt, addr)
                        self.logger.info(f'Sent STATUS packet (deploy expand) to {addr} len={len(pkt)}')
                    except Exception:
                        self.logger.exception('Failed to send STATUS packet after DEPLOY expand')
                elif task_type == 0:
                    # 撤收：置为待机，转台回0位，并上锁
                    self.work_state = RADAR_STATE_STANDBY
                    self.yaw = 0.0
                    self.electronic_lock = True
                    self.retract_state = 1
                    self._send_ack(msg_id, addr, seq, result=1)
                    self.logger.info(f'DEPLOY: retract received from {addr}, electronic_lock=LOCKED, yaw reset to 0, state=STANDBY')
                    try:
                        pkt = self._build_status_packet()
                        self.sock.sendto(pkt, addr)
                        self.logger.info(f'Sent STATUS packet (deploy retract) to {addr} len={len(pkt)}')
                    except Exception:
                        self.logger.exception('Failed to send STATUS packet after DEPLOY retract')
                else:
                    self.logger.warning(f'Unsupported DEPLOY task_type={task_type} from {addr}')
                    self._send_ack(msg_id, addr, seq, result=0)
            elif msg_id == MSG_QUERY:
                # 参数查询：body前2字节为待查询报文ID
                if len(body) >= 2:
                    qid = struct.unpack('<H', body[:2])[0]
                    if qid == CFG_SILENT_FEEDBACK:
                        pkt = self._build_silent_feedback_packet()
                        self.sock.sendto(pkt, addr)
                    elif qid == MSG_STATUS:
                        pkt = self._build_status_packet()
                        self.sock.sendto(pkt, addr)
                    # 其他查询可在此扩展
            else:
                # 未覆盖的指令，做一个收到但未处理的ACK
                self._send_ack(msg_id, addr, seq, result=0)

    def _send_ack(self, resp_to_msg_id: int, addr, seq_from_cmd: int, result: int = 1):
        pkt = self._build_ack(resp_to_msg_id, result, seq_from_cmd)
        try:
            self.sock.sendto(pkt, addr)
            self.logger.info(f'Sent ACK for msg=0x{resp_to_msg_id:04X} to {addr} seq={seq_from_cmd} result={result} len={len(pkt)}')
        except Exception:
            self.logger.exception('Failed to send ACK')
            pass

# =========================
# 入口
# =========================

def main():
    # configure simple logging to console
    logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(levelname)s %(name)s: %(message)s')
    ap = argparse.ArgumentParser(description='iEye 教育系列 雷达仿真器 (UDP)')
    ap.add_argument('--dest-ip', default='127.0.0.1', help='指挥中心IP')
    def parse_port(s: str) -> int:
        try:
            return int(s, 0)  # 支持十进制或0x前缀
        except Exception:
            return int(s)

    ap.add_argument('--dest-port', type=parse_port, default=int('0x1999', 16), help='指挥中心端口，支持十进制或0x前缀')
    ap.add_argument('--bind', default='0.0.0.0', help='本地绑定IP')
    ap.add_argument('--bind-port', type=parse_port, default=int('0x1888', 16), help='雷达本地端口，支持十进制或0x前缀')
    ap.add_argument('--device-id', type=lambda x: int(x, 0), default=DEFAULT_DEVICE_ID, help='雷达设备ID（uint16）')
    ap.add_argument('--device-model', type=int, default=DEVICE_MODEL, help='设备型号')
    ap.add_argument('--check', type=int, choices=[0,1,2], default=DEFAULT_CHECK_MODE, help='校验模式：0无 1和校验 2CRC16')
    ap.add_argument('--status-hz', type=float, default=0.2, help='状态上报频率Hz（Hz，默认0.2，即每5秒上报一次）')
    ap.add_argument('--tracks-hz', type=float, default=5.0, help='航迹上报频率Hz')
    ap.add_argument('--no-tracks', action='store_true', help='不发送航迹')
    ap.add_argument('--targets', type=int, default=2, help='启动时生成的目标数量（默认2）')
    ap.add_argument('--leash-m', type=float, default=5000.0, help='目标相对雷达允许的最大半径（米），超出将回摆')
    ap.add_argument('--startup-state', choices=['standby', 'search', 'track', 'retract'], default='retract', help='启动时的工作状态')
    args = ap.parse_args()

    sim = RadarSimulator(dest_ip=args.dest_ip,
                         dest_port=args.dest_port,
                         bind_ip=args.bind,
                         bind_port=args.bind_port,
                         checksum_mode=args.check,
                         device_model=args.device_model,
                         device_id=args.device_id,
                         status_hz=args.status_hz,
                         send_tracks=not args.no_tracks,
                         track_hz=args.tracks_hz,
                         targets_count=args.targets,
                         startup_state=args.startup_state,
                         leash_m=args.leash_m)
    print(f'Radar Simulator started. Send to {args.dest_ip}:{args.dest_port}, listen on {args.bind}:{args.bind_port}')
    print('Press Ctrl+C to stop.')
    sim.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        sim.stop()
        print('Stopped.')

if __name__ == '__main__':
    main()
