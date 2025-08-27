import argparse
import math
import socket
import struct
import threading
import time
import random
from typing import Tuple, List, Optional
import logging

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
        self.speed = speed_mps
        self.heading = heading_deg
        self.target_type = target_type  # 0未知,1旋翼无人机,2固定翼无人机...
        self.size = size  # 0小,1中,2大,3特大
        self.intensity = intensity_db

    def step(self, dt: float):
        # 简单匀速直线，添加轻微随机摆动
        self.heading = normalize_angle_deg(self.heading + random.uniform(-0.2, 0.2))
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
        self.work_state = RADAR_STATE_STANDBY
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

        # 目标
        self.targets: List[SimTarget] = []
        if self.send_tracks:
            # 默认1个小型旋翼无人机，速度15m/s，北向
            self.targets.append(
                SimTarget(
                    track_id=1,
                    lat=self.radar_lat + 0.002,
                    lon=self.radar_lon + 0.002,
                    alt_m=self.radar_alt + 120.0,
                    speed_mps=15.0,
                    heading_deg=0.0,
                    target_type=1,
                    size=0,
                    intensity_db=25.0,
                )
            )

        # 线程
        self._stop = threading.Event()
        self.t_rx = threading.Thread(target=self._rx_loop, daemon=True)
        self.t_status = threading.Thread(target=self._status_loop, daemon=True)
        self.t_track = threading.Thread(target=self._track_loop, daemon=True)

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
        # 头部 + 惯导有效 + 雷达经纬高 + 航迹信息*1 + 预留16
        body = struct.pack('<Bddf', u8(self.inu_valid), float(self.radar_lon), float(self.radar_lat), float(self.radar_alt))
        body += self._pack_track_info(t)
        body += bytes(16)
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

    def stop(self):
        self._stop.set()
        try:
            self.sock.close()
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
                    t.step(self.track_interval)
                    # 只要在静默区内则不发送
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
                # 长度不符，忽略
                continue
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
                self.work_state = RADAR_STATE_STANDBY
                self._send_ack(msg_id, addr, seq, result=1)
            elif msg_id == CMD_SEARCH:
                self.work_state = RADAR_STATE_SEARCH
                self._send_ack(msg_id, addr, seq, result=1)
            elif msg_id == CMD_TRACK:
                # 简化处理：切换状态为跟踪
                self.work_state = RADAR_STATE_TRACK
                self._send_ack(msg_id, addr, seq, result=1)
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
    ap.add_argument('--dest-port', type=int, default=int('0x1999', 16), help='指挥中心端口，默认0x1999')
    ap.add_argument('--bind', default='0.0.0.0', help='本地绑定IP')
    ap.add_argument('--bind-port', type=int, default=int('0x1888', 16), help='雷达本地端口，默认0x1888')
    ap.add_argument('--device-id', type=lambda x: int(x, 0), default=DEFAULT_DEVICE_ID, help='雷达设备ID（uint16）')
    ap.add_argument('--device-model', type=int, default=DEVICE_MODEL, help='设备型号')
    ap.add_argument('--check', type=int, choices=[0,1,2], default=DEFAULT_CHECK_MODE, help='校验模式：0无 1和校验 2CRC16')
    ap.add_argument('--status-hz', type=float, default=0.2, help='状态上报频率Hz（Hz，默认0.2，即每5秒上报一次）')
    ap.add_argument('--tracks-hz', type=float, default=5.0, help='航迹上报频率Hz')
    ap.add_argument('--no-tracks', action='store_true', help='不发送航迹')
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
                         track_hz=args.tracks_hz)
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
