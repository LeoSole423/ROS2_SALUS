#!/usr/bin/env python3

import base64
import json
import socket
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray

RTCM_BUFFER_SIZE = 4096
RTCM_MIN_FRAME_BYTES = 6
NTRIP_HEADER_MAX_BYTES = 16384


@dataclass(frozen=True)
class RtkSource:
    id: str
    label: str
    host: str
    port: int
    mountpoint: str
    username: str
    password: str


class RtkSourceManager(Node):
    def __init__(self) -> None:
        super().__init__("rtk_source_manager")

        default_sources_path = str(
            Path(get_package_share_directory("sensores"))
            / "config"
            / "rtk_sources.yaml"
        )

        self.declare_parameter("sources_config", default_sources_path)
        self.declare_parameter("active_source_id", "")
        self.declare_parameter("rtcm_topic", "/rtcm")
        self.declare_parameter("source_select_topic", "/gps/rtk_source/select")
        self.declare_parameter("source_manage_topic", "/gps/rtk_source/manage_json")
        self.declare_parameter("sources_topic", "/gps/rtk_sources/json")
        self.declare_parameter("source_status_topic", "/gps/rtk_source/status_json")
        self.declare_parameter("status_period_s", 1.0)
        self.declare_parameter("connect_timeout_s", 5.0)
        self.declare_parameter("read_timeout_s", 2.0)
        self.declare_parameter("reconnect_delay_s", 2.0)

        sources_config = str(self.get_parameter("sources_config").value)
        self.rtcm_topic = str(self.get_parameter("rtcm_topic").value)
        self.source_select_topic = str(
            self.get_parameter("source_select_topic").value
        )
        self.source_manage_topic = str(
            self.get_parameter("source_manage_topic").value
        )
        self.sources_topic = str(self.get_parameter("sources_topic").value)
        self.source_status_topic = str(
            self.get_parameter("source_status_topic").value
        )
        self._status_period_s = float(self.get_parameter("status_period_s").value)
        self._connect_timeout_s = float(
            self.get_parameter("connect_timeout_s").value
        )
        self._read_timeout_s = float(self.get_parameter("read_timeout_s").value)
        self._reconnect_delay_s = float(
            self.get_parameter("reconnect_delay_s").value
        )

        self._sources_config_path = Path(sources_config)
        self._sources = self._load_sources(self._sources_config_path)
        self._sources_by_id = {source.id: source for source in self._sources}
        if not self._sources_by_id:
            raise RuntimeError(f"No RTK sources found in {sources_config}")

        initial_source_id = str(self.get_parameter("active_source_id").value).strip()
        if not initial_source_id or initial_source_id not in self._sources_by_id:
            initial_source_id = self._sources[0].id

        self._data_lock = threading.Lock()
        self._active_source_id = initial_source_id
        self._connected = False
        self._last_error = ""
        self._last_rtcm_time_s: Optional[float] = None
        self._received_count = 0
        self._last_message_size = 0
        self._socket: Optional[socket.socket] = None
        self._wake_connect = threading.Event()
        self._stop_event = threading.Event()

        self._rtcm_pub = self.create_publisher(UInt8MultiArray, self.rtcm_topic, 10)
        self._sources_pub = self.create_publisher(String, self.sources_topic, 2)
        self._status_pub = self.create_publisher(String, self.source_status_topic, 2)
        self.create_subscription(
            String, self.source_select_topic, self._select_source_cb, 10
        )
        self.create_subscription(
            String, self.source_manage_topic, self._manage_source_cb, 10
        )

        self.create_timer(self._status_period_s, self._publish_metadata)

        self._worker = threading.Thread(
            target=self._reader_loop, name="rtk_source_manager", daemon=True
        )
        self._worker.start()

        self.get_logger().info(
            "RTK source manager active: "
            f"{len(self._sources)} source(s), active={self._active_source_id}, "
            f"rtcm_topic={self.rtcm_topic}"
        )

    def destroy_node(self) -> bool:
        self._stop_event.set()
        self._wake_connect.set()
        self._close_socket()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()

    def _load_sources(self, path: Path) -> list[RtkSource]:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        raw_sources = data.get("sources") or []
        sources: list[RtkSource] = []
        for raw in raw_sources:
            try:
                sources.append(
                    RtkSource(
                        id=str(raw["id"]).strip(),
                        label=str(raw.get("label") or raw["id"]).strip(),
                        host=str(raw["host"]).strip(),
                        port=int(raw.get("port", 2101)),
                        mountpoint=str(raw["mountpoint"]).strip(),
                        username=str(raw.get("username", "")).strip(),
                        password=str(raw.get("password", "")).strip(),
                    )
                )
            except Exception as exc:
                raise RuntimeError(
                    f"Invalid RTK source entry in {path}: {raw}"
                ) from exc
        return sources

    def _serialize_sources_locked(self) -> list[dict]:
        return [
            {
                "id": source.id,
                "label": source.label,
                "host": source.host,
                "port": source.port,
                "mountpoint": source.mountpoint,
            }
            for source in self._sources
        ]

    def _write_sources_locked(self) -> None:
        payload = {
            "sources": [
                {
                    "id": source.id,
                    "label": source.label,
                    "host": source.host,
                    "port": source.port,
                    "mountpoint": source.mountpoint,
                    "username": source.username,
                    "password": source.password,
                }
                for source in self._sources
            ]
        }
        self._sources_config_path.parent.mkdir(parents=True, exist_ok=True)
        self._sources_config_path.write_text(
            yaml.safe_dump(payload, sort_keys=False, allow_unicode=False),
            encoding="utf-8",
        )

    def _replace_sources_locked(self, sources: list[RtkSource]) -> None:
        self._sources = list(sources)
        self._sources_by_id = {source.id: source for source in self._sources}

    def _parse_upsert_source_locked(self, payload: dict) -> RtkSource:
        source_id = str(payload.get("id") or "").strip()
        if not source_id:
            raise ValueError("missing_id")

        existing = self._sources_by_id.get(source_id)
        label = str(
            payload.get("label")
            or (existing.label if existing else source_id)
        ).strip()
        host = str(payload.get("host") or (existing.host if existing else "")).strip()
        mountpoint = str(
            payload.get("mountpoint") or (existing.mountpoint if existing else "")
        ).strip()

        raw_port = payload.get("port", existing.port if existing else 2101)
        port = int(raw_port)
        if port <= 0:
            raise ValueError("invalid_port")
        if not host:
            raise ValueError("missing_host")
        if not mountpoint:
            raise ValueError("missing_mountpoint")

        username = payload.get("username", None)
        if username is None or (str(username).strip() == "" and existing is not None):
            username = existing.username if existing else ""
        username = str(username or "").strip()

        password = payload.get("password", None)
        if password is None or (str(password).strip() == "" and existing is not None):
            password = existing.password if existing else ""
        password = str(password or "").strip()

        return RtkSource(
            id=source_id,
            label=label or source_id,
            host=host,
            port=port,
            mountpoint=mountpoint,
            username=username,
            password=password,
        )

    def _publish_metadata(self) -> None:
        with self._data_lock:
            msg_sources = String()
            msg_sources.data = json.dumps(
                {
                    "sources": self._serialize_sources_locked()
                }
            )
            source = self._sources_by_id.get(self._active_source_id)
            rtcm_age_s = None
            if self._last_rtcm_time_s is not None:
                rtcm_age_s = max(0.0, time.monotonic() - self._last_rtcm_time_s)

            payload = {
                "active_source_id": self._active_source_id,
                "active_source_label": source.label if source else None,
                "connected": self._connected,
                "last_error": self._last_error or None,
                "rtcm_age_s": rtcm_age_s,
                "received_count": self._received_count,
                "last_message_size": self._last_message_size,
                "config_path": str(self._sources_config_path),
            }

        self._sources_pub.publish(msg_sources)

        msg_status = String()
        msg_status.data = json.dumps(payload)
        self._status_pub.publish(msg_status)

    def _select_source_cb(self, msg: String) -> None:
        requested_id = str(msg.data).strip()
        if not requested_id:
            return
        if requested_id not in self._sources_by_id:
            self.get_logger().warning(
                f"Ignoring unknown RTK source request: {requested_id}"
            )
            return

        with self._data_lock:
            if requested_id == self._active_source_id:
                return
            self._active_source_id = requested_id
            self._connected = False
            self._last_error = ""

        self.get_logger().info(f"Switching RTK source to {requested_id}")
        self._publish_metadata()
        self._close_socket()
        self._wake_connect.set()

    def _manage_source_cb(self, msg: String) -> None:
        try:
            payload = json.loads(str(msg.data))
        except Exception:
            self.get_logger().warning("Ignoring invalid RTK source management payload")
            return
        if not isinstance(payload, dict):
            self.get_logger().warning("Ignoring RTK source management payload that is not an object")
            return

        action = str(payload.get("action") or "upsert").strip().lower()
        if action not in {"upsert", "delete"}:
            self.get_logger().warning(f"Ignoring unsupported RTK source action: {action}")
            return

        reconnect = False
        try:
            with self._data_lock:
                if action == "delete":
                    source_id = str(payload.get("id") or "").strip()
                    if not source_id:
                        raise ValueError("missing_id")
                    if source_id not in self._sources_by_id:
                        raise ValueError("unknown_id")
                    if len(self._sources) <= 1:
                        raise ValueError("cannot_delete_last_source")

                    remaining = [
                        source for source in self._sources if source.id != source_id
                    ]
                    if self._active_source_id == source_id:
                        self._active_source_id = remaining[0].id
                        reconnect = True
                    self._replace_sources_locked(remaining)
                    self._write_sources_locked()
                    self._last_error = ""
                    self.get_logger().info(f"Deleted RTK source {source_id}")
                else:
                    source = self._parse_upsert_source_locked(payload)
                    activate = bool(payload.get("activate"))
                    existing = self._sources_by_id.get(source.id)
                    if existing is None:
                        new_sources = list(self._sources) + [source]
                        self.get_logger().info(f"Added RTK source {source.id}")
                    else:
                        new_sources = [
                            source if current.id == source.id else current
                            for current in self._sources
                        ]
                        if existing != source:
                            self.get_logger().info(f"Updated RTK source {source.id}")
                    self._replace_sources_locked(new_sources)
                    self._write_sources_locked()
                    if activate and self._active_source_id != source.id:
                        self._active_source_id = source.id
                        reconnect = True
                    if self._active_source_id == source.id:
                        reconnect = True
                    self._last_error = ""
        except Exception as exc:
            self.get_logger().warning(f"RTK source management failed: {exc}")
            with self._data_lock:
                self._last_error = str(exc)
            self._publish_metadata()
            return

        self._publish_metadata()
        if reconnect:
            self._set_connected(False, "")
            self._close_socket()
            self._wake_connect.set()

    def _active_source(self) -> RtkSource:
        with self._data_lock:
            return self._sources_by_id[self._active_source_id]

    def _reader_loop(self) -> None:
        buffer = bytearray()
        while not self._stop_event.is_set():
            source = self._active_source()
            try:
                sock, initial_payload = self._open_stream(source)
                self._set_connected(True, "")
                buffer.clear()
                if initial_payload:
                    buffer.extend(initial_payload)
                    self._consume_rtcm_stream(buffer)

                while not self._stop_event.is_set():
                    if source.id != self._active_source().id:
                        raise InterruptedError("RTK source changed")
                    chunk = sock.recv(RTCM_BUFFER_SIZE)
                    if not chunk:
                        raise ConnectionError("NTRIP stream closed")
                    buffer.extend(chunk)
                    self._consume_rtcm_stream(buffer)
            except InterruptedError:
                pass
            except Exception as exc:
                self._set_connected(False, str(exc))
            finally:
                self._close_socket()

            self._wake_connect.wait(timeout=self._reconnect_delay_s)
            self._wake_connect.clear()

    def _open_stream(self, source: RtkSource) -> tuple[socket.socket, bytes]:
        sock = socket.create_connection(
            (source.host, source.port), timeout=self._connect_timeout_s
        )
        sock.settimeout(self._read_timeout_s)

        auth = base64.b64encode(
            f"{source.username}:{source.password}".encode("utf-8")
        ).decode("ascii")
        request = (
            f"GET /{source.mountpoint} HTTP/1.0\r\n"
            "User-Agent: NTRIP RTKLIB/2.4.3\r\n"
            "Accept: */*\r\n"
            "Connection: close\r\n"
            "Ntrip-Version: Ntrip/2.0\r\n"
            f"Authorization: Basic {auth}\r\n"
            "\r\n"
        )
        sock.sendall(request.encode("ascii"))

        response = bytearray()
        while True:
            chunk = sock.recv(RTCM_BUFFER_SIZE)
            if not chunk:
                if response:
                    raise ConnectionError(
                        f"NTRIP server closed during handshake: "
                        f"{response.decode('latin1', errors='replace')[:160]}"
                    )
                raise ConnectionError("NTRIP server closed before sending headers")
            response.extend(chunk)
            parsed = self._parse_ntrip_response(bytes(response))
            if parsed is not None:
                first_line, payload = parsed
                if "200" not in first_line and "ICY 200 OK" not in first_line:
                    raise ConnectionError(
                        f"NTRIP server rejected source {source.id}: {first_line}"
                    )
                self._socket = sock
                self.get_logger().info(
                    f"NTRIP source connected: {source.label} "
                    f"({source.host}/{source.mountpoint})"
                )
                return sock, payload
            if len(response) > NTRIP_HEADER_MAX_BYTES:
                raise ConnectionError("NTRIP headers too large")

    def _parse_ntrip_response(self, response: bytes) -> Optional[tuple[str, bytes]]:
        if response.startswith(b"ICY 200 OK"):
            line_end = response.find(b"\r\n")
            if line_end == -1:
                line_end = response.find(b"\n")
            if line_end == -1:
                return None
            first_line = response[:line_end].decode("latin1", errors="replace")
            separator_len = 2 if response[line_end: line_end + 2] == b"\r\n" else 1
            return first_line, bytes(response[line_end + separator_len :])

        if response.startswith(b"HTTP/"):
            line_end = response.find(b"\r\n")
            if line_end == -1:
                line_end = response.find(b"\n")
            if line_end == -1:
                return None
            first_line = response[:line_end].decode("latin1", errors="replace")
            separator_len = 2 if response[line_end: line_end + 2] == b"\r\n" else 1
            return first_line, bytes(response[line_end + separator_len :])

        return None

    def _consume_rtcm_stream(self, buffer: bytearray) -> None:
        while len(buffer) >= RTCM_MIN_FRAME_BYTES:
            if buffer[0] != 0xD3:
                buffer.pop(0)
                continue

            payload_length = ((buffer[1] & 0x03) << 8) | buffer[2]
            frame_length = payload_length + RTCM_MIN_FRAME_BYTES
            if len(buffer) < frame_length:
                return

            frame = bytes(buffer[:frame_length])
            del buffer[:frame_length]
            self._publish_rtcm_frame(frame)

    def _publish_rtcm_frame(self, frame: bytes) -> None:
        msg = UInt8MultiArray()
        msg.data = list(frame)
        self._rtcm_pub.publish(msg)

        with self._data_lock:
            self._last_rtcm_time_s = time.monotonic()
            self._received_count += 1
            self._last_message_size = len(frame)

    def _set_connected(self, connected: bool, last_error: str) -> None:
        with self._data_lock:
            self._connected = connected
            self._last_error = last_error

    def _close_socket(self) -> None:
        sock = self._socket
        self._socket = None
        if sock is None:
            return
        try:
            sock.close()
        except OSError:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RtkSourceManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
