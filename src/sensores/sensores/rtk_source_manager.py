#!/usr/bin/env python3

import base64
import json
import socket
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray
from sensores.ntrip_protocol import ChunkDecoder, RtcmDecoder, parse_response
from sensores.rtk_source_config import RtkSource, load_sources, save_sources, validate_source

RTCM_BUFFER_SIZE = 4096


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
        self.declare_parameter("max_reconnect_delay_s", 60.0)
        self.declare_parameter("rtcm_stale_timeout_s", 10.0)

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
        self._max_reconnect_delay_s = float(self.get_parameter("max_reconnect_delay_s").value)
        self._rtcm_stale_timeout_s = float(self.get_parameter("rtcm_stale_timeout_s").value)
        if min(self._connect_timeout_s, self._read_timeout_s, self._reconnect_delay_s,
               self._max_reconnect_delay_s, self._rtcm_stale_timeout_s) <= 0:
            raise ValueError("RTK timeouts must be positive")

        self._sources_config_path = Path(sources_config)
        self._sources, saved_source_id = load_sources(self._sources_config_path)
        self._sources_by_id = {source.id: source for source in self._sources}
        if not self._sources_by_id:
            raise RuntimeError(f"No RTK sources found in {sources_config}")

        initial_source_id = str(self.get_parameter("active_source_id").value).strip() or saved_source_id or self._sources[0].id
        if initial_source_id not in self._sources_by_id:
            raise ValueError("unknown_initial_rtk_source")

        self._data_lock = threading.Lock()
        self._active_source_id = initial_source_id
        self._source_generation = 0
        self._connected = False
        self._last_error = ""
        self._last_rtcm_time_s: Optional[float] = None
        self._received_count = 0
        self._last_message_size = 0
        self._crc_errors = 0
        self._status_sequence = 0
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
        password = str(password or "")

        source = RtkSource(
            id=source_id,
            label=label or source_id,
            host=host,
            port=port,
            mountpoint=mountpoint,
            username=username,
            password=password,
        )
        validate_source(source)
        return source

    def _publish_metadata(self) -> None:
        with self._data_lock:
            self._status_sequence += 1
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
                "status_sequence": self._status_sequence,
                "active_source_id": self._active_source_id,
                "active_source_label": source.label if source else None,
                "connected": self._connected,
                "receiving_rtcm": self._connected and rtcm_age_s is not None and rtcm_age_s <= self._rtcm_stale_timeout_s,
                "rtcm_stale_timeout_s": self._rtcm_stale_timeout_s,
                "crc_errors": self._crc_errors,
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
            # Persist before changing the live stream. A write error must not
            # silently switch the receiver to a different reference station.
            try:
                save_sources(self._sources_config_path, self._sources, requested_id)
            except OSError:
                self._last_error = "rtk_config_write_failed"
                return
            self._active_source_id = requested_id
            self._source_generation += 1
            self._connected = False
            self._last_error = ""
            self._last_rtcm_time_s = None
            self._received_count = 0
            self._last_message_size = 0
            self._crc_errors = 0

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
                    active = remaining[0].id if self._active_source_id == source_id else self._active_source_id
                    save_sources(self._sources_config_path, remaining, active)
                    reconnect = active != self._active_source_id
                    self._active_source_id = active
                    self._replace_sources_locked(remaining)
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
                    active = source.id if activate else self._active_source_id
                    save_sources(self._sources_config_path, new_sources, active)
                    reconnect = active != self._active_source_id or (active == source.id and existing != source)
                    self._active_source_id = active
                    self._replace_sources_locked(new_sources)
                    self._last_error = ""
                if reconnect:
                    self._source_generation += 1
                    self._connected = False
                    self._last_rtcm_time_s = None
                    self._received_count = 0
                    self._last_message_size = 0
                    self._crc_errors = 0
        except Exception:
            self.get_logger().warning("RTK source management failed")
            with self._data_lock:
                self._last_error = "rtk_source_management_failed"
            self._publish_metadata()
            return

        self._publish_metadata()
        if reconnect:
            self._close_socket()
            self._wake_connect.set()

    def _reader_loop(self) -> None:
        delay = self._reconnect_delay_s
        while not self._stop_event.is_set():
            with self._data_lock:
                source = self._sources_by_id[self._active_source_id]
                generation = self._source_generation
                self._last_rtcm_time_s = None
            try:
                sock, response = self._open_stream(source, generation)
                self._set_connected(True, "", generation)
                decoder = RtcmDecoder()
                chunks = ChunkDecoder() if response.chunked else None
                payload = response.payload
                last_valid = time.monotonic()
                while not self._stop_event.is_set():
                    with self._data_lock:
                        if generation != self._source_generation:
                            raise InterruptedError
                    frames = decoder.feed(chunks.feed(payload) if chunks else payload)
                    for frame in frames:
                        self._publish_rtcm_frame(frame, generation)
                        last_valid = time.monotonic()
                        delay = self._reconnect_delay_s
                    with self._data_lock:
                        if generation == self._source_generation:
                            self._crc_errors += decoder.crc_errors
                    decoder.crc_errors = 0
                    if chunks and chunks.finished:
                        raise ConnectionError("ntrip_stream_closed")
                    if time.monotonic() - last_valid > self._rtcm_stale_timeout_s:
                        raise ConnectionError("ntrip_no_valid_rtcm")
                    try:
                        payload = sock.recv(RTCM_BUFFER_SIZE)
                    except socket.timeout:
                        payload = b""
                        continue
                    if not payload:
                        raise ConnectionError("ntrip_stream_closed")
            except InterruptedError:
                pass
            except Exception as exc:
                # Only our controlled protocol errors are safe to publish.
                error = str(exc) if isinstance(exc, ConnectionError) and str(exc).startswith("ntrip_") else "ntrip_connection_failed"
                self._set_connected(False, error, generation)
            finally:
                self._close_socket()
            if self._wake_connect.wait(timeout=delay):
                self._wake_connect.clear()
                delay = self._reconnect_delay_s
            else:
                delay = min(delay * 2, self._max_reconnect_delay_s)

    def _open_stream(self, source: RtkSource, generation: int):
        sock = socket.create_connection(
            (source.host, source.port), timeout=self._connect_timeout_s
        )
        with self._data_lock:
            if generation != self._source_generation or self._stop_event.is_set():
                sock.close()
                raise InterruptedError
            self._socket = sock
        sock.settimeout(self._read_timeout_s)

        auth = base64.b64encode(
            f"{source.username}:{source.password}".encode("utf-8")
        ).decode("ascii")
        request = (
            f"GET /{source.mountpoint.lstrip('/')} HTTP/1.1\r\n"
            f"Host: {source.host}:{source.port}\r\n"
            "User-Agent: NTRIP RTKLIB/2.4.3\r\n"
            "Accept: */*\r\n"
            "Connection: close\r\n"
            "Ntrip-Version: Ntrip/2.0\r\n"
            f"Authorization: Basic {auth}\r\n"
            "\r\n"
        )
        sock.sendall(request.encode("ascii"))

        response = bytearray()
        deadline = time.monotonic() + self._connect_timeout_s
        while True:
            if time.monotonic() >= deadline:
                raise ConnectionError("ntrip_handshake_timeout")
            sock.settimeout(max(0.01, deadline - time.monotonic()))
            chunk = sock.recv(RTCM_BUFFER_SIZE)
            if not chunk:
                raise ConnectionError("ntrip_closed_during_handshake")
            response.extend(chunk)
            parsed = parse_response(bytes(response))
            if parsed is not None:
                sock.settimeout(self._read_timeout_s)
                self.get_logger().info(
                    f"NTRIP source connected: {source.label} "
                    f"({source.host}/{source.mountpoint})"
                )
                return sock, parsed

    def _publish_rtcm_frame(self, frame: bytes, generation: int) -> None:
        msg = UInt8MultiArray()
        msg.data = list(frame)
        with self._data_lock:
            if generation != self._source_generation or self._stop_event.is_set():
                return
            self._rtcm_pub.publish(msg)
            self._last_rtcm_time_s = time.monotonic()
            self._received_count += 1
            self._last_message_size = len(frame)

    def _set_connected(self, connected: bool, last_error: str, generation: int) -> None:
        with self._data_lock:
            if generation != self._source_generation:
                return
            self._connected = connected
            self._last_error = last_error

    def _close_socket(self) -> None:
        with self._data_lock:
            sock = self._socket
            self._socket = None
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
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
