"""Socket integration without ROS discovery, credentials or robot hardware."""
import importlib.util
import json
import socket
import sys
import threading
import time
from pathlib import Path
from types import ModuleType
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sensores.ntrip_protocol import crc24q
from sensores.rtk_source_config import RtkSource, load_sources, save_sources


class Message:
    def __init__(self, data=None):
        self.data = data


@pytest.fixture
def manager(tmp_path, monkeypatch):
    # Import this node with only its ROS boundary stubbed. Socket/parser/config
    # and threading code are the production implementation.
    for name, attrs in {
        "rclpy": {}, "rclpy.node": {"Node": object},
        "ament_index_python": {},
        "ament_index_python.packages": {"get_package_share_directory": lambda _: "unused"},
        "std_msgs": {}, "std_msgs.msg": {"String": Message, "UInt8MultiArray": Message},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location("tested_rtk_manager", Path(__file__).resolve().parents[1] / "sensores/rtk_source_manager.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = module.RtkSourceManager.__new__(module.RtkSourceManager)
    source = RtkSource("a", "A", "127.0.0.1", 2101, "BASE", "test-user", "test-secret")
    node._sources = [source]
    node._sources_by_id = {"a": source}
    node._active_source_id = "a"
    node._source_generation = 0
    node._status_sequence = 0
    node._sources_config_path = tmp_path / "sources.yaml"
    save_sources(node._sources_config_path, node._sources, "a")
    node._data_lock = threading.Lock()
    node._wake_connect = threading.Event()
    node._stop_event = threading.Event()
    node._socket = None
    node._connected = False
    node._last_error = ""
    node._last_rtcm_time_s = None
    node._received_count = node._last_message_size = node._crc_errors = 0
    node._connect_timeout_s = 0.5
    node._read_timeout_s = 0.02
    node._reconnect_delay_s = 0.05
    node._max_reconnect_delay_s = 0.2
    node._rtcm_stale_timeout_s = 0.12
    node._rtcm_pub = Mock()
    node._sources_pub = Mock()
    node._status_pub = Mock()
    node.get_logger = lambda: Mock()
    yield node
    node._stop_event.set()
    node._wake_connect.set()
    node._close_socket()
    if hasattr(node, "_worker"):
        node._worker.join(1)
        assert not node._worker.is_alive()


def valid_frame():
    data = b"\xd3\x00\x04\x43\x50\x00\x00"
    return data + crc24q(data).to_bytes(3, "big")


def wait_for(predicate):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        if predicate():
            return
        threading.Event().wait(0.005)
    assert predicate()


@pytest.mark.parametrize("reply,expected", [
    (b"HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\n\r\n", "ntrip_no_valid_rtcm"),
    (b"HTTP/1.1 401 Unauthorized\r\n\r\n", "ntrip_http_401"),
    (b"SOURCETABLE 200 OK\r\n", "ntrip_sourcetable_not_corrections"),
])
def test_socket_rejection_or_silence_does_not_publish(manager, reply, expected):
    run_server(manager, reply)
    wait_for(lambda: manager._last_error == expected)
    assert manager._received_count == 0
    assert not manager._connected
    assert manager._socket is None


def run_server(manager, reply):
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    listener.settimeout(1)
    source = RtkSource("a", "A", "127.0.0.1", listener.getsockname()[1], "BASE", "test-user", "test-secret")
    manager._sources_by_id["a"] = source
    def serve():
        try:
            connection, _ = listener.accept()
            with connection:
                connection.settimeout(1)
                request = b""
                while b"\r\n\r\n" not in request:
                    request += connection.recv(4096)
                assert b"Host: 127.0.0.1:" in request
                assert b"Accept: */*" in request
                # Fragment both HTTP and RTCM boundaries.
                for offset in range(0, len(reply), 3):
                    connection.sendall(reply[offset:offset + 3])
                manager._stop_event.wait(0.5)
        except (OSError, TimeoutError):
            pass
        finally:
            listener.close()
    threading.Thread(target=serve, daemon=True).start()
    manager._worker = threading.Thread(target=manager._reader_loop, daemon=True)
    manager._worker.start()


def test_socket_chunked_crc_and_status(manager):
    frame = valid_frame()
    bad = frame[:-1] + bytes([frame[-1] ^ 1])
    payload = bad + frame
    chunk = f"{len(payload):x}\r\n".encode() + payload + b"\r\n"
    run_server(manager, b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n" + chunk)
    wait_for(lambda: manager._received_count == 1)
    manager._publish_metadata()
    status = json.loads(manager._status_pub.publish.call_args.args[0].data)
    assert status["receiving_rtcm"] is True
    assert status["crc_errors"] == 1
    assert bytes(manager._rtcm_pub.publish.call_args.args[0].data) == frame
    assert "test-secret" not in json.dumps(status)


def test_source_edit_persists_active_and_rejects_old_generation(manager):
    manager._manage_source_cb(Message(json.dumps({
        "id": "b", "host": "localhost", "mountpoint": "BASE2", "activate": True
    })))
    assert manager._active_source_id == "b"
    assert load_sources(manager._sources_config_path)[1] == "b"
    manager._publish_rtcm_frame(valid_frame(), 0)
    manager._set_connected(True, "old", 0)
    assert not manager._connected
    manager._rtcm_pub.publish.assert_not_called()
    manager._select_source_cb(Message("a"))
    assert load_sources(manager._sources_config_path)[1] == "a"
    assert manager._source_generation == 2


def test_failed_save_leaves_live_source_unchanged(manager, monkeypatch):
    monkeypatch.setattr("sensores.rtk_source_config.os.replace", Mock(side_effect=OSError("secret")))
    manager._manage_source_cb(Message(json.dumps({
        "id": "b", "host": "localhost", "mountpoint": "BASE2", "activate": True
    })))
    assert manager._active_source_id == "a"
    assert list(manager._sources_by_id) == ["a"]
    assert manager._last_error == "rtk_source_management_failed"
