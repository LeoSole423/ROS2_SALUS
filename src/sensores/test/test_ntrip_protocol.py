import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sensores.ntrip_protocol import ChunkDecoder, RtcmDecoder, crc24q, parse_response
from sensores.rtk_source_config import RtkSource, load_sources, save_sources, validate_source


def frame(payload=b"\x43\x50test"):
    data = b"\xd3" + len(payload).to_bytes(2, "big") + payload
    return data + crc24q(data).to_bytes(3, "big")


def test_crc_known_vector():
    assert crc24q(b"123456789") == 0xCDE703


def test_http_waits_for_all_headers():
    header = b"HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\n\r\n"
    for size in range(len(header)):
        assert parse_response(header[:size]) is None
    result = parse_response(header + frame())
    assert result.payload == frame()
    assert not result.chunked


@pytest.mark.parametrize("reply", [
    b"HTTP/1.1 401 Unauthorized\r\n\r\n",
    b"HTTP/1.1 2000 Bad\r\n\r\n",
    b"SOURCETABLE 200 OK\r\n",
    b"HTTP/1.1 200 OK\r\nContent-Type: gnss/sourcetable\r\n\r\n",
    b"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n",
    b"HTTP/1.1 200 OK\r\nContent-Encoding: gzip\r\n\r\n",
    b"HTTP/1.1 200 OK\r\nTransfer-Encoding: gzip\r\n\r\n",
    b"x" * 16385,
])
def test_rejects_non_corrections(reply):
    with pytest.raises(ConnectionError):
        parse_response(reply)


def test_icy_and_chunked():
    assert parse_response(b"ICY 200 OK\r\n" + frame()).payload == frame()
    response = parse_response(b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n")
    assert response.chunked
    decoder = ChunkDecoder()
    wire = b"3;ext=yes\r\nabc\r\n2\r\nde\r\n0\r\n\r\n"
    assert b"".join(decoder.feed(bytes([byte])) for byte in wire) == b"abcde"
    assert decoder.finished


@pytest.mark.parametrize("wire", [b"xyz\r\n", b"-1\r\n", b"100001\r\n", b"2\r\nabXX"])
def test_invalid_chunks(wire):
    with pytest.raises(ConnectionError):
        ChunkDecoder().feed(wire)


def test_split_frames_crc_and_resync():
    decoder = RtcmDecoder()
    valid = frame()
    corrupted = valid[:-1] + bytes([valid[-1] ^ 1])
    assert decoder.feed(b"noise" + corrupted + valid[:4]) == []
    assert decoder.feed(valid[4:] + valid) == [valid, valid]
    assert decoder.crc_errors == 1
    assert not decoder.buffer


def test_config_owner_only_roundtrip(tmp_path):
    source = RtkSource("ign", "IGN", "example.test", 2101, "BASE", "user-secret", " password-secret ")
    path = tmp_path / "sources.yaml"
    save_sources(path, [source], source.id)
    assert load_sources(path) == ([source], "ign")
    assert path.stat().st_mode & 0o777 == 0o600
    assert "user-secret" not in repr(source)
    assert "password-secret" not in repr(source)


def test_config_atomic_failure_preserves_previous(tmp_path, monkeypatch):
    source = RtkSource("ign", "IGN", "example.test", 2101, "BASE", "u", "p")
    path = tmp_path / "sources.yaml"
    save_sources(path, [source], "ign")
    previous = path.read_bytes()
    def fail(*args):
        raise OSError("disk failure")
    monkeypatch.setattr("sensores.rtk_source_config.os.replace", fail)
    with pytest.raises(OSError):
        save_sources(path, [source], "other")
    assert path.read_bytes() == previous
    assert list(tmp_path.iterdir()) == [path]


@pytest.mark.parametrize("text", ["password: [SECRET", "sources: []", "sources: SECRET"])
def test_config_errors_do_not_leak_content(tmp_path, text):
    path = tmp_path / "bad.yaml"
    path.write_text(text)
    with pytest.raises(ValueError, match="^invalid_or_unreadable_rtk_sources_config$"):
        load_sources(path)


@pytest.mark.parametrize("host,port,mount", [("host\r\nInjected: x", 2101, "B"), ("host", 65536, "B"), ("host", 2101, "B?secret")])
def test_endpoint_validation(host, port, mount):
    with pytest.raises(ValueError):
        validate_source(RtkSource("id", "label", host, port, mount, "u", "p"))
