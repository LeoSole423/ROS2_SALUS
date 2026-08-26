"""Bounded NTRIP/RTCM decoding, independent of ROS and receiver hardware."""
from __future__ import annotations

from dataclasses import dataclass

MAX_HEADER_BYTES = 16384


@dataclass(frozen=True)
class NtripResponse:
    payload: bytes
    chunked: bool = False


def parse_response(data: bytes) -> NtripResponse | None:
    """Wait for complete headers; never treat a source table as corrections."""
    if data.startswith(b"SOURCETABLE "):
        raise ConnectionError("ntrip_sourcetable_not_corrections")
    if data.startswith(b"ICY "):
        end = data.find(b"\r\n")
        if end < 0:
            if len(data) > MAX_HEADER_BYTES:
                raise ConnectionError("ntrip_headers_too_large")
            return None
        if data[:end] != b"ICY 200 OK":
            raise ConnectionError("ntrip_rejected")
        return NtripResponse(data[end + 2:])
    end = data.find(b"\r\n\r\n")
    if end < 0:
        if len(data) > MAX_HEADER_BYTES:
            raise ConnectionError("ntrip_headers_too_large")
        return None
    if end > MAX_HEADER_BYTES:
        raise ConnectionError("ntrip_headers_too_large")
    lines = data[:end].decode("latin1").split("\r\n")
    status = lines[0].split()
    if len(status) < 2 or status[0] not in ("HTTP/1.0", "HTTP/1.1"):
        raise ConnectionError("ntrip_invalid_response")
    if status[1] != "200":
        code = status[1] if status[1].isdigit() else "invalid"
        raise ConnectionError(f"ntrip_http_{code}")
    headers = {}
    for line in lines[1:]:
        if ":" in line:
            key, value = line.split(":", 1)
            headers[key.strip().lower()] = value.strip().lower()
    content_type = headers.get("content-type", "").split(";", 1)[0]
    if "sourcetable" in content_type or content_type.startswith("text/"):
        raise ConnectionError("ntrip_sourcetable_or_text_not_corrections")
    transfer = headers.get("transfer-encoding", "")
    if transfer not in ("", "identity", "chunked"):
        raise ConnectionError("ntrip_unsupported_transfer_encoding")
    if headers.get("content-encoding", "identity") not in ("", "identity"):
        raise ConnectionError("ntrip_unsupported_content_encoding")
    return NtripResponse(data[end + 4:], transfer == "chunked")


class ChunkDecoder:
    """Incremental HTTP chunk decoding, including split framing/RTCM packets."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.remaining: int | None = None
        self.finished = False

    def feed(self, data: bytes) -> bytes:
        if self.finished:
            return b""
        self.buffer.extend(data)
        result = bytearray()
        while True:
            if self.remaining is None:
                end = self.buffer.find(b"\r\n")
                if end < 0:
                    if len(self.buffer) > 1024:
                        raise ConnectionError("ntrip_invalid_chunk_header")
                    break
                try:
                    self.remaining = int(self.buffer[:end].split(b";", 1)[0], 16)
                except ValueError:
                    raise ConnectionError("ntrip_invalid_chunk_size") from None
                del self.buffer[:end + 2]
                if not 0 <= self.remaining <= 1048576:
                    raise ConnectionError("ntrip_chunk_too_large")
                if self.remaining == 0:
                    self.finished = True
                    self.buffer.clear()
                    break
            if len(self.buffer) < self.remaining + 2:
                break
            if self.buffer[self.remaining:self.remaining + 2] != b"\r\n":
                raise ConnectionError("ntrip_invalid_chunk_terminator")
            result.extend(self.buffer[:self.remaining])
            del self.buffer[:self.remaining + 2]
            self.remaining = None
        return bytes(result)


def crc24q(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


class RtcmDecoder:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes) -> list[bytes]:
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 6:
            if self.buffer[0] != 0xD3 or self.buffer[1] & 0xFC:
                del self.buffer[0]
                continue
            length = ((self.buffer[1] & 3) << 8) | self.buffer[2]
            size = length + 6
            if len(self.buffer) < size:
                break
            frame = bytes(self.buffer[:size])
            if crc24q(frame[:-3]) != int.from_bytes(frame[-3:], "big"):
                self.crc_errors += 1
                del self.buffer[0]
                continue
            del self.buffer[:size]
            frames.append(frame)
        return frames
