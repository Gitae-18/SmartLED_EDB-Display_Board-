#!/usr/bin/env python3
"""RS485 gateway for SMART_EDB_BOARD.

The wire protocol is kept in sync with Core/Src/rs485.c and
Core/Src/display_protocol.c. Output is newline-delimited JSON so another
process can consume measurements without parsing console text.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - depends on the target machine
    serial = None
    list_ports = None


STX = 0x02
ETX = 0x03
BROADCAST_ID = 0xFF
MAX_PAYLOAD_SIZE = 64

CMD_GET_MEASUREMENTS = 0x01
CMD_GET_STATUS = 0x02
CMD_SET_AC_OUTPUT = 0x10
CMD_PING = 0x7E
CMD_RESPONSE_FLAG = 0x80
CMD_ERROR = 0xFF

COMMAND_NAMES = {
    CMD_GET_MEASUREMENTS: "get_measurements",
    CMD_GET_STATUS: "get_status",
    CMD_SET_AC_OUTPUT: "set_ac_output",
    CMD_PING: "ping",
    CMD_GET_MEASUREMENTS | CMD_RESPONSE_FLAG: "measurements_response",
    CMD_GET_STATUS | CMD_RESPONSE_FLAG: "status_response",
    CMD_SET_AC_OUTPUT | CMD_RESPONSE_FLAG: "set_ac_output_response",
    CMD_PING | CMD_RESPONSE_FLAG: "ping_response",
    CMD_ERROR: "error",
}

ERROR_NAMES = {
    1: "bad_command",
    2: "bad_length",
    3: "send_failed",
}


class ProtocolError(ValueError):
    """Raised when an RS485 frame or application payload is invalid."""


@dataclass(frozen=True)
class Packet:
    destination_id: int
    source_id: int
    command: int
    payload: bytes = b""


def crc16(data: bytes) -> int:
    """BL0942 board CRC-16/Modbus: polynomial 0xA001, initial 0xFFFF."""
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def encode_frame(packet: Packet) -> bytes:
    if not 0 <= packet.destination_id <= 0xFF:
        raise ProtocolError("destination ID must be 0..255")
    if not 0 <= packet.source_id <= 0xFF:
        raise ProtocolError("source ID must be 0..255")
    if not 0 <= packet.command <= 0xFF:
        raise ProtocolError("command must be 0..255")
    if len(packet.payload) > MAX_PAYLOAD_SIZE:
        raise ProtocolError("payload exceeds 64 bytes")

    body = bytes(
        (3 + len(packet.payload), packet.destination_id,
         packet.source_id, packet.command)
    ) + packet.payload
    checksum = crc16(body)
    return bytes((STX,)) + body + struct.pack("<H", checksum) + bytes((ETX,))


def decode_frame(frame: bytes) -> Packet:
    if len(frame) < 8:
        raise ProtocolError("incomplete frame")
    if frame[0] != STX or frame[-1] != ETX:
        raise ProtocolError("invalid STX/ETX")

    body_length = frame[1]
    if not 3 <= body_length <= 3 + MAX_PAYLOAD_SIZE:
        raise ProtocolError(f"invalid body length {body_length}")
    if len(frame) != body_length + 5:
        raise ProtocolError(
            f"frame length mismatch: expected {body_length + 5}, got {len(frame)}"
        )

    received_crc = struct.unpack_from("<H", frame, 2 + body_length)[0]
    expected_crc = crc16(frame[1 : 2 + body_length])
    if received_crc != expected_crc:
        raise ProtocolError(
            f"CRC mismatch: expected 0x{expected_crc:04X}, "
            f"received 0x{received_crc:04X}"
        )
    return Packet(frame[2], frame[3], frame[4], frame[5 : 2 + body_length])


class FrameStreamParser:
    """Extract complete frames from arbitrary serial byte chunks."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.discarded_bytes = 0
        self.invalid_frames = 0

    def feed(self, data: bytes) -> list[tuple[Packet, bytes]]:
        self.buffer.extend(data)
        packets: list[tuple[Packet, bytes]] = []

        while self.buffer:
            try:
                stx_index = self.buffer.index(STX)
            except ValueError:
                self.discarded_bytes += len(self.buffer)
                self.buffer.clear()
                break

            if stx_index:
                self.discarded_bytes += stx_index
                del self.buffer[:stx_index]
            if len(self.buffer) < 2:
                break

            body_length = self.buffer[1]
            if not 3 <= body_length <= 3 + MAX_PAYLOAD_SIZE:
                self.invalid_frames += 1
                del self.buffer[0]
                continue

            frame_length = body_length + 5
            if len(self.buffer) < frame_length:
                break
            candidate = bytes(self.buffer[:frame_length])
            try:
                packet = decode_frame(candidate)
            except ProtocolError:
                self.invalid_frames += 1
                del self.buffer[0]
                continue

            packets.append((packet, candidate))
            del self.buffer[:frame_length]

        return packets


def parse_measurements(payload: bytes) -> dict[str, Any]:
    if len(payload) != 45:
        raise ProtocolError(f"measurement payload must be 45 bytes, got {len(payload)}")

    values = struct.unpack("<IiiiiiiBiBH iBi".replace(" ", ""), payload)
    return {
        "updated_at_ms": values[0],
        "ac_voltage_rms_v": values[1] / 1000.0,
        "ac_current_rms_a": values[2] / 1000.0,
        "dc_current_a": values[3] / 1000.0,
        "ac_active_power_w": values[4] / 1000.0,
        "ac_energy_wh": values[5] / 1000.0,
        "ac_frequency_hz": values[6] / 1000.0,
        "ac_meter_valid": bool(values[7]),
        "dc_voltage_v": values[8] / 1000.0,
        "dc_voltage_valid": bool(values[9]),
        "ntc_adc_raw": values[10],
        "ntc_adc_voltage_v": values[11] / 1000.0,
        "ntc_valid": bool(values[12]),
        "ntc_temperature_c": values[13] / 1000.0,
    }


def parse_status(payload: bytes) -> dict[str, Any]:
    if len(payload) != 10:
        raise ProtocolError(f"status payload must be 10 bytes, got {len(payload)}")
    meter_valid, ac_enabled, received, errors = struct.unpack("<BBII", payload)
    return {
        "ac_meter_valid": bool(meter_valid),
        "ac_output_enabled": bool(ac_enabled),
        "board_rx_packets": received,
        "board_protocol_errors": errors,
    }


def parse_payload(packet: Packet) -> dict[str, Any]:
    command = packet.command
    if command == (CMD_GET_MEASUREMENTS | CMD_RESPONSE_FLAG):
        return parse_measurements(packet.payload)
    if command == (CMD_GET_STATUS | CMD_RESPONSE_FLAG):
        return parse_status(packet.payload)
    if command == (CMD_SET_AC_OUTPUT | CMD_RESPONSE_FLAG):
        if len(packet.payload) != 1 or packet.payload[0] > 1:
            raise ProtocolError("AC output response must contain one boolean byte")
        return {"ac_output_enabled": bool(packet.payload[0])}
    if command in (CMD_PING, CMD_PING | CMD_RESPONSE_FLAG):
        result: dict[str, Any] = {"echo_hex": packet.payload.hex(" ").upper()}
        if len(packet.payload) == 4:
            result["board_tick_ms"] = struct.unpack("<I", packet.payload)[0]
        return result
    if command == CMD_ERROR:
        if len(packet.payload) != 2:
            raise ProtocolError("error response must contain two bytes")
        return {
            "rejected_command": packet.payload[0],
            "error_code": packet.payload[1],
            "error_name": ERROR_NAMES.get(packet.payload[1], "unknown"),
        }
    return {"payload_hex": packet.payload.hex(" ").upper()}


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def packet_record(packet: Packet, raw: bytes, parsed: dict[str, Any]) -> dict[str, Any]:
    return {
        "received_at": utc_timestamp(),
        "destination_id": packet.destination_id,
        "source_id": packet.source_id,
        "board_id": packet.source_id,
        "command": f"0x{packet.command:02X}",
        "command_name": COMMAND_NAMES.get(packet.command, "unknown"),
        "data": parsed,
        "raw_hex": raw.hex(" ").upper(),
    }


class EDBGateway:
    def __init__(
        self,
        port: str,
        baudrate: int,
        gateway_id: int,
        timeout: float,
        debug: bool,
    ) -> None:
        if serial is None:
            raise RuntimeError("pyserial is required: python -m pip install pyserial")
        self.gateway_id = gateway_id
        self.debug = debug
        self.stream = FrameStreamParser()
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=1.0,
        )

    def close(self) -> None:
        self.serial.close()

    def send(self, destination: int, command: int, payload: bytes = b"") -> None:
        packet = Packet(
            destination,
            self.gateway_id,
            command,
            payload,
        )
        frame = encode_frame(packet)
        written = self.serial.write(frame)
        self.serial.flush()
        if written != len(frame):
            raise IOError(f"short serial write: {written}/{len(frame)} bytes")
        if self.debug:
            print(f"TX {frame.hex(' ').upper()}", file=sys.stderr, flush=True)

    def receive(self) -> list[tuple[Packet, bytes]]:
        waiting = self.serial.in_waiting
        chunk = self.serial.read(waiting if waiting else 1)
        return self.stream.feed(chunk) if chunk else []


def select_port(requested: str | None) -> str:
    if requested:
        return requested
    if list_ports is None:
        raise RuntimeError("pyserial is required: python -m pip install pyserial")
    ports = list(list_ports.comports())
    if len(ports) == 1:
        return ports[0].device
    available = ", ".join(port.device for port in ports) or "none"
    raise RuntimeError(f"specify --port (available serial ports: {available})")


def parse_board_ids(specification: str) -> list[int]:
    """Parse comma-separated IDs and inclusive ranges such as 1,3,5-8."""
    board_ids: list[int] = []
    try:
        for item in specification.split(","):
            item = item.strip()
            if not item:
                raise ValueError
            if "-" in item:
                first_text, last_text = item.split("-", 1)
                first, last = int(first_text), int(last_text)
                if first > last:
                    raise ValueError
                values = range(first, last + 1)
            else:
                values = (int(item),)
            for value in values:
                if not 1 <= value <= 254:
                    raise ValueError
                if value not in board_ids:
                    board_ids.append(value)
    except ValueError as error:
        raise ProtocolError(
            "board IDs must be unique values/ranges from 1 to 254 "
            "(example: 1,3,5-8)"
        ) from error
    if not board_ids:
        raise ProtocolError("at least one board ID is required")
    return board_ids


def emit_packet(gateway: EDBGateway, packet: Packet, raw: bytes) -> None:
    # Some USB-RS485 adapters echo transmitted requests; do not report them as data.
    if packet.source_id == gateway.gateway_id:
        if gateway.debug:
            print(f"RX echo {raw.hex(' ').upper()}", file=sys.stderr, flush=True)
        return
    if packet.destination_id not in (gateway.gateway_id, BROADCAST_ID):
        if gateway.debug:
            print(
                f"RX ignored destination={packet.destination_id} "
                f"{raw.hex(' ').upper()}",
                file=sys.stderr,
                flush=True,
            )
        return

    try:
        parsed = parse_payload(packet)
        record = packet_record(packet, raw, parsed)
    except ProtocolError as error:
        record = packet_record(packet, raw, {"parse_error": str(error)})
    print(json.dumps(record, ensure_ascii=False, separators=(",", ":")), flush=True)

    # The board's diagnostic 'p' command sends a ping request to gateway node 2.
    if packet.command == CMD_PING and packet.destination_id != BROADCAST_ID:
        gateway.send(
            packet.source_id,
            CMD_PING | CMD_RESPONSE_FLAG,
            packet.payload,
        )


def emit_timeout(board_id: int, command: int) -> None:
    record = {
        "received_at": utc_timestamp(),
        "event": "board_timeout",
        "board_id": board_id,
        "command": f"0x{command:02X}",
        "command_name": COMMAND_NAMES.get(command, "unknown"),
    }
    print(json.dumps(record, separators=(",", ":")), flush=True)


def run_gateway(args: argparse.Namespace) -> int:
    port = select_port(args.port)
    gateway = EDBGateway(
        port, args.baudrate, args.gateway_id, args.timeout, args.debug
    )
    print(
        f"EDB gateway opened {port} at {args.baudrate} 8-N-1 "
        f"(gateway={args.gateway_id}, boards={','.join(map(str, args.board_ids))})",
        file=sys.stderr,
        flush=True,
    )

    command_payload = b""
    if args.command == "measurements":
        command = CMD_GET_MEASUREMENTS
    elif args.command == "status":
        command = CMD_GET_STATUS
    elif args.command == "ping":
        command = CMD_PING
        command_payload = struct.pack("<I", int(time.monotonic() * 1000) & 0xFFFFFFFF)
    elif args.command == "ac-on":
        command = CMD_SET_AC_OUTPUT
        command_payload = b"\x01"
    elif args.command == "ac-off":
        command = CMD_SET_AC_OUTPUT
        command_payload = b"\x00"
    else:
        command = None

    board_index = 0
    pending_board: int | None = None
    response_deadline = 0.0
    next_request = time.monotonic()
    responses = 0
    try:
        while True:
            now = time.monotonic()
            if pending_board is not None and now >= response_deadline:
                emit_timeout(pending_board, command)
                pending_board = None
                board_index += 1
                next_request = now + args.request_gap

            if command is not None and pending_board is None and now >= next_request:
                if board_index >= len(args.board_ids):
                    if args.once:
                        return 0 if responses > 0 else 2
                    board_index = 0
                    next_request = now + args.interval
                else:
                    pending_board = args.board_ids[board_index]
                    gateway.send(pending_board, command, command_payload)
                    response_deadline = now + args.response_timeout

            received = gateway.receive()
            for packet, raw in received:
                emit_packet(gateway, packet, raw)
                if (
                    pending_board is not None
                    and packet.source_id == pending_board
                    and packet.destination_id == args.gateway_id
                    and packet.command in (command | CMD_RESPONSE_FLAG, CMD_ERROR)
                ):
                    responses += 1
                    pending_board = None
                    board_index += 1
                    next_request = time.monotonic() + args.request_gap
    except KeyboardInterrupt:
        return 0
    finally:
        gateway.close()


def self_test() -> int:
    test_packet = Packet(1, 2, CMD_PING, bytes.fromhex("12 34 56 78"))
    encoded = encode_frame(test_packet)
    assert decode_frame(encoded) == test_packet

    parser = FrameStreamParser()
    assert parser.feed(b"noise" + encoded[:3]) == []
    decoded = parser.feed(encoded[3:])
    assert decoded == [(test_packet, encoded)]

    measurement_payload = struct.pack(
        "<IiiiiiiBiBHiBi",
        1234, 48657, 1737, 4034, 80272, 1500, 60103, 1,
        31271, 1, 2071, 1668, 1, 24400,
    )
    measurement = parse_measurements(measurement_payload)
    assert measurement["ac_voltage_rms_v"] == 48.657
    assert measurement["ntc_temperature_c"] == 24.4
    assert parse_board_ids("1,3,5-7") == [1, 3, 5, 6, 7]
    print("protocol self-test: PASS")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="SMART EDB board RS485 receiver, parser, and poller"
    )
    parser.add_argument("--port", help="serial port, for example COM5 or /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200, help="default: 115200")
    parser.add_argument("--gateway-id", type=int, default=0, help="gateway node ID (default: 0)")
    parser.add_argument(
        "--board-ids", default="1",
        help="board IDs/ranges to poll, for example 1,3,5-8 (default: 1)",
    )
    parser.add_argument("--board-id", type=int, help=argparse.SUPPRESS)
    parser.add_argument(
        "--command",
        choices=("listen", "measurements", "status", "ping", "ac-on", "ac-off"),
        default="measurements",
        help="request to send; listen only receives (default: measurements)",
    )
    parser.add_argument(
        "--interval", type=float, default=1.0,
        help="delay between complete board scans in seconds",
    )
    parser.add_argument(
        "--request-gap", type=float, default=0.02,
        help="delay between board requests in seconds",
    )
    parser.add_argument("--once", action="store_true", help="scan every configured board once")
    parser.add_argument(
        "--response-timeout", type=float, default=0.5,
        help="per-board response timeout in seconds (default: 0.5)",
    )
    parser.add_argument("--timeout", type=float, default=0.05, help="serial read timeout")
    parser.add_argument("--debug", action="store_true", help="print TX/RX details to stderr")
    parser.add_argument("--self-test", action="store_true", help="test codec without serial hardware")
    args = parser.parse_args()

    if not 0 <= args.gateway_id <= 254:
        parser.error("--gateway-id must be 0..254")
    try:
        board_specification = str(args.board_id) if args.board_id is not None else args.board_ids
        args.board_ids = parse_board_ids(board_specification)
    except ProtocolError as error:
        parser.error(str(error))
    if args.gateway_id in args.board_ids:
        parser.error("--gateway-id must not match a board ID")
    if (
        args.interval <= 0 or args.request_gap < 0
        or args.response_timeout <= 0 or args.timeout < 0
    ):
        parser.error("interval and timeout values must be positive")
    return args


def main() -> int:
    args = parse_args()
    if args.self_test:
        return self_test()
    try:
        return run_gateway(args)
    except (OSError, RuntimeError, ProtocolError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
