#!/usr/bin/env python3
"""RS485 gateway for SMART_EDB_BOARD.

The wire protocol is kept in sync with Core/Src/rs485.c and
Core/Src/display_protocol.c. Output is newline-delimited JSON so another
process can consume measurements without parsing console text.
"""

from __future__ import annotations

import argparse
import json
import os
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

try:
    import RPi.GPIO as GPIO
except ImportError:  # pragma: no cover - only available on Raspberry Pi
    GPIO = None


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
        self.length_errors = 0
        self.crc_errors = 0
        self.format_errors = 0
        self.diagnostics: list[str] = []

    def _record_error(self, category: str, message: str, raw: bytes) -> None:
        self.invalid_frames += 1
        if category == "length":
            self.length_errors += 1
        elif category == "crc":
            self.crc_errors += 1
        else:
            self.format_errors += 1
        self.diagnostics.append(
            f"{category}: {message}; raw={raw.hex(' ').upper()}"
        )

    def pop_diagnostics(self) -> list[str]:
        diagnostics, self.diagnostics = self.diagnostics, []
        return diagnostics

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
                self._record_error(
                    "length",
                    f"invalid body length {body_length}",
                    bytes(self.buffer[: min(len(self.buffer), 8)]),
                )
                del self.buffer[0]
                continue

            frame_length = body_length + 5
            if len(self.buffer) < frame_length:
                break
            candidate = bytes(self.buffer[:frame_length])
            try:
                packet = decode_frame(candidate)
            except ProtocolError as error:
                category = "crc" if str(error).startswith("CRC mismatch") else "format"
                self._record_error(category, str(error), candidate)
                if category == "crc":
                    # STX, length, and ETX were valid, so this is a complete
                    # damaged frame. Skip its binary payload as one unit.
                    del self.buffer[:frame_length]
                else:
                    # The length byte itself may be damaged. Advance only past
                    # STX and scan again so a following valid frame is retained.
                    del self.buffer[0]
                continue

            packets.append((packet, candidate))
            del self.buffer[:frame_length]

        return packets


def parse_measurements(payload: bytes) -> dict[str, Any]:
    supported_lengths = (29, 34, 41, 45)
    if len(payload) not in supported_lengths:
        raise ProtocolError(
            "measurement payload must be 29, 34, 41, or 45 bytes, "
            f"got {len(payload)}"
        )

    values = struct.unpack("<IiiiiiiB", payload[:29])
    result: dict[str, Any] = {
        "payload_length": len(payload),
        "updated_at_ms": values[0],
        "ac_voltage_rms_v": values[1] / 1000.0,
        "ac_current_rms_a": values[2] / 1000.0,
        "dc_current_a": values[3] / 1000.0,
        "ac_active_power_w": values[4] / 1000.0,
        "ac_energy_wh": values[5] / 1000.0,
        "ac_frequency_hz": values[6] / 1000.0,
        "ac_meter_valid": bool(values[7]),
    }
    if len(payload) >= 34:
        dc_voltage_mv, dc_voltage_valid = struct.unpack_from("<iB", payload, 29)
        result.update({
            "dc_voltage_v": dc_voltage_mv / 1000.0,
            "dc_voltage_valid": bool(dc_voltage_valid),
        })
    if len(payload) >= 41:
        ntc_raw, ntc_voltage_mv, ntc_valid = struct.unpack_from(
            "<HiB", payload, 34
        )
        result.update({
            "ntc_adc_raw": ntc_raw,
            "ntc_adc_voltage_v": ntc_voltage_mv / 1000.0,
            "ntc_valid": bool(ntc_valid),
        })
    if len(payload) >= 45:
        temperature_mc = struct.unpack_from("<i", payload, 41)[0]
        result["ntc_temperature_c"] = temperature_mc / 1000.0
    return result


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
        de_gpio: int | None,
        de_before_tx: float,
        de_after_tx: float,
    ) -> None:
        if serial is None:
            raise RuntimeError("pyserial is required: python -m pip install pyserial")
        self.gateway_id = gateway_id
        self.debug = debug
        self.de_gpio = de_gpio
        self.de_before_tx = de_before_tx
        self.de_after_tx = de_after_tx
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
        if self.de_gpio is not None:
            if GPIO is None:
                self.serial.close()
                raise RuntimeError(
                    "--de-gpio requires RPi.GPIO or rpi-lgpio; "
                    "add --with rpi-lgpio to the uv command"
                )
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.de_gpio, GPIO.OUT, initial=GPIO.LOW)

    def close(self) -> None:
        if self.de_gpio is not None and GPIO is not None:
            GPIO.output(self.de_gpio, GPIO.LOW)
            GPIO.cleanup(self.de_gpio)
        self.serial.close()

    def send(self, destination: int, command: int, payload: bytes = b"") -> None:
        packet = Packet(
            destination,
            self.gateway_id,
            command,
            payload,
        )
        frame = encode_frame(packet)
        if self.de_gpio is not None:
            GPIO.output(self.de_gpio, GPIO.HIGH)
            if self.de_before_tx > 0:
                time.sleep(self.de_before_tx)
        try:
            written = self.serial.write(frame)
            self.serial.flush()
            if self.de_gpio is not None and self.de_after_tx > 0:
                time.sleep(self.de_after_tx)
        finally:
            if self.de_gpio is not None:
                GPIO.output(self.de_gpio, GPIO.LOW)
        if written != len(frame):
            raise IOError(f"short serial write: {written}/{len(frame)} bytes")
        if self.debug:
            print(f"TX {frame.hex(' ').upper()}", file=sys.stderr, flush=True)

    def receive(self) -> list[tuple[Packet, bytes]]:
        waiting = self.serial.in_waiting
        chunk = self.serial.read(waiting if waiting else 1)
        if chunk and self.debug:
            print(f"RX-BYTES {chunk.hex(' ').upper()}", file=sys.stderr, flush=True)
        packets = self.stream.feed(chunk) if chunk else []
        if self.debug:
            for diagnostic in self.stream.pop_diagnostics():
                print(f"RX-INVALID {diagnostic}", file=sys.stderr, flush=True)
        else:
            self.stream.pop_diagnostics()
        return packets


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


def emit_packet(
    gateway: EDBGateway,
    packet: Packet,
    raw: bytes,
    mqtt_bridge: Any | None = None,
) -> None:
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

    if (
        mqtt_bridge is not None
        and packet.command == (CMD_GET_MEASUREMENTS | CMD_RESPONSE_FLAG)
    ):
        mqtt_bridge.publish_telemetry("measurements", record, qos=1)

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


def connect_mqtt(args: argparse.Namespace) -> Any | None:
    """Connect MQTT when configured; otherwise keep stdout-only operation."""
    if not args.mqtt_host:
        return None

    try:
        from mqtt_client import MqttBridge
    except ImportError as error:
        raise RuntimeError(
            "MQTT requires: python -m pip install paho-mqtt"
        ) from error

    client_id = args.mqtt_client_id or f"edb-gateway-{args.gateway_id}"
    bridge = MqttBridge(
        host=args.mqtt_host,
        port=args.mqtt_port,
        base=args.mqtt_base_topic,
        client_id=client_id,
        username=args.mqtt_username,
        password=args.mqtt_password,
        tls_ca=args.mqtt_tls_ca,
        tls_cert=args.mqtt_tls_cert,
        tls_key=args.mqtt_tls_key,
    )
    result = bridge.connect()
    if result != 0:
        raise RuntimeError(f"MQTT connect failed with result code {result}")
    bridge.loop_start()
    if not bridge.wait_connected(args.mqtt_connect_timeout):
        bridge.disconnect()
        raise RuntimeError(
            f"MQTT connection was not established within "
            f"{args.mqtt_connect_timeout:g} seconds"
        )

    print(
        f"MQTT connected to {args.mqtt_host}:{args.mqtt_port} "
        f"as {client_id}; telemetry topic="
        f"{args.mqtt_base_topic.strip('/')}/{client_id}/telemetry/measurements",
        file=sys.stderr,
        flush=True,
    )
    return bridge


def run_gateway(args: argparse.Namespace) -> int:
    port = select_port(args.port)
    gateway = EDBGateway(
        port, args.baudrate, args.gateway_id, args.timeout, args.debug,
        args.de_gpio, args.de_before_tx, args.de_after_tx,
    )
    mqtt_bridge = None
    try:
        mqtt_bridge = connect_mqtt(args)
        print(
            f"EDB gateway opened {port} at {args.baudrate} 8-N-1 "
            f"(gateway={args.gateway_id}, boards={','.join(map(str, args.board_ids))}, "
            f"DE={'auto' if args.de_gpio is None else f'BCM{args.de_gpio}'})",
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
            command_payload = struct.pack(
                "<I", int(time.monotonic() * 1000) & 0xFFFFFFFF
            )
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
                emit_packet(gateway, packet, raw, mqtt_bridge)
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
        if mqtt_bridge is not None:
            mqtt_bridge.disconnect()
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
    assert measurement["payload_length"] == 45

    legacy_29 = parse_measurements(measurement_payload[:29])
    legacy_34 = parse_measurements(measurement_payload[:34])
    legacy_41 = parse_measurements(measurement_payload[:41])
    assert "dc_voltage_v" not in legacy_29
    assert legacy_34["dc_voltage_v"] == 31.271
    assert "ntc_adc_raw" not in legacy_34
    assert legacy_41["ntc_adc_raw"] == 2071
    assert "ntc_temperature_c" not in legacy_41

    damaged = bytearray(encoded)
    damaged[-3] ^= 0x01
    following = encode_frame(Packet(1, 2, CMD_PING, b"next"))
    recovery_parser = FrameStreamParser()
    recovered = recovery_parser.feed(bytes(damaged) + following)
    assert recovered == [(Packet(1, 2, CMD_PING, b"next"), following)]
    assert recovery_parser.crc_errors == 1
    assert recovery_parser.invalid_frames == 1
    assert recovery_parser.pop_diagnostics()

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
    parser.add_argument(
        "--de-gpio", type=int,
        help="BCM GPIO connected to active-high RS485 DE (for this GW: 11)",
    )
    parser.add_argument(
        "--de-before-tx", type=float, default=0.001,
        help="DE settling delay before TX in seconds (default: 0.001)",
    )
    parser.add_argument(
        "--de-after-tx", type=float, default=0.0,
        help="hold DE after UART drain in seconds (default: 0; switch to RX immediately)",
    )
    parser.add_argument("--debug", action="store_true", help="print TX/RX details to stderr")
    parser.add_argument(
        "--mqtt-host", default=os.getenv("MQTT_HOST"),
        help="MQTT broker hostname; enables MQTT publishing (env: MQTT_HOST)",
    )
    parser.add_argument(
        "--mqtt-port", type=int, default=int(os.getenv("MQTT_PORT", "1883")),
        help="MQTT broker port (env: MQTT_PORT, default: 1883)",
    )
    parser.add_argument(
        "--mqtt-client-id", default=os.getenv("MQTT_CLIENT_ID"),
        help="MQTT client/gateway ID (env: MQTT_CLIENT_ID)",
    )
    parser.add_argument(
        "--mqtt-username", default=os.getenv("MQTT_USERNAME"),
        help="MQTT username (env: MQTT_USERNAME)",
    )
    parser.add_argument(
        "--mqtt-password", default=os.getenv("MQTT_PASSWORD"),
        help="MQTT password (env: MQTT_PASSWORD)",
    )
    parser.add_argument(
        "--mqtt-base-topic", default=os.getenv("MQTT_BASE_TOPIC", "display"),
        help="MQTT base topic (env: MQTT_BASE_TOPIC, default: display)",
    )
    parser.add_argument(
        "--mqtt-tls-ca", default=os.getenv("MQTT_TLS_CA"),
        help="trusted CA certificate path (env: MQTT_TLS_CA)",
    )
    parser.add_argument(
        "--mqtt-tls-cert", default=os.getenv("MQTT_TLS_CERT"),
        help="client certificate path (env: MQTT_TLS_CERT)",
    )
    parser.add_argument(
        "--mqtt-tls-key", default=os.getenv("MQTT_TLS_KEY"),
        help="client private-key path (env: MQTT_TLS_KEY)",
    )
    parser.add_argument(
        "--mqtt-connect-timeout", type=float,
        default=float(os.getenv("MQTT_CONNECT_TIMEOUT", "10")),
        help="MQTT connection timeout in seconds (default: 10)",
    )
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
    if args.de_gpio is not None and not 0 <= args.de_gpio <= 27:
        parser.error("--de-gpio must be a BCM GPIO number from 0 to 27")
    if (
        args.interval <= 0 or args.request_gap < 0
        or args.response_timeout <= 0 or args.timeout < 0
        or args.de_before_tx < 0 or args.de_after_tx < 0
    ):
        parser.error("interval and timeout values must be positive")
    if not 1 <= args.mqtt_port <= 65535:
        parser.error("--mqtt-port must be 1..65535")
    if args.mqtt_connect_timeout <= 0:
        parser.error("--mqtt-connect-timeout must be positive")
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
