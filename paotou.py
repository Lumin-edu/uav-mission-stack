#!/usr/bin/env python3
import argparse
import os
import sys
import termios
from typing import Optional


BAUD_MAP = {
    1200: termios.B1200,
    2400: termios.B2400,
    4800: termios.B4800,
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
}


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(frame: bytes) -> bytes:
    crc = crc16_modbus(frame)
    return frame + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


class SerialPort:
    def __init__(
        self,
        device: str,
        baudrate: int,
        timeout: float,
        parity: str = "N",
        stopbits: int = 1,
    ) -> None:
        if baudrate not in BAUD_MAP:
            raise ValueError(
                f"Unsupported baudrate {baudrate}. Supported: {sorted(BAUD_MAP)}"
            )
        if parity not in ("N", "E", "O"):
            raise ValueError("Parity must be one of: N, E, O")
        if stopbits not in (1, 2):
            raise ValueError("Stopbits must be 1 or 2")
        self.device = device
        self.baudrate = baudrate
        self.timeout = timeout
        self.parity = parity
        self.stopbits = stopbits
        self.fd: Optional[int] = None
        self._old_attrs = None

    def open(self) -> None:
        self.fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
        attrs = termios.tcgetattr(self.fd)
        self._old_attrs = attrs[:]

        attrs[0] = 0
        attrs[1] = 0
        cflag = termios.CLOCAL | termios.CREAD | termios.CS8
        if self.parity == "E":
            cflag |= termios.PARENB
        elif self.parity == "O":
            cflag |= termios.PARENB | termios.PARODD
        if self.stopbits == 2:
            cflag |= termios.CSTOPB
        attrs[2] = cflag
        attrs[3] = 0
        attrs[4] = BAUD_MAP[self.baudrate]
        attrs[5] = BAUD_MAP[self.baudrate]
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = max(1, int(self.timeout * 10))

        termios.tcflush(self.fd, termios.TCIOFLUSH)
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def close(self) -> None:
        if self.fd is None:
            return
        if self._old_attrs is not None:
            termios.tcsetattr(self.fd, termios.TCSANOW, self._old_attrs)
        os.close(self.fd)
        self.fd = None

    def write(self, data: bytes) -> None:
        if self.fd is None:
            raise RuntimeError("Serial port is not open")
        os.write(self.fd, data)
        termios.tcdrain(self.fd)

    def read(self, size: int = 256) -> bytes:
        if self.fd is None:
            raise RuntimeError("Serial port is not open")
        return os.read(self.fd, size)

    def __enter__(self) -> "SerialPort":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def build_yf_frame(slave_id: int, register: int, angle: int) -> bytes:
    if not 0 <= slave_id <= 247:
        raise ValueError("slave_id must be in 0..247")
    if not 0 <= register <= 0xFFFF:
        raise ValueError("register must be in 0..65535")
    if not 0 <= angle <= 180:
        raise ValueError("angle must be in 0..180")

    # YF-1CHH-DJ-USB-KZ-V1 uses function 0x05, but the value field is not
    # standard coil ON/OFF. The captured Windows frame for 105 deg is:
    # 01 05 00 00 69 00 E3 9A, so angle is stored in the high byte.
    frame = bytes(
        (
            slave_id,
            0x05,
            (register >> 8) & 0xFF,
            register & 0xFF,
            angle & 0xFF,
            0x00,
        )
    )
    return append_crc(frame)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Control YF-1CHH-DJ-USB-KZ-V1 one-channel USB servo board."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--parity", choices=("N", "E", "O"), default="N")
    parser.add_argument("--stopbits", type=int, choices=(1, 2), default=1)
    parser.add_argument("--slave-id", type=int, default=1)
    parser.add_argument("--register", type=lambda x: int(x, 0), default=0)
    parser.add_argument("--angle", type=int, required=True)
    parser.add_argument("--timeout", type=float, default=0.3)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    try:
        request = build_yf_frame(args.slave_id, args.register, args.angle)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    print(f"Port     : {args.port}")
    print(f"Baudrate : {args.baudrate}")
    print(f"Slave ID : {args.slave_id}")
    print(f"Register : 0x{args.register:04X}")
    print(f"Angle    : {args.angle}")
    print(f"Request  : {request.hex(' ')}")

    if args.dry_run:
        return 0

    try:
        with SerialPort(
            args.port,
            args.baudrate,
            args.timeout,
            parity=args.parity,
            stopbits=args.stopbits,
        ) as serial_port:
            serial_port.write(request)
            reply = serial_port.read(256)
            print(f"Response : {reply.hex(' ')}")
            if reply and reply != request:
                print("Warning: response differs from request", file=sys.stderr)
    except PermissionError:
        print(
            f"Permission denied for {args.port}. Try: sudo usermod -aG dialout $USER",
            file=sys.stderr,
        )
        return 3
    except Exception as exc:
        print(f"Serial communication failed: {exc}", file=sys.stderr)
        return 4

    return 0


if __name__ == "__main__":
    raise SystemExit(main())