#!/usr/bin/env python3
"""
Launch foxglove_bridge with a friendlier pre-flight port check.

Environment variables:
  FOXGLOVE_PORT               default: 8765
  FOXGLOVE_ADDRESS            default: 0.0.0.0
  FOXGLOVE_SEND_BUFFER_LIMIT  default: 524288000
"""

from __future__ import annotations

import errno
import os
import socket
import sys


def _parse_port(value: str) -> int:
    try:
        port = int(value)
    except ValueError as exc:
        raise SystemExit(f"FOXGLOVE_PORT must be an integer, got: {value!r}") from exc
    if not (1 <= port <= 65535):
        raise SystemExit(f"FOXGLOVE_PORT must be between 1 and 65535, got: {port}")
    return port


def _socket_family(address: str) -> socket.AddressFamily:
    return socket.AF_INET6 if ":" in address and "." not in address else socket.AF_INET


def _probe_bind(address: str, port: int) -> tuple[bool, str]:
    family = _socket_family(address)
    sock = socket.socket(family, socket.SOCK_STREAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((address, port))
        return True, ""
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            return False, (
                f"Foxglove bridge cannot bind to {address}:{port} because that port is already in use.\n"
                f"Try a different port, for example:\n"
                f"  FOXGLOVE_PORT={port + 1} pixi run foxglove\n\n"
                f"To see what is listening on that port:\n"
                f"  lsof -nP -iTCP:{port} -sTCP:LISTEN"
            )
        if exc.errno == errno.EACCES:
            return False, f"Permission denied while trying to bind {address}:{port}."
        return False, f"Could not probe {address}:{port}: {exc}"
    finally:
        sock.close()


def main() -> int:
    port = _parse_port(os.environ.get("FOXGLOVE_PORT", "8765"))
    address = os.environ.get("FOXGLOVE_ADDRESS", "0.0.0.0")
    send_buffer_limit = os.environ.get("FOXGLOVE_SEND_BUFFER_LIMIT", "524288000")

    ok, message = _probe_bind(address, port)
    if not ok:
        print(message, file=sys.stderr)
        return 1

    cmd = [
        "ros2",
        "launch",
        "foxglove_bridge",
        "foxglove_bridge_launch.xml",
        f"port:={port}",
        f"address:={address}",
        f"send_buffer_limit:={send_buffer_limit}",
        *sys.argv[1:],
    ]
    os.execvp(cmd[0], cmd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
