"""Pytest plugin rejecting outbound IP sockets during legacy regression tests.

Loopback stays allowed: on Windows, asyncio's ProactorEventLoop builds its
self-pipe with a 127.0.0.1 socketpair, so every async test would otherwise
error before reaching the code under test. No robot lives on loopback.
"""
import ipaddress
import socket

_original_connect = socket.socket.connect
_original_connect_ex = socket.socket.connect_ex


def _forbidden(sock, address):
    if sock.family not in (socket.AF_INET, socket.AF_INET6):
        return False
    try:
        return not ipaddress.ip_address(address[0]).is_loopback
    except (ValueError, IndexError, TypeError):
        return True  # hostnames would need DNS: refuse


def _connect(self, address):
    if _forbidden(self, address):
        raise RuntimeError("Offline regression suite forbids IP network connections")
    return _original_connect(self, address)


def _connect_ex(self, address):
    if _forbidden(self, address):
        raise RuntimeError("Offline regression suite forbids IP network connections")
    return _original_connect_ex(self, address)


def pytest_sessionstart(session):
    socket.socket.connect = _connect
    socket.socket.connect_ex = _connect_ex

