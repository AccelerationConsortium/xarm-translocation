"""Pytest plugin rejecting all outbound IP sockets during legacy regression tests."""
import socket

_original_connect = socket.socket.connect
_original_connect_ex = socket.socket.connect_ex


def _connect(self, address):
    if self.family in (socket.AF_INET, socket.AF_INET6):
        raise RuntimeError("Offline regression suite forbids IP network connections")
    return _original_connect(self, address)


def _connect_ex(self, address):
    if self.family in (socket.AF_INET, socket.AF_INET6):
        raise RuntimeError("Offline regression suite forbids IP network connections")
    return _original_connect_ex(self, address)


def pytest_sessionstart(session):
    socket.socket.connect = _connect
    socket.socket.connect_ex = _connect_ex

