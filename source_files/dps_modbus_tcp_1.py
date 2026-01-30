#!/usr/bin/env python3
"""
dps_modbus_tcp.py

Modbus TCP backend for DPS/XY power supply GUIs.
Intended as a replacement for dps_modbus.py (serial/minimalmodbus),
but using Modbus TCP (pymodbus).

Features:
- connect/reconnect
- thread-safe read/write
- helper methods for typical DPS/XY register map (centi-units)

Usage:
    from dps_modbus_tcp import DPSTcp

    dps = DPSTcp(host="192.168.11.53", port=502, unit_id=1)
    dps.connect()
    v, a = dps.get_set_voltage(), dps.get_set_current()
    dps.set_voltage(12.3)
    dps.set_current(1.2)
    dps.set_output(True)
"""

from __future__ import annotations

import time
import threading
from dataclasses import dataclass
from typing import Optional, List, Dict, Any

from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusIOException


# ---------------------------
# Default register mapping
# ---------------------------
DEFAULT_REGS = {
    "set_voltage": 0,       # Vset * 100
    "set_current": 1,       # Iset * 100
    "out_voltage": 2,       # Vout * 100
    "out_current": 3,       # Iout * 100
    "in_voltage": 5,        # Vin * 100 (optional)
    "status": 0x12,         # bit0 output enable
}


@dataclass
class DPSValues:
    set_voltage: float = 0.0
    set_current: float = 0.0
    out_voltage: float = 0.0
    out_current: float = 0.0
    in_voltage: float = 0.0
    output_on: bool = False
    raw_status: int = 0
    timestamp: float = 0.0


class DPSTcp:
    """
    TCP modbus interface for DPS/XY PSU.
    """

    def __init__(
        self,
        host: str,
        port: int = 502,
        unit_id: int = 1,
        timeout: float = 3.0,
        regs: Optional[Dict[str, int]] = None,
    ):
        self.host = host
        self.port = port
        self.unit_id = unit_id
        self.timeout = timeout

        self.regs = dict(DEFAULT_REGS)
        if regs:
            self.regs.update(regs)

        self._client: Optional[ModbusTcpClient] = None
        self._lock = threading.Lock()

    # ---------------------------
    # Connection
    # ---------------------------
    def connect(self) -> bool:
        with self._lock:
            try:
                if self._client:
                    self._client.close()
                self._client = ModbusTcpClient(self.host, port=self.port, timeout=self.timeout)
                return bool(self._client.connect())
            except Exception:
                self._client = None
                return False

    def close(self):
        with self._lock:
            if self._client:
                try:
                    self._client.close()
                except Exception:
                    pass
            self._client = None

    def is_connected(self) -> bool:
        with self._lock:
            return self._client is not None

    def ensure_connected(self) -> bool:
        """
        Make sure we have a connected client.
        """
        with self._lock:
            if self._client is None:
                pass
            else:
                # pymodbus sometimes keeps object but socket dead;
                # calling connect() again is safe and cheap.
                try:
                    if self._client.connect():
                        return True
                except Exception:
                    pass

        return self.connect()

    # ---------------------------
    # Low-level helpers
    # ---------------------------
    def _read_holding(self, address: int, count: int = 1) -> Optional[List[int]]:
        if not self.ensure_connected():
            return None

        with self._lock:
            try:
                rr = self._client.read_holding_registers(address, count=count, device_id=self.unit_id)
                if rr is None or isinstance(rr, ModbusIOException):
                    return None
                regs = getattr(rr, "registers", None)
                if regs is None:
                    return None
                return regs
            except Exception:
                return None

    def _write_register(self, address: int, value: int) -> bool:
        if not self.ensure_connected():
            return False

        with self._lock:
            try:
                wr = self._client.write_register(address, int(value), device_id=self.unit_id)
                return wr is not None
            except Exception:
                return False

    # ---------------------------
    # High-level: voltage/current setpoints
    # ---------------------------
    def set_voltage(self, volt: float) -> bool:
        raw = int(round(float(volt) * 100))
        return self._write_register(self.regs["set_voltage"], raw)

    def set_current(self, amp: float) -> bool:
        raw = int(round(float(amp) * 100))
        return self._write_register(self.regs["set_current"], raw)

    def get_set_voltage(self) -> Optional[float]:
        regs = self._read_holding(self.regs["set_voltage"], 1)
        if not regs:
            return None
        return regs[0] / 100.0

    def get_set_current(self) -> Optional[float]:
        regs = self._read_holding(self.regs["set_current"], 1)
        if not regs:
            return None
        return regs[0] / 100.0

    # ---------------------------
    # Output enable/disable
    # ---------------------------
    def get_status_register(self) -> Optional[int]:
        regs = self._read_holding(self.regs["status"], 1)
        if not regs:
            return None
        return int(regs[0])

    def is_output_on(self) -> Optional[bool]:
        st = self.get_status_register()
        if st is None:
            return None
        return bool(st & 0x1)

    def set_output(self, on: bool) -> bool:
        st = self.get_status_register()
        if st is None:
            return False

        if on:
            new_st = st | 0x1
        else:
            new_st = st & ~0x1

        return self._write_register(self.regs["status"], new_st)

    def toggle_output(self) -> bool:
        st = self.get_status_register()
        if st is None:
            return False
        new_st = st ^ 0x1
        return self._write_register(self.regs["status"], new_st)

    # ---------------------------
    # Full snapshot read
    # ---------------------------
    def read_all(self) -> Optional[DPSValues]:
        """
        Read all relevant values as one snapshot.
        Does multiple reads for robustness.
        """

        # read starting at 0, length 0x0E typical for these devices
        regs = self._read_holding(self.regs["set_voltage"], count=0x0E)
        if not regs:
            return None

        status = self.get_status_register()
        if status is None:
            status = 0

        def safe(idx: int) -> int:
            if idx < 0 or idx >= len(regs):
                return 0
            return int(regs[idx])

        vals = DPSValues(
            set_voltage=safe(0) / 100.0,
            set_current=safe(1) / 100.0,
            out_voltage=safe(2) / 100.0,
            out_current=safe(3) / 100.0,
            in_voltage=safe(self.regs.get("in_voltage", 5)) / 100.0,
            raw_status=status,
            output_on=bool(status & 0x1),
            timestamp=time.time(),
        )

        return vals


# ---------------------------
# Simple manual test
# ---------------------------
if __name__ == "__main__":
    dps = DPSTcp(host="192.168.11.53", port=502, unit_id=1)
    print("connect:", dps.connect())
    print("values:", dps.read_all())
