"""
cn616a.py - Backend (no GUI) driver for the Omega CN616A (Modbus RTU), pymodbus 2.5.3.

Core concepts (important!):
- Absolute Setpoint (Base+0x08): the temperature setpoint you normally want.
- Control Setpoint (Base+0x0A): dynamic setpoint in RUN, OR manual output value in IDLE.

Reads/writes use pymodbus 2.5.3 conventions:
- ModbusSerialClient in pymodbus.client.sync
- keyword is `unit=`, not `slave=`
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple
import struct
import time

from pymodbus.client.sync import ModbusSerialClient
from pymodbus.exceptions import ModbusException


# ----------------------------
# System registers (0-based offsets)
# ----------------------------
SYSTEM_STATE_REG = 0x0015  # 40021

class SystemState(IntEnum):
    RUN = 0
    IDLE = 1


# PV temperatures
PV_BASE = 0x0100  # 40256 for zone 1 (float, 2 regs), +2 regs per zone

# PID register bases per zone
PID_BASE_BY_ZONE = {
    1: 0x0800,
    2: 0x0880,
    3: 0x0900,
    4: 0x0980,
    5: 0x0A00,
    6: 0x0A80,
}

# PID block offsets
PID_CONTROL_METHOD      = 0x02  # enum
PID_ABSOLUTE_SETPOINT   = 0x08  # float (2 regs)  <-- normal "temperature setpoint"
PID_CONTROL_SETPOINT    = 0x0A  # float (2 regs)  <-- dynamic SP in RUN, manual output value in IDLE
PID_P_GAIN              = 0x0C  # float (2 regs) (best-effort mapping)
PID_I_GAIN              = 0x0E  # float (2 regs)
PID_D_GAIN              = 0x10  # float (2 regs)
PID_CONTROL_OUTPUT      = 0x12  # float (2 regs) 0..100%
PID_CONTROL_MODE        = 0x1A  # enum
PID_LOOP_STATUS         = 0x17  # enum/status
PID_AUTOTUNE_CONTROL  = 0x16  # u16 enum: 0 disable/stop, 1 enable/start
PID_AUTOTUNE_SETPOINT = 0x18  # float

class ControlMethod(IntEnum):
    ON_OFF = 0
    PID = 1

class ControlMode(IntEnum):
    STANDARD = 0
    RAMP_SOAK_TERMINATE = 1
    RAMP_SOAK_HOLD = 2


class CN616AError(RuntimeError):
    pass


@dataclass
class SerialParams:
    baudrate: int = 115200
    parity: str = "N"       # 'N', 'E', 'O'
    stopbits: int = 1
    bytesize: int = 8
    timeout: float = 1.0    # seconds


class CN616A:
    """
    Minimal, safe backend driver for CN616A over Modbus RTU.

    Recommended usage:
        ctl = CN616A("COM12", slave=1)
        ctl.connect()
        ctl.set_temperature_setpoint(1, 100.0)
        ctl.set_run(True)
        print(ctl.describe(1))
        ctl.close()
    """

    def __init__(
        self,
        port: str,
        slave: int = 1,
        serial: Optional[SerialParams] = None,
        retries: int = 2,
        retry_delay: float = 0.15,
        write_quiet_s: float = 0.03,
    ):
        self.port = port
        self.slave = int(slave)
        self.serial = serial or SerialParams()
        self.retries = int(retries)
        self.retry_delay = float(retry_delay)
        self.write_quiet_s = float(write_quiet_s)

        self.client: Optional[ModbusSerialClient] = None

    # ----------------------------
    # Connection
    # ----------------------------
    def connect(self) -> None:
        if self.client is not None:
            return

        # NOTE: method="rtu" is essential for pymodbus 2.5.3 framing
        self.client = ModbusSerialClient(
            method="rtu",
            port=self.port,
            baudrate=self.serial.baudrate,
            parity=self.serial.parity,
            stopbits=self.serial.stopbits,
            bytesize=self.serial.bytesize,
            timeout=self.serial.timeout,
        )

        if not self.client.connect():
            self.client = None
            raise CN616AError(f"Failed to connect on {self.port}")

    def close(self) -> None:
        if self.client is not None:
            try:
                self.client.close()
            finally:
                self.client = None

    # ----------------------------
    # Low-level helpers
    # ----------------------------
    def _ensure_connected(self) -> ModbusSerialClient:
        if self.client is None:
            raise CN616AError("Not connected. Call connect() first.")
        return self.client

    def _do(self, fn, *args, **kwargs):
        last_exc = None
        for _ in range(self.retries + 1):
            try:
                return fn(*args, **kwargs)
            except (ModbusException, OSError) as e:
                last_exc = e
                time.sleep(self.retry_delay)
        raise CN616AError(f"Modbus operation failed after retries: {last_exc}") from last_exc

    @staticmethod
    def _regs_to_float(regs: Tuple[int, int]) -> float:
        # Big-endian: first register is MSW, second is LSW
        b = struct.pack(">HH", regs[0] & 0xFFFF, regs[1] & 0xFFFF)
        return struct.unpack(">f", b)[0]

    @staticmethod
    def _float_to_regs(val: float) -> Tuple[int, int]:
        b = struct.pack(">f", float(val))
        msw, lsw = struct.unpack(">HH", b)
        return msw, lsw

    def read_u16(self, reg: int) -> int:
        c = self._ensure_connected()
        rr = self._do(c.read_holding_registers, address=reg, count=1, unit=self.slave)
        if rr.isError():
            raise CN616AError(f"Read u16 error at 0x{reg:04X}: {rr}")
        return int(rr.registers[0])

    def write_u16(self, reg: int, value: int) -> None:
        c = self._ensure_connected()
        time.sleep(self.write_quiet_s)
        rq = self._do(c.write_register, address=reg, value=int(value) & 0xFFFF, unit=self.slave)
        if rq.isError():
            raise CN616AError(f"Write u16 error at 0x{reg:04X}: {rq}")

    def read_float(self, reg: int) -> float:
        c = self._ensure_connected()
        rr = self._do(c.read_holding_registers, address=reg, count=2, unit=self.slave)
        if rr.isError():
            raise CN616AError(f"Read float error at 0x{reg:04X}: {rr}")
        return self._regs_to_float((rr.registers[0], rr.registers[1]))

    def write_float(self, reg: int, value: float) -> None:
        c = self._ensure_connected()
        time.sleep(self.write_quiet_s)
        msw, lsw = self._float_to_regs(value)
        rq = self._do(c.write_registers, address=reg, values=[msw, lsw], unit=self.slave)
        if rq.isError():
            raise CN616AError(f"Write float error at 0x{reg:04X}: {rq}")

    # ----------------------------
    # Address helpers
    # ----------------------------
    def _pid_base(self, zone: int) -> int:
        z = int(zone)
        if z not in PID_BASE_BY_ZONE:
            raise CN616AError(f"Unsupported zone {zone}. Supported: {sorted(PID_BASE_BY_ZONE)}")
        return PID_BASE_BY_ZONE[z]

    def _pv_reg(self, zone: int) -> int:
        z = int(zone)
        if z < 1:
            raise CN616AError("zone must be >= 1")
        return PV_BASE + 2 * (z - 1)

    # ----------------------------
    # High-level status/control
    # ----------------------------
    def get_system_state(self) -> SystemState:
        return SystemState(self.read_u16(SYSTEM_STATE_REG))

    def set_run(self, run: bool) -> None:
        self.write_u16(SYSTEM_STATE_REG, SystemState.RUN if run else SystemState.IDLE)

    def read_pv(self, zone: int) -> float:
        return self.read_float(self._pv_reg(zone))

    def read_output_pct(self, zone: int) -> float:
        base = self._pid_base(zone)
        return self.read_float(base + PID_CONTROL_OUTPUT)

    def get_control_method(self, zone: int) -> ControlMethod:
        base = self._pid_base(zone)
        return ControlMethod(self.read_u16(base + PID_CONTROL_METHOD))

    def set_control_method(self, zone: int, method: ControlMethod) -> None:
        base = self._pid_base(zone)
        self.write_u16(base + PID_CONTROL_METHOD, int(method))

    def get_control_mode(self, zone: int) -> ControlMode:
        base = self._pid_base(zone)
        return ControlMode(self.read_u16(base + PID_CONTROL_MODE))

    def set_control_mode(self, zone: int, mode: ControlMode) -> None:
        base = self._pid_base(zone)
        self.write_u16(base + PID_CONTROL_MODE, int(mode))

    # ----------------------------
    # Setpoints (THE IMPORTANT PART)
    # ----------------------------
    def get_temperature_setpoint(self, zone: int) -> float:
        """Temperature setpoint (Absolute Setpoint: Base+0x08)."""
        base = self._pid_base(zone)
        return self.read_float(base + PID_ABSOLUTE_SETPOINT)

    def set_temperature_setpoint(self, zone: int, temp_c: float) -> None:
        """Set temperature setpoint in °C (Absolute Setpoint: Base+0x08)."""
        base = self._pid_base(zone)
        self.write_float(base + PID_ABSOLUTE_SETPOINT, float(temp_c))

    def get_control_setpoint_raw(self, zone: int) -> float:
        """
        Raw Control Setpoint (Base+0x0A).
        Meaning depends on SystemState:
          - RUN: dynamic control setpoint
          - IDLE: manual output value
        """
        base = self._pid_base(zone)
        return self.read_float(base + PID_CONTROL_SETPOINT)

    def set_manual_output_pct(self, zone: int, pct: float) -> None:
        """
        Manual output control (0..100%) using Control Setpoint semantics in IDLE.
        This will force IDLE first, then write Base+0x0A.

        Use with caution: this bypasses temperature regulation behavior.
        """
        if pct < 0.0 or pct > 100.0:
            raise CN616AError("Manual output percent must be in [0, 100].")

        # Force IDLE so Base+0x0A behaves as manual output value
        self.set_run(False)
        base = self._pid_base(zone)
        self.write_float(base + PID_CONTROL_SETPOINT, float(pct))
    def set_pid_gains(self, zone: int, p: float, i: float, d: float) -> None:
        base = self._pid_base(zone)
        self.write_float(base + PID_P_GAIN, float(p))
        self.write_float(base + PID_I_GAIN, float(i))
        self.write_float(base + PID_D_GAIN, float(d))

    def read_pid_gains(self, zone: int) -> tuple[float, float, float]:
        base = self._pid_base(zone)
        p = self.read_float(base + PID_P_GAIN)
        i = self.read_float(base + PID_I_GAIN)
        d = self.read_float(base + PID_D_GAIN)
        return p, i, d
    
    def set_autotune_setpoint(self, zones: int | list[int] | tuple[int, ...], setpoints: float | list[float] | tuple[float, ...]) -> None:
        """
        Set the autotune setpoint(s) (Base+0x18).

        - Backwards compatible: pass (zone, sp) as scalars.
        - Batch form: pass parallel lists/tuples of zones and setpoints, e.g.
              ctl.set_autotune_setpoint([1, 2], [80.0, 60.0])
        """
        # Normalize to lists
        if isinstance(zones, int):
            z_list = [zones]
        else:
            z_list = list(zones)

        if isinstance(setpoints, (int, float)):
            sp_list = [float(setpoints)]
        else:
            sp_list = [float(x) for x in setpoints]

        if len(z_list) != len(sp_list):
            raise CN616AError(
                f"set_autotune_setpoint expects equal-length zones and setpoints; "
                f"got {len(z_list)} zone(s) and {len(sp_list)} setpoint(s)."
            )

        # One write per zone (single step each)
        for z, sp in zip(z_list, sp_list):
            base = self._pid_base(int(z))
            self.write_float(base + PID_AUTOTUNE_SETPOINT, float(sp))

    def start_autotune(self, zones: int | list[int] | tuple[int, ...]) -> None:
        """
        Start autotune (Base+0x16 = 1).

        - Backwards compatible: pass a single zone (int).
        - Batch form: pass a list/tuple of zones, e.g.
              ctl.start_autotune([1, 2, 3])
        """
        if isinstance(zones, int):
            z_list = [zones]
        else:
            z_list = list(zones)

        # One write per zone (single step each)
        for z in z_list:
            base = self._pid_base(int(z))
            self.write_u16(base + PID_AUTOTUNE_CONTROL, 1)

    def stop_autotune(self, zone: int) -> None:
        base = self._pid_base(zone)
        self.write_u16(base + PID_AUTOTUNE_CONTROL, 0)
    
    def is_autotune_running(self, zone: int) -> bool:
        base = self._pid_base(zone)
        return self.read_u16(base + PID_AUTOTUNE_CONTROL) == 1

    # ----------------------------
    # Convenience helpers
    # ----------------------------
    def set_temperature_and_run(self, zone: int, temp_c: float) -> None:
        """Common operation: set temperature setpoint and ensure RUN."""
        self.set_temperature_setpoint(zone, temp_c)
        self.set_run(True)

    def describe(self, zone: int = 1) -> str:
        state = self.get_system_state().name
        pv = self.read_pv(zone)
        sp = self.get_temperature_setpoint(zone)
        out = self.read_output_pct(zone)
        method = self.get_control_method(zone).name
        mode = self.get_control_mode(zone).name
        return (
            f"CN616A(port={self.port}, unit={self.slave}) | state={state} | "
            f"z{zone}: PV={pv:.3f}C SP_abs={sp:.3f}C OUT={out:.2f}% | "
            f"method={method} mode={mode}"
        )


# ----------------------------
# Minimal CLI (still no GUI)
# ----------------------------
if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="CN616A backend CLI (no GUI)")
    ap.add_argument("--port", required=True, help="Serial port (e.g. COM12)")
    ap.add_argument("--slave", type=int, default=1, help="Modbus unit id (default 1)")
    ap.add_argument("--zone", type=int, default=1, help="Zone number (default 1)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--parity", choices=["N", "E", "O"], default="N")
    ap.add_argument("--stopbits", type=int, choices=[1, 2], default=1)
    ap.add_argument("--timeout", type=float, default=1.0)

    ap.add_argument("--run", action="store_true", help="Set RUN (outputs enabled)")
    ap.add_argument("--idle", action="store_true", help="Set IDLE (outputs disabled)")
    ap.add_argument("--pv", action="store_true", help="Read PV")
    ap.add_argument("--out", action="store_true", help="Read output %")
    ap.add_argument("--get-temp", action="store_true", help="Read temperature setpoint (Absolute SP)")
    ap.add_argument("--set-temp", type=float, default=None, help="Write temperature setpoint (Absolute SP)")
    ap.add_argument("--set-output", type=float, default=None, help="Manual output %% (forces IDLE, writes Control SP)")
    ap.add_argument("--pid", action="store_true", help="Set control method to PID")
    ap.add_argument("--onoff", action="store_true", help="Set control method to ON/OFF")
    ap.add_argument("--standard", action="store_true", help="Set control mode to STANDARD")
    ap.add_argument("--status", action="store_true", help="Print full status line")
    ap.add_argument("--set-pid", nargs=3, type=float, metavar=("P","I","D"),help="Set PID gains")
    ap.add_argument("--get-pid", action="store_true",help="Read PID gains")

    args = ap.parse_args()

    serial = SerialParams(
        baudrate=args.baud,
        parity=args.parity,
        stopbits=args.stopbits,
        timeout=args.timeout,
    )

    ctl = CN616A(port=args.port, slave=args.slave, serial=serial)
    ctl.connect()
    try:
        if args.pid:
            ctl.set_control_method(args.zone, ControlMethod.PID)
        if args.onoff:
            ctl.set_control_method(args.zone, ControlMethod.ON_OFF)
        if args.standard:
            ctl.set_control_mode(args.zone, ControlMode.STANDARD)

        if args.idle:
            ctl.set_run(False)
        if args.run:
            ctl.set_run(True)

        if args.set_temp is not None:
            ctl.set_temperature_setpoint(args.zone, args.set_temp)
        if args.set_output is not None:
            ctl.set_manual_output_pct(args.zone, args.set_output)

        if args.pv:
            print(f"PV(z{args.zone}) = {ctl.read_pv(args.zone):.3f} C")
        if args.get_temp:
            print(f"SP_abs(z{args.zone}) = {ctl.get_temperature_setpoint(args.zone):.3f} C")
        if args.out:
            print(f"OUT(z{args.zone}) = {ctl.read_output_pct(args.zone):.2f} %")

        if args.status:
            print(ctl.describe(args.zone))
            
        if args.set_pid:
            p, i, d = args.set_pid
            ctl.set_pid_gains(args.zone, p, i, d)

        if args.get_pid:
            p, i, d = ctl.read_pid_gains(args.zone)
            print(f"P={p}  I={i}  D={d}")

    finally:
        ctl.close()