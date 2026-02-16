"""
cn616a_gui.py - Tkinter GUI for cn616a.py backend.

Place this file next to cn616a.py and run:
    python cn616a_gui.py
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk, messagebox
import time
from collections import deque
from dataclasses import dataclass
import json
import os
from datetime import datetime

try:
    from zoneinfo import ZoneInfo  # py>=3.9
except Exception:  # pragma: no cover
    ZoneInfo = None

# Import your backend
from cn616a import (
    CN616A,
    CN616AError,
    SerialParams,
    ControlMethod,
)

# Optional COM port enumeration (pyserial)
try:
    from serial.tools import list_ports
except Exception:  # pragma: no cover
    list_ports = None

def parse_int_list(spec: str, *, lo: int, hi: int) -> list[int]:
    """
    Parse strings like: "1-6, 8, 10-12" into sorted unique ints.
    """
    out: set[int] = set()
    s = (spec or "").strip()
    if not s:
        return []
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            a = int(a.strip())
            b = int(b.strip())
            if a > b:
                a, b = b, a
            for v in range(a, b + 1):
                if lo <= v <= hi:
                    out.add(v)
        else:
            v = int(part)
            if lo <= v <= hi:
                out.add(v)
    return sorted(out)

@dataclass
class UiDefaults:
    poll_ms: int = 750
    baudrate: int = 115200
    parity: str = "N"
    stopbits: int = 1
    timeout: float = 1.0
    slave: int = 1
    zone: int = 1


class CN616AGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Omega CN616A Controller (Modbus RTU)")
        self.geometry("940x560")
        self.minsize(880, 520)

        self.defaults = UiDefaults()

        self.ctl: CN616A | None = None
        self.connected = tk.BooleanVar(value=False)
        self.polling = tk.BooleanVar(value=True)
        self._poll_after_id: str | None = None

        # --- Equilibrium tracking (per slave, per zone) ---
        self._eq_window_n = 100
        self._eq_threshold = 10.0  # sum(|SP-PV|) over last N samples
        self._eq_deque: dict[tuple[int,int], deque[float]] = {}

        # ----------------------------
        # Tk variables
        # ----------------------------
        self.var_port = tk.StringVar(value="COM12")
        self.var_slave = tk.IntVar(value=self.defaults.slave)
        self.var_zone = tk.IntVar(value=self.defaults.zone)

        self.var_baud = tk.IntVar(value=self.defaults.baudrate)
        self.var_parity = tk.StringVar(value=self.defaults.parity)
        self.var_stopbits = tk.IntVar(value=self.defaults.stopbits)
        self.var_timeout = tk.DoubleVar(value=self.defaults.timeout)

        self.var_state = tk.StringVar(value="—")
        self.var_pv = tk.StringVar(value="—")
        self.var_sp_abs = tk.StringVar(value="—")
        self.var_out = tk.StringVar(value="—")
        self.var_method = tk.StringVar(value="—")
        self.var_mode = tk.StringVar(value="—")

        self.var_autotune = tk.StringVar(value="—")
        self.var_eq_sum = tk.StringVar(value="—")
        self.var_eq = tk.StringVar(value="—")

        self.var_set_temp = tk.StringVar(value="")
        self.var_set_output = tk.StringVar(value="")

        self.var_pid_p = tk.StringVar(value="")
        self.var_pid_i = tk.StringVar(value="")
        self.var_pid_d = tk.StringVar(value="")
        self.var_poll_slaves = tk.StringVar(value=str(self.defaults.slave))  # e.g. "1" or "1-10"
        self.var_poll_zones  = tk.StringVar(value="1-6")                    # CN616A supports zones 1..6

        # ---- Logging (JSONL) ----
        self._tz = ZoneInfo("America/Los_Angeles") if ZoneInfo else None

        log_dir = os.path.join(os.getcwd(), "logs")
        os.makedirs(log_dir, exist_ok=True)

        # One file per session
        ts = self._now().strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join(log_dir, f"cn616a_log_{ts}.jsonl")

        self._write_log({
            "type": "event",
            "event": "gui_start",
            "message": "GUI started",
        })

        # ----------------------------
        # Layout
        # ----------------------------
        self._build_ui()
        self._refresh_ports()

        # Start polling loop
        self.after(150, self._poll_tick)

        # Close handler
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ----------------------------
    # UI construction
    # ----------------------------
    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        top = ttk.Frame(self, padding=10)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(0, weight=1)

        main = ttk.Frame(self, padding=(10, 0, 10, 10))
        main.grid(row=1, column=0, sticky="nsew")
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(1, weight=1)

        # --- Connection group
        conn = ttk.LabelFrame(top, text="Connection", padding=10)
        conn.grid(row=0, column=0, sticky="ew")
        for c in range(12):
            conn.columnconfigure(c, weight=0)
        conn.columnconfigure(11, weight=1)

        ttk.Label(conn, text="Port:").grid(row=0, column=0, sticky="w")
        self.cmb_port = ttk.Combobox(conn, textvariable=self.var_port, width=12)
        self.cmb_port.grid(row=0, column=1, sticky="w", padx=(6, 14))

        ttk.Button(conn, text="Refresh", command=self._refresh_ports).grid(
            row=0, column=2, sticky="w", padx=(0, 14)
        )

        ttk.Label(conn, text="Baud:").grid(row=0, column=3, sticky="w")
        ent_baud = ttk.Entry(conn, width=8, textvariable=self.var_baud)
        ent_baud.grid(row=0, column=4, sticky="w", padx=(6, 14))

        ttk.Label(conn, text="Parity:").grid(row=0, column=5, sticky="w")
        cmb_parity = ttk.Combobox(
            conn, width=4, textvariable=self.var_parity, values=["N", "E", "O"], state="readonly"
        )
        cmb_parity.grid(row=0, column=6, sticky="w", padx=(6, 14))

        # Second row in conn
        ttk.Label(conn, text="Stopbits:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        cmb_stop = ttk.Combobox(conn, width=4, textvariable=self.var_stopbits, values=[1, 2], state="readonly")
        cmb_stop.grid(row=1, column=1, sticky="w", padx=(6, 14), pady=(8, 0))

        ttk.Label(conn, text="Timeout (s):").grid(row=1, column=2, sticky="w", pady=(8, 0))
        ent_to = ttk.Entry(conn, width=8, textvariable=self.var_timeout)
        ent_to.grid(row=1, column=3, sticky="w", padx=(6, 14), pady=(8, 0))

        self.btn_connect = ttk.Button(conn, text="Connect", command=self._connect)
        self.btn_connect.grid(row=1, column=4, sticky="w", pady=(8, 0))

        self.btn_disconnect = ttk.Button(conn, text="Disconnect", command=self._disconnect, state="disabled")
        self.btn_disconnect.grid(row=1, column=5, sticky="w", padx=(8, 0), pady=(8, 0))

        ttk.Checkbutton(conn, text="Auto-refresh", variable=self.polling).grid(
            row=1, column=6, sticky="w", padx=(16, 0), pady=(8, 0)
        )

        # --- Status panel (left)
        status = ttk.LabelFrame(main, text="Live Status", padding=10)
        status.grid(row=0, column=0, sticky="ew", padx=(0, 8))
        status.columnconfigure(1, weight=1)

        def status_row(r: int, label: str, var: tk.StringVar):
            ttk.Label(status, text=label).grid(row=r, column=0, sticky="w", pady=2)
            ttk.Label(status, textvariable=var, font=("Segoe UI", 10, "bold")).grid(row=r, column=1, sticky="w", pady=2)

        status_row(0, "State:", self.var_state)
        status_row(1, "PV (°C):", self.var_pv)
        status_row(2, "SP_abs (°C):", self.var_sp_abs)
        status_row(3, "Output (%):", self.var_out)
        status_row(4, "Method:", self.var_method)
        status_row(5, "Mode:", self.var_mode)
        status_row(6, "Autotune:", self.var_autotune)
        status_row(7, "Eq (Σ|e|):", self.var_eq_sum)
        status_row(8, "Equilibrium:", self.var_eq)

        # (Removed RUN/IDLE/Read Now buttons from Live Status)

        # --- Controls panel (right)
        controls = ttk.LabelFrame(main, text="Controls", padding=10)
        controls.grid(row=0, column=1, sticky="ew")
        for c in range(8):
            controls.columnconfigure(c, weight=0)
        controls.columnconfigure(1, weight=1)
        ttk.Label(controls, text="Poll Slaves:").grid(row=0, column=0, sticky="w")
        ttk.Entry(controls, textvariable=self.var_poll_slaves, width=14).grid(row=0, column=1, sticky="w", padx=(6, 14))

        ttk.Label(controls, text="Poll Zones:").grid(row=0, column=2, sticky="w")
        ttk.Entry(controls, textvariable=self.var_poll_zones, width=14).grid(row=0, column=3, sticky="w", padx=(6, 0))

        # Optional little hint label
        ttk.Label(controls, text='(e.g. "1-6,8")').grid(row=0, column=4, sticky="w", padx=(10, 0))

        # Slave + Zone moved here
        ttk.Label(controls, text="Slave:").grid(row=0, column=0, sticky="w")
        sp_slave = ttk.Spinbox(controls, from_=1, to=247, width=6, textvariable=self.var_slave)
        sp_slave.grid(row=0, column=1, sticky="w", padx=(6, 14))

        ttk.Label(controls, text="Zone:").grid(row=0, column=2, sticky="w")
        sp_zone = ttk.Spinbox(controls, from_=1, to=6, width=6, textvariable=self.var_zone)
        sp_zone.grid(row=0, column=3, sticky="w", padx=(6, 0))

        # Set temperature (Absolute Setpoint)
        ttk.Label(controls, text="Set Temp (°C):").grid(row=1, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(controls, textvariable=self.var_set_temp, width=12).grid(
            row=1, column=1, sticky="w", padx=(6, 10), pady=(10, 0)
        )
        ttk.Button(controls, text="Write", command=self._write_temp).grid(
            row=1, column=2, sticky="w", pady=(10, 0)
        )
        # (Removed "Write + RUN")

        # Manual output (no Set button now)
        ttk.Label(controls, text="Manual Output (%):").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(controls, textvariable=self.var_set_output, width=12).grid(
            row=2, column=1, sticky="w", padx=(6, 10), pady=(8, 0)
        )
        # (Removed "Set (forces IDLE)")

        ttk.Separator(controls).grid(row=3, column=0, columnspan=8, sticky="ew", pady=10)

        # Control method quick set
        ttk.Button(controls, text="Method: PID", command=lambda: self._set_method(ControlMethod.PID)).grid(row=4, column=0, sticky="w")
        ttk.Button(controls, text="Method: ON/OFF", command=lambda: self._set_method(ControlMethod.ON_OFF)).grid(row=4, column=1, sticky="w", padx=(8, 0))

        # (Removed Mode button)

        ttk.Separator(controls).grid(row=5, column=0, columnspan=8, sticky="ew", pady=10)

        # PID gains
        ttk.Label(controls, text="PID Gains").grid(row=6, column=0, sticky="w")

        ttk.Label(controls, text="P:").grid(row=7, column=0, sticky="w")
        ttk.Entry(controls, textvariable=self.var_pid_p, width=12).grid(row=7, column=1, sticky="w", padx=(6, 10))
        ttk.Label(controls, text="I:").grid(row=7, column=2, sticky="w")
        ttk.Entry(controls, textvariable=self.var_pid_i, width=12).grid(row=7, column=3, sticky="w", padx=(6, 10))
        ttk.Label(controls, text="D:").grid(row=7, column=4, sticky="w")
        ttk.Entry(controls, textvariable=self.var_pid_d, width=12).grid(row=7, column=5, sticky="w", padx=(6, 0))

        ttk.Button(controls, text="Read PID", command=self._read_pid).grid(row=8, column=0, sticky="w", pady=(8, 0))
        ttk.Button(controls, text="Write PID", command=self._write_pid).grid(row=8, column=1, sticky="w", pady=(8, 0), padx=(8, 0))

        # --- Log panel (bottom)
        log = ttk.LabelFrame(main, text="Log", padding=10)
        log.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(10, 0))
        log.rowconfigure(0, weight=1)
        log.columnconfigure(0, weight=1)

        self.txt_log = tk.Text(log, height=10, wrap="word")
        self.txt_log.grid(row=0, column=0, sticky="nsew")
        scr = ttk.Scrollbar(log, orient="vertical", command=self.txt_log.yview)
        scr.grid(row=0, column=1, sticky="ns")
        self.txt_log.configure(yscrollcommand=scr.set)

        self._log("GUI started.")

    # ----------------------------
    # Helpers
    # ----------------------------
    def _log(self, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self.txt_log.insert("end", f"[{ts}] {msg}\n")
        self.txt_log.see("end")

    def _set_connected_ui(self, is_conn: bool) -> None:
        self.connected.set(is_conn)
        self.btn_connect.configure(state=("disabled" if is_conn else "normal"))
        self.btn_disconnect.configure(state=("normal" if is_conn else "disabled"))

    def _refresh_ports(self) -> None:
        ports = []
        if list_ports is not None:
            try:
                ports = [p.device for p in list_ports.comports()]
            except Exception:
                ports = []

        current = self.var_port.get().strip()
        if current and current not in ports:
            ports = [current] + ports

        self.cmb_port["values"] = ports if ports else [current or "COM1"]
        if not current and ports:
            self.var_port.set(ports[0])

        if list_ports is None:
            self._log("pyserial not available; COM ports list may be empty (manual entry still works).")
        else:
            self._log(f"Ports refreshed: {', '.join(ports) if ports else '(none found)'}")

    def _get_ctl(self) -> CN616A:
        if self.ctl is None:
            raise CN616AError("Not connected.")
        return self.ctl

    def _make_serial_params(self) -> SerialParams:
        return SerialParams(
            baudrate=int(self.var_baud.get()),
            parity=str(self.var_parity.get()).strip().upper(),
            stopbits=int(self.var_stopbits.get()),
            timeout=float(self.var_timeout.get()),
        )

    def _zone(self) -> int:
        return int(self.var_zone.get())

    def _now(self) -> datetime:
        if self._tz is not None:
            return datetime.now(self._tz)
        return datetime.now()

    def _write_log(self, rec: dict) -> None:
        """
        Append one JSON record (one line) to the session log.
        """
        rec = dict(rec)  # copy
        rec["ts"] = self._now().isoformat()

        # Optional: a stable epoch field (handy for pandas)
        try:
            rec["t_epoch_s"] = self._now().timestamp()
        except Exception:
            pass

        with open(self.log_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")



    def _update_equilibrium(self, slave: int, zone: int, pv_c: float, sp_c: float) -> tuple[float, bool]:
        """
        Update and return (sum_abs_error, is_equilibrium) for a given (slave, zone).

        Definition implemented (matches your description):
            S(t) = sum_{window} |SP_current(t) - PV_i|

        Implementation notes:
        - We store a rolling window of PV samples only (per slave, per zone).
        - On each poll, we recompute sum(|SP_current - PV_i|) over the window.
          This makes S respond *immediately* to SP changes (including changing back).
        - Equilibrium heuristic: once we have N samples, equilibrium is S < threshold.
        """
        key = (int(slave), int(zone))
        sp_now = float(sp_c)
        pv_now = float(pv_c)

        dq = self._eq_deque.get(key)
        if dq is None:
            dq = deque(maxlen=self._eq_window_n)
            self._eq_deque[key] = dq

        dq.append(pv_now)

        s = 0.0
        for pv in dq:
            s += abs(sp_now - pv)

        is_eq = (len(dq) == dq.maxlen) and (s < self._eq_threshold)
        return s, is_eq

    # ----------------------------
    # Connection actions
    # ----------------------------
    def _connect(self) -> None:
        if self.ctl is not None:
            return
        port = self.var_port.get().strip()
        if not port:
            messagebox.showerror("Missing port", "Please enter/select a serial port (e.g., COM12).")
            return

        serial = self._make_serial_params()
        slave = int(self.var_slave.get())
        target_slave = int(self.var_slave.get())
        target_zone  = int(self.var_zone.get())
        
        try:
            self.ctl = CN616A(port=port, slave=slave, serial=serial)
            self.ctl.connect()
            self._set_connected_ui(True)
            self._log(f"Connected to {port} (unit={slave}).")
            self._read_all()
            self._write_log({
                "type": "event",
                "event": "connect",
                "port": port,
                "slave": slave,
                "serial": {
                    "baudrate": serial.baudrate,
                    "parity": serial.parity,
                    "stopbits": serial.stopbits,
                    "timeout": serial.timeout,
                    },
                })

        except Exception as e:
            self.ctl = None
            self._set_connected_ui(False)
            self._log(f"Connect failed: {e}")
            messagebox.showerror("Connect failed", str(e))
            self._write_log({
                "type": "event",
                "event": "connect_failed",
                "port": port,
                "slave": target_slave,
                "zone": target_zone,
                "error": str(e),
            })


    def _disconnect(self) -> None:
        if self.ctl is None:
            return
        try:
            self.ctl.close()
            self._write_log({
                "type": "event",
                "event": "disconnect",
            })
        except Exception as e:
            self._log(f"Disconnect warning: {e}")
            self._write_log({
                "type": "event",
                "event": "disconnect",
                "error": e
            })
        finally:
            self.ctl = None
            self._set_connected_ui(False)
            self._log("Disconnected.")
            self._set_status_placeholders()

    def _set_status_placeholders(self) -> None:
        for v in (self.var_state, self.var_pv, self.var_sp_abs, self.var_out, self.var_method, self.var_mode, self.var_autotune, self.var_eq_sum, self.var_eq):
            v.set("—")

    # ----------------------------
    # Read / write actions
    # ----------------------------
    def _read_all(self) -> None:
        ctl = self._get_ctl()
        z = self._zone()
        state = ctl.get_system_state()
        pv = ctl.read_pv(z)
        sp = ctl.get_temperature_setpoint(z)
        out = ctl.read_output_pct(z)
        method = ctl.get_control_method(z)
        mode = ctl.get_control_mode(z)
        at = ctl.is_autotune_running(z)

        eq_sum, eq_ok = self._update_equilibrium(int(self.var_slave.get()), z, float(pv), float(sp))

        self.var_state.set(state.name)
        self.var_pv.set(f"{pv:.3f}")
        self.var_sp_abs.set(f"{sp:.3f}")
        self.var_out.set(f"{out:.2f}")
        self.var_method.set(method.name)
        self.var_mode.set(mode.name)
        self.var_autotune.set("ON" if at else "off")
        self.var_eq_sum.set(f"{eq_sum:.2f}")
        self.var_eq.set("YES" if eq_ok else "no")

                # Telemetry log (easy time-vs-temp extraction)
        self._write_log({
            "type": "telemetry",
            "port": self.var_port.get().strip(),
            "slave": int(self.var_slave.get()),
            "zone": z,
            "state": state.name,
            "pv_c": float(pv),
            "sp_abs_c": float(sp),
            "output_pct": float(out),
            "method": method.name,
            "mode": mode.name,
            "autotune": bool(at),
            "eq_sum_abs": float(eq_sum),
            "equilibrium": bool(eq_ok),
        })

    def _read_all_telemetry(self) -> None:
        """
        Read telemetry for all configured slaves and zones, and log one record per (slave, zone).
        """
        ctl = self._get_ctl()

        slaves = parse_int_list(self.var_poll_slaves.get(), lo=1, hi=247)
        zones  = parse_int_list(self.var_poll_zones.get(),  lo=1, hi=6)

        if not slaves:
            slaves = [int(self.var_slave.get())]  # fallback
        if not zones:
            zones = [int(self.var_zone.get())]    # fallback

        port = self.var_port.get().strip()

        # Iterate all slaves and zones
        for slave in slaves:
            # switch unit id on the fly
            old_slave = getattr(ctl, "slave", None)
            ctl.slave = slave

            try:
                state = ctl.get_system_state()

                for z in zones:
                    pv = ctl.read_pv(z)
                    sp = ctl.get_temperature_setpoint(z)
                    out = ctl.read_output_pct(z)
                    method = ctl.get_control_method(z)
                    mode = ctl.get_control_mode(z)
                    at = ctl.is_autotune_running(z)
                    eq_sum, eq_ok = self._update_equilibrium(slave, z, float(pv), float(sp))

                    # Update the GUI "Live Status" with *something* useful:
                    # (we'll show the currently-selected slave/zone)
                    if slave == int(self.var_slave.get()) and z == int(self.var_zone.get()):
                        self.var_state.set(state.name)
                        self.var_pv.set(f"{pv:.3f}")
                        self.var_sp_abs.set(f"{sp:.3f}")
                        self.var_out.set(f"{out:.2f}")
                        self.var_method.set(method.name)
                        self.var_mode.set(mode.name)
                        self.var_autotune.set("ON" if at else "off")
                        self.var_eq_sum.set(f"{eq_sum:.2f}")
                        self.var_eq.set("YES" if eq_ok else "no")

                    # TELEMETRY LOG: one line per slave+zone
                    self._write_log({
                        "type": "telemetry",
                        "port": port,
                        "slave": slave,
                        "zone": z,
                        "state": state.name,
                        "pv_c": float(pv),
                        "sp_abs_c": float(sp),
                        "output_pct": float(out),
                        "method": method.name,
                        "mode": mode.name,
                        "autotune": bool(at),
                        "eq_sum_abs": float(eq_sum),
                        "equilibrium": bool(eq_ok),
                    })

            except Exception as e:
                # Log failure clearly as non-data
                self._write_log({
                    "type": "event",
                    "event": "telemetry_read_failed",
                    "port": port,
                    "slave": slave,
                    "error": str(e),
                })

            finally:
                # restore previous slave
                if old_slave is not None:
                    ctl.slave = old_slave

    def _write_temp(self) -> None:
        ctl = self._get_ctl()
        z = self._zone()
        target_slave = int(self.var_slave.get())
        target_zone  = int(self.var_zone.get())
        try:
            val = float(self.var_set_temp.get().strip())
            ctl.set_temperature_setpoint(z, val)
            self._log(f"Wrote SP_abs(z{z}) = {val} °C")
            self._read_all()
            self._write_log({
                "type": "command",
                "command": "set_temperature_setpoint",
                "slave": target_slave,
                "zone": target_zone,
                "value_c": val,
            })
        except ValueError:
            messagebox.showerror("Invalid temperature", "Enter a numeric temperature (°C).")
        except Exception as e:
            self._log(f"Write temp failed: {e}")
            messagebox.showerror("Write failed", str(e))
            self._write_log({
                "type": "command",
                "command": "set_temperature_setpoint",
                "slave": target_slave,
                "zone": target_zone,
                "value_c": val,
                "error": e
            })

    def _set_method(self, method: ControlMethod) -> None:
        ctl = self._get_ctl()
        z = self._zone()
        target_slave = int(self.var_slave.get())
        target_zone  = int(self.var_zone.get())
        try:
            ctl.set_control_method(z, method)
            self._log(f"Set control method: {method.name} (z{z})")
            self._read_all()
            self._write_log({
                "type": "command",
                "command": "set_control_method",
                "slave": target_slave,
                "zone": target_zone,
                "method": method.name,
            })
        except Exception as e:
            self._log(f"Set method failed: {e}")
            messagebox.showerror("Write failed", str(e))
            self._write_log({
                "type": "command",
                "command": "set_control_method",
                "slave": target_slave,
                "zone": target_zone,
                "method": method.name,
                "error": e,
            })

    def _read_pid(self) -> None:
        ctl = self._get_ctl()
        z = self._zone()
        target_slave = int(self.var_slave.get())
        target_zone = int(self.var_zone.get())

        # Ensure reads go to the selected slave
        old_slave = getattr(ctl, "slave", None)
        ctl.slave = target_slave

        try:
            self._write_log({
                "type": "command",
                "command": "read_pid_gains",
                "slave": target_slave,
                "zone": target_zone,
                "status": "attempt",
            })

            p, i, d = ctl.read_pid_gains(z)

            self.var_pid_p.set(f"{p}")
            self.var_pid_i.set(f"{i}")
            self.var_pid_d.set(f"{d}")

            self._log(f"Read PID gains (z{z}): P={p}, I={i}, D={d}")

            self._write_log({
                "type": "command",
                "command": "read_pid_gains",
                "slave": target_slave,
                "zone": target_zone,
                "status": "ok",
                "p": p,
                "i": i,
                "d": d,
            })

        except Exception as e:
            self._log(f"Read PID failed: {e}")
            messagebox.showerror("Read failed", str(e))

            self._write_log({
                "type": "command",
                "command": "read_pid_gains",
                "slave": target_slave,
                "zone": target_zone,
                "status": "error",
                "error": str(e),
            })

        finally:
            if old_slave is not None:
                ctl.slave = old_slave

    def _write_pid(self) -> None:
        ctl = self._get_ctl()
        z = self._zone()
        target_slave = int(self.var_slave.get())
        target_zone = int(self.var_zone.get())

        old_slave = getattr(ctl, "slave", None)
        ctl.slave = target_slave

        try:
            p = float(self.var_pid_p.get().strip())
            i = float(self.var_pid_i.get().strip())
            d = float(self.var_pid_d.get().strip())

            self._write_log({
                "type": "command",
                "command": "set_pid_gains",
                "slave": target_slave,
                "zone": target_zone,
                "status": "attempt",
                "p": p,
                "i": i,
                "d": d,
            })

            ctl.set_pid_gains(z, p, i, d)

            self._log(f"Wrote PID gains (z{z}): P={p}, I={i}, D={d}")

            self._write_log({
                "type": "command",
                "command": "set_pid_gains",
                "slave": target_slave,
                "zone": target_zone,
                "status": "ok",
                "p": p,
                "i": i,
                "d": d,
            })

            self._read_all()  # optional refresh

        except ValueError:
            messagebox.showerror("Invalid PID", "Enter numeric values for P, I, and D.")
            self._write_log({
                "type": "command",
                "command": "set_pid_gains",
                "slave": target_slave,
                "zone": target_zone,
                "status": "error",
                "error": "ValueError: non-numeric PID input",
            })

        except Exception as e:
            self._log(f"Write PID failed: {e}")
            messagebox.showerror("Write failed", str(e))
            self._write_log({
                "type": "command",
                "command": "set_pid_gains",
                "slave": target_slave,
                "zone": target_zone,
                "status": "error",
                "error": str(e),
            })

        finally:
            if old_slave is not None:
                ctl.slave = old_slave

    # ----------------------------
    # Poll loop
    # ----------------------------
    def _poll_tick(self) -> None:
        self._poll_after_id = self.after(self.defaults.poll_ms, self._poll_tick)

        if not self.polling.get():
            return
        if self.ctl is None:
            return

        try:
            self._read_all_telemetry()
        except Exception as e:
            self._log(f"Poll read failed: {e}")
            self._write_log({"type": "event", "event": "poll_exception", "error": str(e)})

    # ----------------------------
    # Close
    # ----------------------------
    def _on_close(self) -> None:
        try:
            if self._poll_after_id is not None:
                self.after_cancel(self._poll_after_id)
        except Exception:
            pass

        try:
            if self.ctl is not None:
                self.ctl.close()
        except Exception:
            pass

        self.destroy()


if __name__ == "__main__":
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

    app = CN616AGui()
    app.mainloop()