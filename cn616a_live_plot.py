from __future__ import annotations

import json
import os
from dataclasses import dataclass
from datetime import datetime
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.dates as mdates


@dataclass
class PlotConfig:
    poll_ms: int = 500
    scan_newest_ms: int = 1500          # how often to check for a newer log file
    max_points: int = 3600
    default_logs_dir: str = "logs"


def _parse_iso_ts(ts: str) -> datetime:
    return datetime.fromisoformat(ts)


def _newest_jsonl(log_dir: str) -> str | None:
    if not os.path.isdir(log_dir):
        return None
    files = [
        os.path.join(log_dir, f)
        for f in os.listdir(log_dir)
        if f.lower().endswith(".jsonl")
    ]
    if not files:
        return None
    return max(files, key=lambda p: os.path.getmtime(p))


class LogTail:
    """
    Efficiently tails a JSONL file: reads only appended bytes since last read.
    """
    def __init__(self, path: str):
        self.path = path
        self._fp = None
        self._pos = 0

    def open(self, *, start_at_end: bool = True) -> None:
        self.close()
        self._fp = open(self.path, "r", encoding="utf-8")
        if start_at_end:
            self._fp.seek(0, os.SEEK_END)
        else:
            self._fp.seek(0, os.SEEK_SET)
        self._pos = self._fp.tell()

    def close(self) -> None:
        if self._fp is not None:
            try:
                self._fp.close()
            except Exception:
                pass
        self._fp = None
        self._pos = 0

    def read_new_records(self) -> list[dict]:
        if self._fp is None:
            return []

        self._fp.seek(self._pos, os.SEEK_SET)
        lines = self._fp.readlines()
        if not lines:
            return []

        # If last line is incomplete, keep it for next tick
        if lines and not lines[-1].endswith("\n"):
            complete = lines[:-1]
            if complete:
                # advance by complete lines
                self._pos += sum(len(s.encode("utf-8")) for s in complete)
            self._fp.seek(self._pos, os.SEEK_SET)
            lines = complete

        self._pos = self._fp.tell()

        out = []
        for ln in lines:
            ln = ln.strip()
            if not ln:
                continue
            try:
                out.append(json.loads(ln))
            except Exception:
                continue
        return out


class ZoneTab:
    def __init__(self, parent: ttk.Notebook, title: str, cfg: PlotConfig):
        self.cfg = cfg
        self.frame = ttk.Frame(parent)
        parent.add(self.frame, text=title)

        self.frame.rowconfigure(0, weight=1)
        self.frame.columnconfigure(0, weight=1)

        self.fig = Figure(figsize=(6.5, 3.5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("PV (°C)")
        self.ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))

        self.line, = self.ax.plot([], [], linewidth=1.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        self.status = tk.StringVar(value="—")
        ttk.Label(self.frame, textvariable=self.status).grid(row=1, column=0, sticky="ew", pady=(4, 0))

        self.t: list[datetime] = []
        self.pv: list[float] = []

    def clear(self) -> None:
        self.t.clear()
        self.pv.clear()
        self.line.set_data([], [])
        self.status.set("—")
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw_idle()

    def add_point(self, t: datetime, pv_c: float, meta: dict) -> None:
        self.t.append(t)
        self.pv.append(pv_c)

        if len(self.t) > self.cfg.max_points:
            self.t = self.t[-self.cfg.max_points:]
            self.pv = self.pv[-self.cfg.max_points:]

        sp = meta.get("sp_abs_c", None)
        out = meta.get("output_pct", None)
        method = meta.get("method", None)
        state = meta.get("state", None)
        self.status.set(f"PV={pv_c:.3f} °C   SP={sp:.3f}   Out={out:.1f}%   {state}   {method}")

    def redraw(self) -> None:
        if not self.t:
            return
        self.line.set_data(self.t, self.pv)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.autofmt_xdate()
        self.canvas.draw_idle()


class LivePlotApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("CN616A Live Temperature Viewer (auto-follow newest log)")
        self.geometry("980x640")
        self.minsize(900, 560)

        self.cfg = PlotConfig()

        self.tail: LogTail | None = None
        self.current_log = tk.StringVar(value="")
        self.auto_follow_newest = tk.BooleanVar(value=True)
        self.clear_on_switch = tk.BooleanVar(value=True)
        self.start_at_end = tk.BooleanVar(value=True)  # "live only" by default

        self.tabs: dict[int, ZoneTab] = {}

        self._build_ui()
                # Pre-create tabs for Zones 1..6
        for z in range(1, 7):
            self._get_or_create_tab(z)

        self._after_poll = self.after(self.cfg.poll_ms, self._poll_tick)
        self._after_scan = self.after(self.cfg.scan_newest_ms, self._scan_tick)

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        top = ttk.Frame(self, padding=10)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(1, weight=1)

        ttk.Label(top, text="Log file:").grid(row=0, column=0, sticky="w")
        ttk.Entry(top, textvariable=self.current_log).grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Button(top, text="Browse…", command=self._browse).grid(row=0, column=2, sticky="e")

        ttk.Button(top, text="Open", command=self._open_log_manual).grid(row=0, column=3, sticky="e", padx=(8, 0))
        ttk.Button(top, text="Close", command=self._close_log).grid(row=0, column=4, sticky="e", padx=(8, 0))

        opts = ttk.Frame(top)
        opts.grid(row=1, column=0, columnspan=5, sticky="ew", pady=(8, 0))
        ttk.Checkbutton(opts, text="Auto-follow newest log in ./logs", variable=self.auto_follow_newest).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(opts, text="Clear plots when switching to a newer log", variable=self.clear_on_switch).grid(row=0, column=1, sticky="w", padx=(16, 0))
        ttk.Checkbutton(opts, text="Start at end (live only)", variable=self.start_at_end).grid(row=0, column=2, sticky="w", padx=(16, 0))

        self.nb = ttk.Notebook(self, padding=8)
        self.nb.grid(row=1, column=0, sticky="nsew")

        self.footer = tk.StringVar(value="No log open.")
        ttk.Label(self, textvariable=self.footer, padding=(10, 0, 10, 10)).grid(row=2, column=0, sticky="ew")

        # Preselect newest if available
        newest = _newest_jsonl(self.cfg.default_logs_dir)
        if newest:
            self.current_log.set(newest)

    def _browse(self) -> None:
        initial = self.cfg.default_logs_dir if os.path.isdir(self.cfg.default_logs_dir) else os.getcwd()
        path = filedialog.askopenfilename(
            initialdir=initial,
            title="Select CN616A JSONL log file",
            filetypes=[("JSONL logs", "*.jsonl"), ("All files", "*.*")],
        )
        if path:
            self.current_log.set(path)
            self.auto_follow_newest.set(False)

    def _open_log_manual(self) -> None:
        self.auto_follow_newest.set(False)
        self._open_log(self.current_log.get().strip(), reason="manual open")

    def _open_log(self, path: str, *, reason: str) -> None:
        if not path:
            return
        if not os.path.isfile(path):
            return

        # Already open?
        if self.tail is not None and os.path.abspath(self.tail.path) == os.path.abspath(path):
            return

        try:
            self._close_log()
            self.tail = LogTail(path)
            self.tail.open(start_at_end=self.start_at_end.get())
            self.current_log.set(path)

            if self.clear_on_switch.get():
                self._clear_all_tabs()

            self.footer.set(f"Opened ({reason}): {path}")
        except Exception as e:
            self.tail = None
            messagebox.showerror("Open failed", str(e))

    def _close_log(self) -> None:
        if self.tail is not None:
            try:
                self.tail.close()
            except Exception:
                pass
        self.tail = None
        self.footer.set("No log open.")

    def _clear_all_tabs(self) -> None:
        for tab in self.tabs.values():
            tab.clear()

    def _get_or_create_tab(self, zone: int) -> ZoneTab:
        if zone in self.tabs:
            return self.tabs[zone]

        title = f"Zone {zone}"
        tab = ZoneTab(self.nb, title, self.cfg)

        # Insert tab in numeric order
        insert_index = 0
        for z in sorted(self.tabs.keys()):
            if z < zone:
                insert_index += 1

        self.nb.insert(insert_index, tab.frame, text=title)
        self.tabs[zone] = tab
        return tab

    def _poll_tick(self) -> None:
        self._after_poll = self.after(self.cfg.poll_ms, self._poll_tick)

        if self.tail is None:
            return

        recs = self.tail.read_new_records()
        if not recs:
            return

        updated: set[ZoneTab] = set()

        for r in recs:
            if r.get("type") != "telemetry":
                continue
            try:
                #slave = int(r.get("slave", 1))
                zone = int(r.get("zone", 1))
                pv_c = float(r["pv_c"])
                ts = r.get("ts")
                t = _parse_iso_ts(ts) if ts else datetime.now()
            except Exception:
                continue

            tab = self._get_or_create_tab(zone)
            tab.add_point(t, pv_c, r)
            updated.add(tab)

        for tab in updated:
            tab.redraw()

    def _scan_tick(self) -> None:
        self._after_scan = self.after(self.cfg.scan_newest_ms, self._scan_tick)

        if not self.auto_follow_newest.get():
            return

        newest = _newest_jsonl(self.cfg.default_logs_dir)
        if newest:
            self._open_log(newest, reason="auto-follow newest")

    def destroy(self) -> None:
        for aid in (getattr(self, "_after_poll", None), getattr(self, "_after_scan", None)):
            try:
                if aid is not None:
                    self.after_cancel(aid)
            except Exception:
                pass
        try:
            self._close_log()
        except Exception:
            pass
        super().destroy()


if __name__ == "__main__":
    app = LivePlotApp()
    app.mainloop()