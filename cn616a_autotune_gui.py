# -*- coding: utf-8 -*-
"""
CN616A Autotune GUI (Tkinter)

- Select Modbus slave (unit id)
- Each slave has zones 1–6
- For each zone: checkbox to include + setpoint entry
- Run button:
    1) set_autotune_setpoint([zones], [setpoints])
    2) start_autotune([zones])

Requires:
- python (standard library tkinter)
- cn616a.py in the same folder (or installed as a module)
"""

import tkinter as tk
from tkinter import ttk, messagebox

import cn616a


DEFAULT_PORT = "COM4"
SLAVE_MIN = 1
SLAVE_MAX = 32
ZONES = [1, 2, 3, 4, 5, 6]


class AutotuneGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("CN616A Autotune")
        self.resizable(False, False)

        # ----------------------------
        # Top controls
        # ----------------------------
        top = ttk.Frame(self, padding=10)
        top.grid(row=0, column=0, sticky="ew")

        ttk.Label(top, text="Port:").grid(row=0, column=0, sticky="w", padx=(0, 6))
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.port_entry = ttk.Entry(top, textvariable=self.port_var, width=10)
        self.port_entry.grid(row=0, column=1, sticky="w", padx=(0, 14))

        ttk.Label(top, text="Slave:").grid(row=0, column=2, sticky="w", padx=(0, 6))
        self.slave_var = tk.StringVar(value=str(SLAVE_MIN))
        self.slave_combo = ttk.Combobox(
            top,
            textvariable=self.slave_var,
            values=[str(i) for i in range(SLAVE_MIN, SLAVE_MAX + 1)],
            width=6,
            state="readonly",
        )
        self.slave_combo.grid(row=0, column=3, sticky="w")

        # ----------------------------
        # Zone table
        # ----------------------------
        table = ttk.Frame(self, padding=(10, 0, 10, 10))
        table.grid(row=1, column=0, sticky="ew")

        ttk.Label(table, text="Use").grid(row=0, column=0, sticky="w")
        ttk.Label(table, text="Zone").grid(row=0, column=1, sticky="w", padx=(10, 0))
        ttk.Label(table, text="Autotune Setpoint (°C)").grid(row=0, column=2, sticky="w", padx=(10, 0))

        self.zone_use_vars = {}
        self.zone_sp_vars = {}

        for r, z in enumerate(ZONES, start=1):
            use_var = tk.BooleanVar(value=False)
            sp_var = tk.StringVar(value="")

            cb = ttk.Checkbutton(table, variable=use_var)
            cb.grid(row=r, column=0, sticky="w")

            ttk.Label(table, text=str(z)).grid(row=r, column=1, sticky="w", padx=(10, 0))

            ent = ttk.Entry(table, textvariable=sp_var, width=18)
            ent.grid(row=r, column=2, sticky="w", padx=(10, 0))

            self.zone_use_vars[z] = use_var
            self.zone_sp_vars[z] = sp_var

        # ----------------------------
        # Bottom controls
        # ----------------------------
        bottom = ttk.Frame(self, padding=(10, 0, 10, 10))
        bottom.grid(row=2, column=0, sticky="ew")

        self.status_var = tk.StringVar(value="Ready.")
        status = ttk.Label(bottom, textvariable=self.status_var)
        status.grid(row=0, column=0, sticky="w")

        self.run_btn = ttk.Button(bottom, text="Run Autotune", command=self.on_run)
        self.run_btn.grid(row=0, column=1, sticky="e", padx=(20, 0))

        # make bottom expand nicely
        bottom.columnconfigure(0, weight=1)

        # convenience: double-click a row to toggle
        self.bind_all("<Return>", lambda e: self.on_run())

    def on_run(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Missing Port", "Please enter a COM port (e.g., COM4).")
            return

        try:
            slave = int(self.slave_var.get())
        except ValueError:
            messagebox.showerror("Invalid Slave", "Slave must be an integer.")
            return

        zones = []
        sps = []
        for z in ZONES:
            if self.zone_use_vars[z].get():
                raw = self.zone_sp_vars[z].get().strip()
                if raw == "":
                    messagebox.showerror("Missing Setpoint", f"Zone {z} is selected but has no setpoint.")
                    return
                try:
                    sp = float(raw)
                except ValueError:
                    messagebox.showerror("Invalid Setpoint", f"Zone {z} setpoint must be a number.")
                    return
                zones.append(z)
                sps.append(sp)

        if not zones:
            messagebox.showinfo("Nothing Selected", "Select at least one zone to autotune.")
            return

        self.run_btn.configure(state="disabled")
        self.status_var.set(f"Connecting to {port} (slave {slave})…")
        self.update_idletasks()

        ctl = None
        try:
            ctl = cn616a.CN616A(port=port, slave=slave)
            ctl.connect()

            self.status_var.set(f"Setting autotune SP for zones {zones}…")
            self.update_idletasks()
            ctl.set_autotune_setpoint(zones, sps)

            self.status_var.set(f"Starting autotune for zones {zones}…")
            self.update_idletasks()
            ctl.start_autotune(zones)

            self.status_var.set(f"Autotune started for slave {slave}, zones {zones}.")
            messagebox.showinfo("Started", f"Autotune started.\n\nSlave: {slave}\nZones: {zones}")

        except Exception as e:
            self.status_var.set("Error.")
            messagebox.showerror("Autotune Failed", str(e))

        finally:
            try:
                if ctl is not None:
                    ctl.close()
            except Exception:
                pass
            self.run_btn.configure(state="normal")


if __name__ == "__main__":
    app = AutotuneGUI()
    app.mainloop()
