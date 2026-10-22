"""Interactive dashboard for the drone swarm simulation (tkinter).

Layout::

    +---------------------------------------------------------------+
    | toolbar: start / pause / reset / step / route / export        |
    +---------+---------------------------------------+-----------+
    | control |                                       | Dashboard |
    | panel   |        simulation canvas              | Log       |
    |         |                                       | Experiments|
    |         |                                       | Charts    |
    +---------+---------------------------------------+-----------+
    | status bar                                                   |
    +---------------------------------------------------------------+
"""

from __future__ import annotations

import os
import queue
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import filedialog, ttk
from typing import Any, Dict, List, Optional

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from ..config import SimConfig
from ..experiments import (ExperimentResult, export_csv, plot_results,
                           run_all_experiments)
from ..simulation import Simulation
from .canvas import COLORS, SwarmCanvas

FONT = ("Segoe UI", 9)
FONT_BOLD = ("Segoe UI", 9, "bold")
TITLE_FONT = ("Segoe UI", 10, "bold")


class DroneSwarmApp:
    """The desktop application."""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Autonomous Drone Swarm Communication Simulation")
        self.root.geometry("1420x860")
        self.root.minsize(1100, 700)
        self.root.configure(bg="#0b1220")

        self.config = SimConfig()
        self.sim = Simulation(self.config)
        self.sim.reset()

        self.selected_drone: Optional[str] = None
        self.last_event_index = 0
        self.history_time: deque = deque(maxlen=240)
        self.history_pdr: deque = deque(maxlen=240)
        self.history_latency: deque = deque(maxlen=240)
        self.experiment_results: List[ExperimentResult] = []
        self.worker_queue: "queue.Queue[tuple]" = queue.Queue()
        self.experiment_running = False

        self._configure_style()
        self._build_toolbar()
        self._build_body()
        self._build_statusbar()

        self._refresh_dashboard()
        self._refresh_drone_list()
        self.root.after(30, self._tick)

    # ------------------------------------------------------------------
    # styling
    # ------------------------------------------------------------------
    def _configure_style(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", background="#0b1220", foreground="#e8eefc", fieldbackground="#131c30")
        style.configure("TFrame", background="#0b1220")
        style.configure("TLabel", background="#0b1220", foreground="#e8eefc", font=FONT)
        style.configure("TLabelframe", background="#0b1220", foreground="#8fd0e8")
        style.configure("TLabelframe.Label", background="#0b1220", foreground="#8fd0e8",
                        font=TITLE_FONT)
        style.configure("TButton", background="#1d2b45", foreground="#e8eefc", padding=4)
        style.map("TButton", background=[("active", "#2a3d61")])
        style.configure("Treeview", background="#131c30", fieldbackground="#131c30",
                        foreground="#e8eefc", rowheight=20)
        style.configure("Treeview.Heading", background="#1d2b45", foreground="#e8eefc")
        style.configure("TNotebook", background="#0b1220")
        style.configure("TNotebook.Tab", background="#16213a", foreground="#c6d3ea")
        style.map("TNotebook.Tab", background=[("selected", "#1d2b45")])
        style.configure("TProgressbar", background="#2f9e8f")

    # ------------------------------------------------------------------
    # toolbar
    # ------------------------------------------------------------------
    def _build_toolbar(self) -> None:
        bar = tk.Frame(self.root, bg="#131c30", height=42)
        bar.pack(side=tk.TOP, fill=tk.X)
        bar.pack_propagate(False)

        self.btn_play = tk.Button(bar, text="Start", command=self._toggle_play,
                                  bg="#2f9e8f", fg="#06121a", font=FONT_BOLD,
                                  relief=tk.FLAT, padx=14, activebackground="#3cb5a3")
        self.btn_play.pack(side=tk.LEFT, padx=(8, 4), pady=7)

        tk.Button(bar, text="Reset", command=self._reset, bg="#1d2b45", fg="#e8eefc",
                  font=FONT, relief=tk.FLAT, padx=12, activebackground="#2a3d61"
                  ).pack(side=tk.LEFT, padx=4, pady=7)
        tk.Button(bar, text="Step", command=self._single_step, bg="#1d2b45", fg="#e8eefc",
                  font=FONT, relief=tk.FLAT, padx=12, activebackground="#2a3d61"
                  ).pack(side=tk.LEFT, padx=4, pady=7)
        tk.Button(bar, text="Route to D1", command=self._show_route, bg="#1d2b45",
                  fg="#e8eefc", font=FONT, relief=tk.FLAT, padx=12,
                  activebackground="#2a3d61").pack(side=tk.LEFT, padx=4, pady=7)

        tk.Label(bar, text="Experiment duration (s)", bg="#131c30", fg="#8fa0bd",
                 font=FONT).pack(side=tk.LEFT, padx=(18, 4))
        self.duration_var = tk.StringVar(value="90")
        tk.Spinbox(bar, from_=20, to=600, increment=10, textvariable=self.duration_var,
                   width=6, bg="#1d2b45", fg="#e8eefc", font=FONT,
                   buttonbackground="#1d2b45", insertbackground="#e8eefc"
                   ).pack(side=tk.LEFT, pady=7)

        tk.Button(bar, text="Run all experiments", command=self._run_experiments,
                  bg="#4a6fd4", fg="#06121a", font=FONT_BOLD, relief=tk.FLAT, padx=12,
                  activebackground="#5b80e8").pack(side=tk.RIGHT, padx=8, pady=7)
        tk.Button(bar, text="Export CSV", command=self._export_csv, bg="#1d2b45",
                  fg="#e8eefc", font=FONT, relief=tk.FLAT, padx=12,
                  activebackground="#2a3d61").pack(side=tk.RIGHT, padx=4, pady=7)

    # ------------------------------------------------------------------
    # body
    # ------------------------------------------------------------------
    def _build_body(self) -> None:
        body = tk.Frame(self.root, bg="#0b1220")
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.left_panel = tk.Frame(body, bg="#0b1220", width=290)
        self.left_panel.pack(side=tk.LEFT, fill=tk.Y)
        self.left_panel.pack_propagate(False)

        self.canvas = SwarmCanvas(body, self.sim)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.right_panel = tk.Frame(body, bg="#0b1220", width=460)
        self.right_panel.pack(side=tk.RIGHT, fill=tk.Y)
        self.right_panel.pack_propagate(False)

        self._build_control_panel()
        self._build_notebook()

    # ------------------------------------------------------------------
    # control panel
    # ------------------------------------------------------------------
    def _build_control_panel(self) -> None:
        canvas_frame = tk.Frame(self.left_panel, bg="#0b1220")
        canvas_frame.pack(fill=tk.BOTH, expand=True)

        # --- swarm configuration -------------------------------------
        swarm = ttk.LabelFrame(canvas_frame, text="Swarm configuration", padding=6)
        swarm.pack(fill=tk.X, padx=6, pady=(6, 3))

        row = tk.Frame(swarm, bg="#0b1220")
        row.pack(fill=tk.X)
        tk.Label(row, text="Number of drones", bg="#0b1220", fg="#e8eefc",
                 font=FONT).pack(side=tk.LEFT)
        self.drone_count_var = tk.IntVar(value=self.config.num_drones)
        self.drone_spin = tk.Spinbox(row, from_=2, to=30, textvariable=self.drone_count_var,
                                     width=5, command=self._apply_drone_count, bg="#1d2b45",
                                     fg="#e8eefc", font=FONT, buttonbackground="#1d2b45",
                                     insertbackground="#e8eefc")
        self.drone_spin.pack(side=tk.RIGHT)
        tk.Label(swarm, text="change applies on reset", bg="#0b1220", fg="#8fa0bd",
                 font=("Segoe UI", 8)).pack(anchor="w")

        self.scale_range = self._add_scale(swarm, "Communication range", 60, 320,
                                           self.config.comm_range, self._on_range, "px")
        self.scale_speed = self._add_scale(swarm, "Drone speed", 10, 100,
                                           self.config.drone_speed, self._on_speed, "px/s")
        self.scale_sim = self._add_scale(swarm, "Simulation speed", 0.2, 4.0,
                                         self.config.sim_speed, self._on_sim_speed, "x")
        self.scale_rate = self._add_scale(swarm, "Message interval", 0.2, 4.0,
                                          self.config.message_interval, self._on_rate, "s")

        # --- network conditions -------------------------------------
        conditions = ttk.LabelFrame(canvas_frame, text="Network conditions", padding=6)
        conditions.pack(fill=tk.X, padx=6, pady=3)
        self.scale_loss = self._add_scale(conditions, "Packet loss", 0, 50,
                                          self.config.packet_loss * 100, self._on_loss, "%")
        self.scale_delay = self._add_scale(conditions, "Link delay", 0, 200,
                                            self.config.delay_ms, self._on_delay, "ms")

        # --- failure simulation -------------------------------------
        failure = ttk.LabelFrame(canvas_frame, text="Failure simulation", padding=6)
        failure.pack(fill=tk.BOTH, expand=True, padx=6, pady=3)

        tk.Label(failure, text="Drones", bg="#0b1220", fg="#8fa0bd",
                 font=("Segoe UI", 8)).pack(anchor="w")
        self.drone_list = tk.Listbox(failure, height=7, bg="#131c30", fg="#e8eefc",
                                     font=FONT, highlightthickness=0, exportselection=False)
        self.drone_list.pack(fill=tk.X)
        self.drone_list.bind("<<ListboxSelect>>", self._on_drone_selected)

        buttons = tk.Frame(failure, bg="#0b1220")
        buttons.pack(fill=tk.X, pady=4)
        tk.Button(buttons, text="Disconnect", command=self._disconnect_drone, bg="#b4523f",
                  fg="#fff", font=FONT, relief=tk.FLAT, padx=6,
                  activebackground="#c85f4b").pack(side=tk.LEFT, fill=tk.X, expand=True,
                                                   padx=(0, 2))
        tk.Button(buttons, text="Reconnect", command=self._reconnect_drone, bg="#2f9e8f",
                  fg="#06121a", font=FONT, relief=tk.FLAT, padx=6,
                  activebackground="#3cb5a3").pack(side=tk.RIGHT, fill=tk.X, expand=True,
                                                   padx=(2, 0))

        tk.Label(failure, text="Links of the selected drone", bg="#0b1220", fg="#8fa0bd",
                 font=("Segoe UI", 8)).pack(anchor="w")
        self.link_list = tk.Listbox(failure, height=6, bg="#131c30", fg="#e8eefc",
                                    font=FONT, highlightthickness=0, exportselection=False)
        self.link_list.pack(fill=tk.X)

        link_buttons = tk.Frame(failure, bg="#0b1220")
        link_buttons.pack(fill=tk.X, pady=4)
        tk.Button(link_buttons, text="Disable link", command=self._disable_link,
                  bg="#b4523f", fg="#fff", font=FONT, relief=tk.FLAT,
                  activebackground="#c85f4b").pack(side=tk.LEFT, fill=tk.X, expand=True,
                                                   padx=(0, 2))
        tk.Button(link_buttons, text="Restore link", command=self._restore_link,
                  bg="#2f9e8f", fg="#06121a", font=FONT, relief=tk.FLAT,
                  activebackground="#3cb5a3").pack(side=tk.RIGHT, fill=tk.X, expand=True,
                                                   padx=(2, 0))

    def _add_scale(self, parent, label: str, low: float, high: float, value: float,
                   command, unit: str) -> tk.Scale:
        frame = tk.Frame(parent, bg="#0b1220")
        frame.pack(fill=tk.X, pady=2)
        tk.Label(frame, text=label, bg="#0b1220", fg="#e8eefc",
                 font=FONT).pack(side=tk.LEFT)
        value_label = tk.Label(frame, text=f"{value:g}{unit}", bg="#0b1220", fg="#8fd0e8",
                               font=FONT_BOLD, width=7, anchor="e")
        value_label.pack(side=tk.RIGHT)
        scale = tk.Scale(frame, from_=low, to=high, orient=tk.HORIZONTAL, resolution=1,
                         showvalue=False, bg="#0b1220", fg="#e8eefc", troughcolor="#1d2b45",
                         activebackground="#2f9e8f", highlightthickness=0, bd=0,
                         command=lambda v: self._on_scale(scale, value_label, command, unit, v))
        scale.set(value)
        scale.pack(fill=tk.X)
        return scale

    def _on_scale(self, scale, label, command, unit, raw_value) -> None:
        value = float(raw_value)
        label.config(text=f"{value:g}{unit}")
        command(value)

    # ------------------------------------------------------------------
    # right hand notebook
    # ------------------------------------------------------------------
    def _build_notebook(self) -> None:
        self.notebook = ttk.Notebook(self.right_panel)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        self.dashboard_tab = tk.Frame(self.notebook, bg="#0b1220")
        self.log_tab = tk.Frame(self.notebook, bg="#0b1220")
        self.experiment_tab = tk.Frame(self.notebook, bg="#0b1220")
        self.chart_tab = tk.Frame(self.notebook, bg="#0b1220")

        self.notebook.add(self.dashboard_tab, text="Dashboard")
        self.notebook.add(self.log_tab, text="Communication log")
        self.notebook.add(self.experiment_tab, text="Experiments")
        self.notebook.add(self.chart_tab, text="Charts")

        self._build_dashboard()
        self._build_log()
        self._build_experiments()
        self._build_charts()

    def _build_dashboard(self) -> None:
        self.metric_labels: Dict[str, tk.Label] = {}

        network = ttk.LabelFrame(self.dashboard_tab, text="Network state", padding=6)
        network.pack(fill=tk.X, padx=6, pady=(6, 3))
        traffic = ttk.LabelFrame(self.dashboard_tab, text="Traffic and performance", padding=6)
        traffic.pack(fill=tk.X, padx=6, pady=3)

        self._add_metrics(network, [
            ("active_drones", "Active drones"),
            ("disconnected_drones", "Disconnected drones"),
            ("active_links", "Active links"),
            ("components", "Connected components"),
            ("isolated_drones", "Isolated drones"),
        ])
        self._add_metrics(traffic, [
            ("messages_sent", "Messages transmitted"),
            ("messages_delivered", "Messages delivered"),
            ("messages_dropped", "Dropped (packet loss)"),
            ("messages_undeliverable", "Undeliverable (no route)"),
            ("pdr_percent", "Packet delivery ratio (%)"),
            ("loss_percent", "Packet loss (%)"),
            ("avg_latency_ms", "Average latency (ms)"),
            ("p95_latency_ms", "95th percentile latency (ms)"),
            ("max_latency_ms", "Maximum latency (ms)"),
            ("throughput_bytes_s", "Throughput (bytes/s)"),
        ])

        detail = ttk.LabelFrame(self.dashboard_tab, text="Selected drone", padding=6)
        detail.pack(fill=tk.X, padx=6, pady=3)
        self.detail_labels: Dict[str, tk.Label] = {}
        for key, caption in (("id", "Drone"), ("address", "UDP endpoint"),
                             ("status", "Status"), ("position", "Position"),
                             ("links", "Neighbours"), ("sent", "Messages sent"),
                             ("received", "Messages received"),
                             ("forwarded", "Messages forwarded")):
            row = tk.Frame(detail, bg="#0b1220")
            row.pack(fill=tk.X, pady=1)
            tk.Label(row, text=caption, bg="#0b1220", fg="#8fa0bd", font=FONT,
                     width=17, anchor="w").pack(side=tk.LEFT)
            value_label = tk.Label(row, text="-", bg="#0b1220", fg="#e8eefc", font=FONT_BOLD,
                                   anchor="w")
            value_label.pack(side=tk.RIGHT)
            self.detail_labels[key] = value_label

        reach = ttk.LabelFrame(self.dashboard_tab, text="Reachability", padding=6)
        reach.pack(fill=tk.X, padx=6, pady=3)
        self.reach_label = tk.Label(reach, text="-", bg="#0b1220", fg="#e8eefc",
                                    font=FONT, anchor="w", justify="left")
        self.reach_label.pack(fill=tk.X)

    def _add_metrics(self, parent, entries) -> None:
        for key, caption in entries:
            row = tk.Frame(parent, bg="#0b1220")
            row.pack(fill=tk.X, pady=1)
            tk.Label(row, text=caption, bg="#0b1220", fg="#8fa0bd", font=FONT,
                     anchor="w").pack(side=tk.LEFT)
            value_label = tk.Label(row, text="-", bg="#0b1220", fg="#e8eefc",
                                   font=FONT_BOLD, anchor="e")
            value_label.pack(side=tk.RIGHT)
            self.metric_labels[key] = value_label

    def _build_log(self) -> None:
        columns = ("time", "source", "destination", "type", "status", "detail")
        self.log_tree = ttk.Treeview(self.log_tab, columns=columns, show="headings",
                                     height=24)
        headings = {"time": "Time (s)", "source": "Source", "destination": "Destination",
                    "type": "Message type", "status": "Status", "detail": "Detail"}
        widths = {"time": 60, "source": 60, "destination": 70, "type": 130,
                  "status": 90, "detail": 110}
        for column in columns:
            self.log_tree.heading(column, text=headings[column])
            self.log_tree.column(column, width=widths[column], anchor="w")
        scroll = ttk.Scrollbar(self.log_tab, orient=tk.VERTICAL, command=self.log_tree.yview)
        self.log_tree.configure(yscrollcommand=scroll.set)
        self.log_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.log_tree.tag_configure("DELIVERED", foreground="#5fd3a8")
        self.log_tree.tag_configure("FORWARDED", foreground="#8fd0e8")
        self.log_tree.tag_configure("DROPPED", foreground="#e58f7a")
        self.log_tree.tag_configure("UNREACHABLE", foreground="#e0b95c")
        self.log_tree.tag_configure("FAILURE", foreground="#e58f7a")
        self.log_tree.tag_configure("RECOVERED", foreground="#5fd3a8")

    def _build_experiments(self) -> None:
        top = tk.Frame(self.experiment_tab, bg="#0b1220")
        top.pack(fill=tk.X, padx=6, pady=6)
        self.experiment_status = tk.Label(top, text="Ready", bg="#0b1220", fg="#8fa0bd",
                                          font=FONT, anchor="w")
        self.experiment_status.pack(side=tk.LEFT)
        tk.Button(top, text="Plot charts", command=self._plot_experiment_charts, bg="#1d2b45",
                  fg="#e8eefc", font=FONT, relief=tk.FLAT,
                  activebackground="#2a3d61").pack(side=tk.RIGHT)

        columns = ("name", "drones", "pdr", "latency", "throughput", "links", "recovery")
        self.experiment_tree = ttk.Treeview(self.experiment_tab, columns=columns,
                                            show="headings", height=16)
        headings = {"name": "Experiment", "drones": "Drones", "pdr": "PDR %",
                    "latency": "Latency ms", "throughput": "B/s", "links": "Links",
                    "recovery": "Recovery s"}
        widths = {"name": 220, "drones": 60, "pdr": 60, "latency": 80, "throughput": 70,
                  "links": 60, "recovery": 80}
        for column in columns:
            self.experiment_tree.heading(column, text=headings[column])
            self.experiment_tree.column(column, width=widths[column], anchor="w")
        scroll = ttk.Scrollbar(self.experiment_tab, orient=tk.VERTICAL,
                               command=self.experiment_tree.yview)
        self.experiment_tree.configure(yscrollcommand=scroll.set)
        self.experiment_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)

    def _build_charts(self) -> None:
        self.chart_figure = Figure(figsize=(4.6, 6.6), dpi=100)
        self.chart_figure.patch.set_facecolor("#0b1220")
        self.chart_axes_pdr = self.chart_figure.add_subplot(211)
        self.chart_axes_lat = self.chart_figure.add_subplot(212)
        for axis in (self.chart_axes_pdr, self.chart_axes_lat):
            axis.set_facecolor("#0b1220")
            axis.tick_params(colors="#8fa0bd", labelsize=7)
            axis.xaxis.label.set_color("#8fa0bd")
            axis.yaxis.label.set_color("#8fa0bd")
            axis.title.set_color("#e8eefc")
            for spine in axis.spines.values():
                spine.set_color("#24304d")
            axis.grid(alpha=0.25, color="#33405e")

        self.chart_axes_pdr.set_title("Packet delivery ratio (%)", fontsize=9)
        self.chart_axes_pdr.set_ylabel("PDR %", fontsize=8)
        self.chart_axes_lat.set_title("Average latency (ms)", fontsize=9)
        self.chart_axes_lat.set_ylabel("latency ms", fontsize=8)

        self.chart_canvas = FigureCanvasTkAgg(self.chart_figure, master=self.chart_tab)
        self.chart_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ------------------------------------------------------------------
    # status bar
    # ------------------------------------------------------------------
    def _build_statusbar(self) -> None:
        bar = tk.Frame(self.root, bg="#131c30", height=24)
        bar.pack(side=tk.BOTTOM, fill=tk.X)
        bar.pack_propagate(False)
        self.status_var = tk.StringVar(value="ready")
        tk.Label(bar, textvariable=self.status_var, bg="#131c30", fg="#8fa0bd",
                 font=("Segoe UI", 8), anchor="w").pack(side=tk.LEFT, padx=8)

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def _toggle_play(self) -> None:
        running = self.sim.toggle()
        self.btn_play.config(text="Pause" if running else "Start")
        self.status_var.set("simulation running" if running else "simulation paused")

    def _single_step(self) -> None:
        self.sim.step(0.05)
        self.canvas.redraw()
        self._refresh_dashboard()
        self._refresh_log()

    def _reset(self) -> None:
        self.sim.running = False
        self.btn_play.config(text="Start")
        self.config.num_drones = int(self.drone_count_var.get())
        self.sim.config = self.config
        self.sim.reset()
        self.canvas.selected = None
        self.canvas.route = []
        self.canvas.redraw()
        self.last_event_index = 0
        self.history_time.clear()
        self.history_pdr.clear()
        self.history_latency.clear()
        self.log_tree.delete(*self.log_tree.get_children())
        self._refresh_dashboard()
        self._refresh_drone_list()
        self.status_var.set("simulation reset")

    def _apply_drone_count(self) -> None:
        self.config.num_drones = int(self.drone_count_var.get())
        self.status_var.set("drone count changed - press Reset to apply")

    def _on_range(self, value: float) -> None:
        self.config.comm_range = value
        for drone in self.sim.drones.values():
            drone.comm_range = value

    def _on_speed(self, value: float) -> None:
        self.config.drone_speed = value
        for drone in self.sim.drones.values():
            drone.speed = value

    def _on_sim_speed(self, value: float) -> None:
        self.config.sim_speed = value

    def _on_rate(self, value: float) -> None:
        self.config.message_interval = value

    def _on_loss(self, value: float) -> None:
        self.config.packet_loss = value / 100.0
        self.sim.failures.packet_loss = self.config.packet_loss

    def _on_delay(self, value: float) -> None:
        self.config.delay_ms = value
        self.sim.failures.delay_ms = value

    # --- selection ---------------------------------------------------
    def on_selection_changed(self, drone_id: Optional[str]) -> None:
        self.selected_drone = drone_id
        self._refresh_drone_list()
        self._refresh_dashboard()

    def _current_drone_id(self) -> Optional[str]:
        """The drone the user selected (canvas click or list box)."""
        return self.canvas.selected or self.selected_drone

    def _refresh_drone_list(self) -> None:
        self.drone_list.delete(0, tk.END)
        for drone_id, drone in self.sim.drones.items():
            suffix = "  [offline]" if drone.status != "ACTIVE" else ""
            self.drone_list.insert(tk.END, f"{drone_id}  ({drone.port}){suffix}")
        selected = self._current_drone_id()
        if selected:
            for index, drone_id in enumerate(self.sim.drones):
                if drone_id == selected:
                    self.drone_list.selection_clear(0, tk.END)
                    self.drone_list.selection_set(index)
                    break
        self._refresh_link_list()

    def _on_drone_selected(self, _event=None) -> None:
        selection = self.drone_list.curselection()
        if not selection:
            return
        index = selection[0]
        if index < len(self.sim.drones):
            drone_id = list(self.sim.drones)[index]
            self.selected_drone = drone_id
            self.canvas.selected = drone_id
            self.canvas.redraw()
            self._refresh_link_list()
            self._refresh_dashboard()

    def _refresh_link_list(self) -> None:
        self.link_list.delete(0, tk.END)
        selected = self._current_drone_id()
        if not selected:
            return
        for neighbour in sorted(self.sim.topology.neighbours(selected)):
            state = " (down)" if self.sim.failures.link_disabled(selected, neighbour) else ""
            self.link_list.insert(tk.END, f"{selected} <-> {neighbour}{state}")

    def _selected_link(self) -> Optional[tuple]:
        selection = self.link_list.curselection()
        if not selection:
            return None
        text = self.link_list.get(selection[0])
        other = text.split("<->")[1].strip().split(" ")[0]
        return (self._current_drone_id(), other)

    # --- failures ----------------------------------------------------
    def _disconnect_drone(self) -> None:
        selected = self._current_drone_id()
        if not selected:
            return
        if self.sim.disconnect_drone(selected):
            self.status_var.set(f"{selected} disconnected")
        self.canvas.redraw()
        self._refresh_drone_list()
        self._refresh_dashboard()

    def _reconnect_drone(self) -> None:
        selected = self._current_drone_id()
        if not selected:
            return
        if self.sim.reconnect_drone(selected):
            self.status_var.set(f"{selected} reconnected")
        self.canvas.redraw()
        self._refresh_drone_list()
        self._refresh_dashboard()

    def _disable_link(self) -> None:
        link = self._selected_link()
        if not link:
            self.status_var.set("select a link in the list first")
            return
        self.sim.disable_link(link[0], link[1])
        self.status_var.set(f"link {link[0]} <-> {link[1]} disabled")
        self.canvas.redraw()
        self._refresh_link_list()
        self._refresh_dashboard()

    def _restore_link(self) -> None:
        link = self._selected_link()
        if not link:
            self.status_var.set("select a link in the list first")
            return
        self.sim.restore_link(link[0], link[1])
        self.status_var.set(f"link {link[0]} <-> {link[1]} restored")
        self.canvas.redraw()
        self._refresh_link_list()
        self._refresh_dashboard()

    def _show_route(self) -> None:
        source = self._current_drone_id()
        if not source:
            return
        destination = "D1" if "D1" in self.sim.drones and "D1" != source else None
        if destination is None:
            destination = next((d for d in self.sim.drones if d != source), None)
        if destination is None:
            return
        route = self.sim.topology.find_route(source, destination)
        if route:
            self.canvas.show_route(route)
            self.status_var.set(f"route: " + " -> ".join(route) +
                                 f"   ({len(route) - 1} hops)")
        else:
            self.canvas.show_route([])
            self.status_var.set(f"no route between {source} and {destination}")

    # ------------------------------------------------------------------
    # experiments
    # ------------------------------------------------------------------
    def _run_experiments(self) -> None:
        if self.experiment_running:
            return
        try:
            duration = max(20, int(self.duration_var.get()))
        except ValueError:
            duration = 90
        self.experiment_running = True
        self.experiment_status.config(text="running experiments...")
        self.notebook.select(self.experiment_tab)

        def worker() -> None:
            try:
                # use a separate port range so that the experiment does not
                # compete with the drones of the running dashboard
                base_port = self.config.base_port + 2000
                results = run_all_experiments(
                    duration=duration,
                    outdir="results",
                    base_port=base_port,
                    progress=lambda name, fraction: self.worker_queue.put(("progress", name, fraction)),
                )
                self.worker_queue.put(("done", results, None))
            except Exception as error:  # pragma: no cover - defensive
                self.worker_queue.put(("error", None, error))

        threading.Thread(target=worker, daemon=True).start()

    def _poll_worker(self) -> None:
        while True:
            try:
                item = self.worker_queue.get_nowait()
            except queue.Empty:
                break
            kind = item[0]
            if kind == "progress":
                _, name, fraction = item
                self.experiment_status.config(text=f"{name}  ({fraction * 100:.0f}%)")
            elif kind == "done":
                _, results, _ = item
                self.experiment_running = False
                self.experiment_results = results
                self.experiment_status.config(text=f"finished - {len(results)} experiments")
                self._fill_experiment_tree()
                self._plot_experiment_charts()
            elif kind == "error":
                _, _, error = item
                self.experiment_running = False
                self.experiment_status.config(text=f"failed: {error}")

    def _fill_experiment_tree(self) -> None:
        self.experiment_tree.delete(*self.experiment_tree.get_children())
        for result in self.experiment_results:
            self.experiment_tree.insert("", tk.END, values=(
                result.name,
                result.config.get("drones", "-"),
                result.metrics.get("pdr_percent", "-"),
                result.metrics.get("avg_latency_ms", "-"),
                result.metrics.get("throughput_bytes_s", "-"),
                result.metrics.get("active_links", "-"),
                result.metrics.get("recovery_time_s", 0),
            ))

    def _plot_experiment_charts(self) -> None:
        if not self.experiment_results:
            self.status_var.set("run the experiments first")
            return
        try:
            paths = plot_results(self.experiment_results, outdir="results")
            self.status_var.set("charts written to " + ", ".join(paths))
        except Exception as error:  # pragma: no cover - defensive
            self.status_var.set(f"chart error: {error}")

    def _export_csv(self) -> None:
        if not self.experiment_results:
            self.status_var.set("run the experiments before exporting")
            return
        path = filedialog.asksaveasfilename(
            title="Export experiment results",
            defaultextension=".csv",
            initialfile="drone_swarm_results.csv",
            filetypes=[("CSV file", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        export_csv(self.experiment_results, path)
        self.status_var.set(f"results exported to {path}")

    # ------------------------------------------------------------------
    # main loop
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        now = time.perf_counter()
        frame = getattr(self, "_last_frame", now)
        self._last_frame = now
        delta = min(0.25, now - frame)

        if self.sim.running:
            self.sim.step(delta * self.config.sim_speed)
            self.canvas.redraw()
            self._refresh_dashboard()
            self._refresh_log()
            self._record_history()

        self._poll_worker()
        self.root.after(30, self._tick)

    def _record_history(self) -> None:
        if self.sim.time - (self.history_time[-1] if self.history_time else 0.0) < 1.0:
            return
        snapshot = self.sim.snapshot()
        self.history_time.append(snapshot["sim_time"])
        self.history_pdr.append(snapshot["pdr_percent"])
        self.history_latency.append(snapshot["avg_latency_ms"])
        self._update_charts()

    def _update_charts(self) -> None:
        self.chart_axes_pdr.clear()
        self.chart_axes_lat.clear()
        self.chart_axes_pdr.plot(self.history_time, self.history_pdr, color="#2f9e8f",
                                 linewidth=1.6)
        self.chart_axes_pdr.set_title("Packet delivery ratio (%)", fontsize=9, color="#e8eefc")
        self.chart_axes_pdr.set_ylim(0, 105)
        self.chart_axes_pdr.grid(alpha=0.25, color="#33405e")
        self.chart_axes_lat.plot(self.history_time, self.history_latency, color="#e08b3c",
                                 linewidth=1.6)
        self.chart_axes_lat.set_title("Average latency (ms)", fontsize=9, color="#e8eefc")
        self.chart_axes_lat.grid(alpha=0.25, color="#33405e")
        for axis in (self.chart_axes_pdr, self.chart_axes_lat):
            axis.tick_params(colors="#8fa0bd", labelsize=7)
            for spine in axis.spines.values():
                spine.set_color("#24304d")
        self.chart_canvas.draw_idle()

    # ------------------------------------------------------------------
    # refresh helpers
    # ------------------------------------------------------------------
    def _refresh_dashboard(self) -> None:
        snapshot = self.sim.snapshot()
        for key, label in self.metric_labels.items():
            value = snapshot.get(key, "-")
            label.config(text=f"{value:.2f}" if isinstance(value, float) else str(value))

        selected = self._current_drone_id()
        drone = self.sim.drones.get(selected) if selected else None
        if drone is None:
            for label in self.detail_labels.values():
                label.config(text="-")
            self.reach_label.config(text="select a drone in the canvas")
            return

        self.detail_labels["id"].config(text=drone.drone_id)
        self.detail_labels["address"].config(text=f"{self.config.host}:{drone.port}")
        self.detail_labels["status"].config(text=drone.status)
        self.detail_labels["position"].config(text=f"({drone.x:.0f}, {drone.y:.0f})")
        self.detail_labels["links"].config(text=str(len(self.sim.topology.neighbours(drone.drone_id))))
        self.detail_labels["sent"].config(text=str(drone.messages_sent))
        self.detail_labels["received"].config(text=str(drone.messages_received))
        self.detail_labels["forwarded"].config(text=str(drone.messages_forwarded))

        lines = []
        for other in list(self.sim.drones)[:6]:
            route = self.sim.topology.find_route(drone.drone_id, other)
            if route is None:
                lines.append(f"{drone.drone_id} -> {other}: unreachable")
            else:
                lines.append(f"{drone.drone_id} -> {other}: " + " -> ".join(route))
        self.reach_label.config(text="\n".join(lines))

    def _refresh_log(self) -> None:
        new_events = [event for event in self.sim.events
                      if event["index"] >= self.last_event_index]
        for event in new_events:
            self.last_event_index = event["index"] + 1
            self.log_tree.insert("", tk.END, values=(
                f"{event['time']:.2f}",
                event["source"],
                event["destination"],
                event["type"],
                event["status"],
                event["detail"],
            ), tags=(event["status"],))
        if new_events:
            children = self.log_tree.get_children()
            if children:
                self.log_tree.see(children[-1])


def main() -> None:
    root = tk.Tk()
    DroneSwarmApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
