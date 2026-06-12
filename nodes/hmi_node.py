#!/usr/bin/env python3
"""
ROS 2 node — IHM tkinter MARBLE Sensors Monitor.

Souscrit à :
  /aanderaa/data   (std_msgs/String, JSON)
  /aquadopp/data   (std_msgs/String, JSON)
  /sbe37/data      (std_msgs/String, JSON)

Affiche une fenêtre avec trois panneaux côte à côte.
rclpy.spin() tourne dans un thread séparé ; tkinter dans le thread principal.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import tkinter as tk
from tkinter import ttk
import threading
import json
import os
import signal
import subprocess
import time
import sys

from serial.tools import list_ports

# ─── Palette ─────────────────────────────────────────────────────────────────

BG           = "#1e1e1e"
PANEL_BG     = "#252526"
HEADER_BG    = "#2d2d30"
BORDER       = "#3e3e42"
TEXT_DIM     = "#858585"
TEXT_NORM    = "#cccccc"
TEXT_VAL     = "#ffffff"
TEXT_SECTION = "#4ec9b0"   # cyan-vert (style VS Code)
COL_GREEN    = "#4ec9b0"
COL_RED      = "#f44747"
COL_YELLOW   = "#dcdcaa"

FONT_TITLE   = ("Consolas", 12, "bold")
FONT_SECTION = ("Consolas", 9, "bold")
FONT_LABEL   = ("Consolas", 9)
FONT_VALUE   = ("Consolas", 9, "bold")
FONT_CLOCK   = ("Consolas", 9)

# ─── SensorPanel ─────────────────────────────────────────────────────────────

class SensorPanel(tk.Frame):
    """
    Panneau d'affichage pour un capteur.

    groups : list of (titre_section, [(clé_field, label_affiché, unité_défaut), ...])
    on_connect : callback (port, baud) — si fourni, affiche une barre de
                 sélection du port qui demande au node de se reconnecter.
    """

    def __init__(self, parent, title: str, groups: list,
                 on_connect=None, default_baud: int = 115200,
                 on_stop=None, **kw):
        super().__init__(parent, bg=PANEL_BG,
                         highlightbackground=BORDER, highlightthickness=1, **kw)
        self._rows = {}  # clé → tk.StringVar(valeur)
        self._on_connect = on_connect

        # ── En-tête ──────────────────────────────────────────────────────────
        hdr = tk.Frame(self, bg=HEADER_BG)
        hdr.pack(fill=tk.X)

        self._dot = tk.Label(hdr, text="●", bg=HEADER_BG,
                             font=("Consolas", 11), fg=COL_RED)
        self._dot.pack(side=tk.LEFT, padx=(10, 4), pady=7)

        tk.Label(hdr, text=title, bg=HEADER_BG,
                 font=FONT_TITLE, fg=TEXT_VAL).pack(side=tk.LEFT, pady=7)

        self._ts_var = tk.StringVar(value="--:--:--")
        tk.Label(hdr, textvariable=self._ts_var, bg=HEADER_BG,
                 font=FONT_CLOCK, fg=TEXT_DIM).pack(side=tk.RIGHT, padx=10, pady=7)

        # ── Barre de connexion (port / baud) ──────────────────────────────────
        if on_connect is not None:
            bar = tk.Frame(self, bg=HEADER_BG)
            bar.pack(fill=tk.X)

            tk.Label(bar, text="Port :", bg=HEADER_BG, font=FONT_LABEL,
                     fg=TEXT_DIM).pack(side=tk.LEFT, padx=(10, 3), pady=4)
            self._port_var = tk.StringVar()
            self._port_combo = ttk.Combobox(bar, textvariable=self._port_var,
                                            width=13, font=FONT_LABEL)
            self._port_combo.pack(side=tk.LEFT, pady=4)

            tk.Label(bar, text="Baud :", bg=HEADER_BG, font=FONT_LABEL,
                     fg=TEXT_DIM).pack(side=tk.LEFT, padx=(8, 3), pady=4)
            self._baud_var = tk.StringVar(value=str(default_baud))
            ttk.Combobox(bar, textvariable=self._baud_var, width=7,
                         values=('1200', '2400', '4800', '9600', '19200',
                                 '38400', '57600', '115200', '230400'),
                         font=FONT_LABEL).pack(side=tk.LEFT, pady=4)

            tk.Button(bar, text="↻", command=self._refresh_ports,
                      bg=PANEL_BG, fg=TEXT_VAL, font=FONT_LABEL,
                      activebackground=BORDER, activeforeground=TEXT_VAL,
                      relief=tk.FLAT, padx=6).pack(side=tk.LEFT, padx=(8, 0), pady=4)
            tk.Button(bar, text="Connecter", command=self._do_connect,
                      bg=COL_GREEN, fg=BG, font=FONT_LABEL,
                      activebackground=TEXT_SECTION, activeforeground=BG,
                      relief=tk.FLAT, padx=8).pack(side=tk.LEFT, padx=(6, 4), pady=4)

            if on_stop is not None:
                tk.Button(bar, text="■", command=on_stop,
                          bg=PANEL_BG, fg=COL_RED, font=FONT_LABEL,
                          activebackground=BORDER, activeforeground=COL_RED,
                          relief=tk.FLAT, padx=6).pack(side=tk.LEFT, padx=(0, 10), pady=4)

            self._refresh_ports()

        # ── Corps (scrollable) ────────────────────────────────────────────────
        outer = tk.Frame(self, bg=PANEL_BG)
        outer.pack(fill=tk.BOTH, expand=True)

        canvas = tk.Canvas(outer, bg=PANEL_BG, highlightthickness=0, bd=0)
        vsb = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)

        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        inner = tk.Frame(canvas, bg=PANEL_BG)
        win_id = canvas.create_window((0, 0), window=inner, anchor="nw")

        def _on_inner_configure(event):
            canvas.configure(scrollregion=canvas.bbox("all"))
        def _on_canvas_configure(event):
            canvas.itemconfig(win_id, width=event.width)
        def _on_wheel(event):
            canvas.yview_scroll(-1 * (event.delta // 120), "units")

        inner.bind("<Configure>", _on_inner_configure)
        canvas.bind("<Configure>", _on_canvas_configure)
        canvas.bind("<MouseWheel>", _on_wheel)

        # ── Champs par groupe ─────────────────────────────────────────────────
        for group_title, fields in groups:
            tk.Label(inner, text=f"  ── {group_title}",
                     bg=PANEL_BG, font=FONT_SECTION,
                     fg=TEXT_SECTION, anchor=tk.W
                     ).pack(fill=tk.X, pady=(10, 3), padx=6)

            for key, label, default_unit in fields:
                row = tk.Frame(inner, bg=PANEL_BG)
                row.pack(fill=tk.X, padx=10, pady=1)

                tk.Label(row, text=f"{label}:", bg=PANEL_BG,
                         font=FONT_LABEL, fg=TEXT_DIM,
                         width=30, anchor=tk.W).pack(side=tk.LEFT)

                val_var = tk.StringVar(value="---")
                tk.Label(row, textvariable=val_var, bg=PANEL_BG,
                         font=FONT_VALUE, fg=TEXT_VAL,
                         width=18, anchor=tk.E).pack(side=tk.LEFT)

                self._rows[key] = (val_var, default_unit)

        # ── Barre de statut ───────────────────────────────────────────────────
        stat = tk.Frame(self, bg=HEADER_BG)
        stat.pack(fill=tk.X, side=tk.BOTTOM)
        self._status_var = tk.StringVar(value="En attente du capteur...")
        tk.Label(stat, textvariable=self._status_var,
                 bg=HEADER_BG, font=FONT_CLOCK, fg=TEXT_DIM
                 ).pack(side=tk.LEFT, padx=10, pady=3)

    # ── Sélection du port ─────────────────────────────────────────────────────

    def _refresh_ports(self) -> None:
        """Re-scanne les ports série disponibles sur la machine."""
        ports = [p.device for p in list_ports.comports()]
        self._port_combo['values'] = ports
        if ports and not self._port_var.get():
            self._port_var.set(ports[0])

    def _do_connect(self) -> None:
        port = self._port_var.get().strip()
        if not port:
            self._status_var.set("⚠ Choisir un port d'abord")
            return
        try:
            baud = int(self._baud_var.get())
        except ValueError:
            self._status_var.set(f"⚠ Baudrate invalide : {self._baud_var.get()}")
            return
        self._on_connect(port, baud)
        self._dot.config(fg=COL_YELLOW)

    def set_status(self, text: str) -> None:
        """Affiche un message dans la barre de statut du panneau."""
        self._status_var.set(text)

    # ── Mise à jour ───────────────────────────────────────────────────────────

    def update_data(self, data: dict) -> None:
        status = data.get('status', 'unknown')

        if status == 'ok':
            self._dot.config(fg=COL_GREEN)
            self._ts_var.set(data.get('timestamp', '--:--:--'))
            self._status_var.set("Connecté  ●  données reçues")
            fields = data.get('fields', {})
            for key, (val_var, default_unit) in self._rows.items():
                if key in fields:
                    entry   = fields[key]
                    display = entry.get('display', '---')
                    unit    = entry.get('unit', default_unit)
                    val_var.set(f"{display} {unit}".strip() if unit else display)

        elif status == 'error':
            self._dot.config(fg=COL_RED)
            err = data.get('error', '?')
            if 'could not open port' in err.lower():
                self._status_var.set("⚠ Veuillez connecter le capteur")
            else:
                self._status_var.set(f"Erreur : {err[:55]}")

        elif status == 'no_data':
            self._dot.config(fg=COL_YELLOW)
            self._status_var.set("Connecté — aucune donnée reçue")

        else:
            self._dot.config(fg=COL_YELLOW)
            self._status_var.set("En attente du capteur...")


# ─── Application principale ───────────────────────────────────────────────────

class HMIApp:

    # Définition des groupes AANDERAA
    AANDERAA_GROUPS = [
        ("Attitude", [
            ("Pitch",           "Pitch",                    "°"),
            ("Roll",            "Roll",                     "°"),
            ("Heading",         "Heading (Cap)",            "°"),
            ("StDev Pitch",     "Pitch  — écart-type",      "°"),
            ("StDev Roll",      "Roll   — écart-type",      "°"),
            ("StDev Heading",   "Cap    — écart-type",      "°"),
        ]),
        ("Hauteur des vagues", [
            ("Significant Wave Height Hm0", "Hm0  (significatif)", "m"),
            ("Wave Height Wind Hm0",        "Hm0  (vent)",         "m"),
            ("Wave Height Swell Hm0",       "Hm0  (houle)",        "m"),
            ("Wave Height H1/3",            "H1/3",                "m"),
            ("Wave Height Hmax",            "Hmax",                "m"),
        ]),
        ("Périodes et directions", [
            ("Wave Mean Period Tz",         "Période moyenne Tz",     "s"),
            ("Wave Mean Period Tm02",       "Période moyenne Tm02",   "s"),
            ("Wave Peak Period Wind",       "Période pic  (vent)",    "s"),
            ("Wave Peak Period Swell",      "Période pic  (houle)",   "s"),
            ("Wave Peak Direction",         "Direction pic",           "°"),
            ("Wave Peak Direction Wind",    "Direction pic  (vent)",   "°"),
            ("Wave Peak Direction Swell",   "Direction pic  (houle)",  "°"),
            ("Wave Mean Direction",         "Direction moyenne",        "°"),
            ("Mean Spreading Angle",        "Angle de spreading",       "°"),
        ]),
        ("Système", [
            ("Input Voltage",  "Tension d'alimentation", "V"),
            ("Input Current",  "Courant d'alimentation", "mA"),
            ("Memory Used",    "Mémoire utilisée",       "Bytes"),
        ]),
    ]

    # Définition des groupes Aquadopp
    AQUADOPP_GROUPS = [
        ("Environnement", [
            ("speed_of_sound_ms", "Vitesse du son", "m/s"),
            ("temperature_c",     "Température",    "°C"),
            ("pressure_dbar",     "Pression",       "dbar"),
        ]),
        ("Orientation", [
            ("heading_deg", "Cap (Heading)", "°"),
            ("pitch_deg",   "Pitch",         "°"),
            ("roll_deg",    "Roll",          "°"),
        ]),
    ]

    # Définition des groupes SBE 37-SIP
    SBE37_GROUPS = [
        ("CTD", [
            ("temperature_c",   "Température",   "°C"),
            ("conductivity_sm", "Conductivité",  "S/m"),
            ("pressure_dbar",   "Pression",      "dbar"),
        ]),
        ("Dérivées", [
            ("salinity_psu",      "Salinité",        "PSU"),
            ("sound_velocity_ms", "Vitesse du son",  "m/s"),
            ("depth_m",           "Profondeur",      "m"),
        ]),
    ]

    # Définition des groupes RBRcoda3
    RBR_GROUPS = [
        ("Mesures", [
            ("temperature_c",     "Température",       "°C"),
            ("pressure_dbar",     "Pression absolue",  "dbar"),
        ]),
        ("Dérivées", [
            ("sea_pressure_dbar", "Pression marine",   "dbar"),
            ("depth_m",           "Profondeur",        "m"),
        ]),
        ("Capteur", [
            ("model",     "Modèle",          ""),
            ("serial",    "N° série",        ""),
            ("firmware",  "Firmware",        ""),
            ("mode",      "Mode",            ""),
            ("period_ms", "Période",         "ms"),
        ]),
        ("Flux", [
            ("sample_time",  "Horodatage capteur",  ""),
            ("sample_count", "Échantillons reçus",  ""),
        ]),
    ]

    # Exécutables ros2 de chaque capteur (lancés au clic sur Connecter)
    _EXES = {
        'aanderaa': 'aanderaa_node',
        'aquadopp': 'aquadopp_node',
        'sbe37':    'sbe37_node',
        'rbrcoda3': 'rbrcoda3_node',
    }

    def __init__(self, node: 'SensorsHMINode'):
        self._node = node
        self._procs = {}   # sensor → subprocess.Popen des nodes lancés par l'IHM

        self._root = tk.Tk()
        self._root.title("MARBLE — Sensors Monitor")
        self._root.configure(bg=BG)
        self._root.geometry("2100x800")
        self._root.resizable(True, True)

        # ── Titre ─────────────────────────────────────────────────────────────
        top_bar = tk.Frame(self._root, bg=HEADER_BG)
        top_bar.pack(fill=tk.X)
        tk.Label(top_bar, text="MARBLE  ──  Sensors Monitor",
                 bg=HEADER_BG, font=("Consolas", 14, "bold"),
                 fg=TEXT_VAL, pady=10).pack(side=tk.LEFT, padx=14)

        # ── Colonnes ──────────────────────────────────────────────────────────
        cols = tk.Frame(self._root, bg=BG)
        cols.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)
        cols.columnconfigure(0, weight=3)  # AANDERAA (plus de champs)
        cols.columnconfigure(1, weight=2)  # Aquadopp
        cols.columnconfigure(2, weight=2)  # SBE 37-SIP
        cols.columnconfigure(3, weight=1)  # RBRcoda3
        cols.rowconfigure(0, weight=1)

        self._a_panel = SensorPanel(
            cols, "AANDERAA Motus Wave Sensor 5729", self.AANDERAA_GROUPS,
            on_connect=lambda p, b: self._connect('aanderaa', p, b),
            on_stop=lambda: self._stop('aanderaa'),
            default_baud=115200)
        self._a_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 5))

        self._q_panel = SensorPanel(
            cols, "Aquadopp S4VP", self.AQUADOPP_GROUPS,
            on_connect=lambda p, b: self._connect('aquadopp', p, b),
            on_stop=lambda: self._stop('aquadopp'),
            default_baud=115200)
        self._q_panel.grid(row=0, column=1, sticky="nsew", padx=(5, 5))

        self._s_panel = SensorPanel(
            cols, "SBE 37-SIP MicroCAT", self.SBE37_GROUPS,
            on_connect=lambda p, b: self._connect('sbe37', p, b),
            on_stop=lambda: self._stop('sbe37'),
            default_baud=9600)
        self._s_panel.grid(row=0, column=2, sticky="nsew", padx=(5, 5))

        self._r_panel = SensorPanel(
            cols, "RBRcoda3", self.RBR_GROUPS,
            on_connect=lambda p, b: self._connect('rbrcoda3', p, b),
            on_stop=lambda: self._stop('rbrcoda3'),
            default_baud=9600)
        self._r_panel.grid(row=0, column=3, sticky="nsew", padx=(5, 0))

        # ── Barre du bas ──────────────────────────────────────────────────────
        bot = tk.Frame(self._root, bg=HEADER_BG, pady=3)
        bot.pack(fill=tk.X, side=tk.BOTTOM)
        tk.Label(bot, text="ROS 2  |  marble_sensors_hmi",
                 bg=HEADER_BG, font=FONT_CLOCK, fg=TEXT_DIM
                 ).pack(side=tk.LEFT, padx=12)
        self._clock_var = tk.StringVar()
        tk.Label(bot, textvariable=self._clock_var,
                 bg=HEADER_BG, font=FONT_CLOCK, fg=TEXT_DIM
                 ).pack(side=tk.RIGHT, padx=12)

        self._root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._root.after(500, self._tick)

    # ── Lancement / arrêt des nodes capteurs ──────────────────────────────────

    def _panel_of(self, sensor: str) -> SensorPanel:
        return {'aanderaa': self._a_panel, 'aquadopp': self._q_panel,
                'sbe37': self._s_panel, 'rbrcoda3': self._r_panel}[sensor]

    def _connect(self, sensor: str, port: str, baud: int) -> None:
        """Clic sur Connecter : lance le node s'il ne tourne pas, sinon
        lui demande juste de changer de port à chaud."""
        proc = self._procs.get(sensor)
        ihm_proc_alive = proc is not None and proc.poll() is None

        if ihm_proc_alive or self._node.last_msg_age(sensor) < 10.0:
            # Node déjà actif (lancé par l'IHM ou par un launch externe)
            self._node.send_set_port(sensor, port, baud)
            self._panel_of(sensor).set_status(f"Changement de port → {port} @ {baud}...")
            return

        cmd = ['ros2', 'run', 'marble_sensors_hmi', self._EXES[sensor],
               '--ros-args', '-p', f'port:={port}', '-p', f'baud:={baud}']
        kwargs = {'start_new_session': True} if os.name == 'posix' else {}
        try:
            self._procs[sensor] = subprocess.Popen(cmd, **kwargs)
        except FileNotFoundError:
            self._panel_of(sensor).set_status("⚠ commande 'ros2' introuvable")
            return
        self._node.get_logger().info(f"Node {sensor} lancé — {port} @ {baud}")
        self._panel_of(sensor).set_status(f"Node lancé — connexion à {port} @ {baud}...")

    def _stop(self, sensor: str) -> None:
        """Clic sur ■ : arrête le node lancé par l'IHM."""
        proc = self._procs.pop(sensor, None)
        if proc is None or proc.poll() is not None:
            self._panel_of(sensor).set_status("Aucun node lancé par l'IHM")
            return
        try:
            if os.name == 'posix':
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)   # arrêt propre rclpy
            else:
                proc.terminate()
        except (ProcessLookupError, PermissionError):
            pass
        self._node.get_logger().info(f"Node {sensor} arrêté")
        self._panel_of(sensor).set_status("Node arrêté")

    # ── Rafraîchissement (500 ms) ─────────────────────────────────────────────

    def _tick(self) -> None:
        self._clock_var.set(time.strftime("%Y-%m-%d  %H:%M:%S"))
        a_data, q_data, s_data, r_data = self._node.get_data()
        if a_data:
            self._a_panel.update_data(a_data)
        if q_data:
            self._q_panel.update_data(q_data)
        if s_data:
            self._s_panel.update_data(s_data)
        if r_data:
            self._r_panel.update_data(r_data)
        self._root.after(500, self._tick)

    def _on_close(self) -> None:
        self._node.get_logger().info("Fermeture de l'IHM — arrêt des nodes lancés")
        for sensor, proc in list(self._procs.items()):
            if proc.poll() is None:
                try:
                    if os.name == 'posix':
                        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    else:
                        proc.terminate()
                except (ProcessLookupError, PermissionError):
                    pass
        self._root.destroy()

    def run(self) -> None:
        self._root.mainloop()


# ─── Nœud ROS 2 ──────────────────────────────────────────────────────────────

class SensorsHMINode(Node):

    def __init__(self):
        super().__init__('sensors_hmi')
        self._lock      = threading.Lock()
        self._aanderaa  = None
        self._aquadopp  = None
        self._sbe37     = None
        self._rbrcoda3  = None
        self._last_rx   = {}   # sensor → instant du dernier message reçu

        self.create_subscription(String, 'aanderaa/data',  self._cb_aanderaa,  10)
        self.create_subscription(String, 'aquadopp/data',  self._cb_aquadopp,  10)
        self.create_subscription(String, 'sbe37/data',     self._cb_sbe37,     10)
        self.create_subscription(String, 'rbrcoda3/data',  self._cb_rbrcoda3,  10)

        # Publishers pour demander aux nodes de changer de port à chaud
        self._port_pubs = {
            'aanderaa': self.create_publisher(String, 'aanderaa/set_port', 10),
            'aquadopp': self.create_publisher(String, 'aquadopp/set_port', 10),
            'sbe37':    self.create_publisher(String, 'sbe37/set_port',    10),
            'rbrcoda3': self.create_publisher(String, 'rbrcoda3/set_port', 10),
        }

        self.get_logger().info("HMI node démarré — en attente des données capteurs")

    def send_set_port(self, sensor: str, port: str, baud: int) -> None:
        msg = String()
        msg.data = json.dumps({'port': port, 'baud': baud})
        self._port_pubs[sensor].publish(msg)
        self.get_logger().info(f"set_port {sensor} → {port} @ {baud}")

    def last_msg_age(self, sensor: str) -> float:
        """Secondes depuis le dernier message du capteur (inf si jamais reçu)."""
        t = self._last_rx.get(sensor)
        return time.time() - t if t else float('inf')

    def _cb_aanderaa(self, msg: String) -> None:
        self._last_rx['aanderaa'] = time.time()
        try:
            with self._lock:
                self._aanderaa = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON AANDERAA invalide : {e}")

    def _cb_aquadopp(self, msg: String) -> None:
        self._last_rx['aquadopp'] = time.time()
        try:
            with self._lock:
                self._aquadopp = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON Aquadopp invalide : {e}")

    def _cb_sbe37(self, msg: String) -> None:
        self._last_rx['sbe37'] = time.time()
        try:
            with self._lock:
                self._sbe37 = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON SBE37 invalide : {e}")

    def _cb_rbrcoda3(self, msg: String) -> None:
        self._last_rx['rbrcoda3'] = time.time()
        try:
            with self._lock:
                self._rbrcoda3 = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON RBRcoda3 invalide : {e}")

    def get_data(self):
        with self._lock:
            return (
                dict(self._aanderaa)  if self._aanderaa  else {},
                dict(self._aquadopp)  if self._aquadopp  else {},
                dict(self._sbe37)     if self._sbe37     else {},
                dict(self._rbrcoda3)  if self._rbrcoda3  else {},
            )


# ─── Entry point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SensorsHMINode()

    # rclpy.spin() dans un thread séparé (tkinter doit rester dans le thread principal)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Fenêtre tkinter dans le thread principal
    app = HMIApp(node)
    app.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
