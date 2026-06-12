#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import time
import json
import threading

from marble_sensors_hmi.drivers.aanderaa_driver import (
    read_all, sst, parse_do_output, fmt, SCALAR_FIELDS
)


class AanderaaNode(Node):

    def __init__(self):
        super().__init__('aanderaa_node')

        self.declare_parameter('port',            '/dev/ttyUSB1')
        self.declare_parameter('baud',            115200)
        self.declare_parameter('passkey',         '1')
        self.declare_parameter('sample_interval', 10)

        self._port     = self.get_parameter('port').value
        self._baud     = self.get_parameter('baud').value
        self._passkey  = self.get_parameter('passkey').value
        self._interval = self.get_parameter('sample_interval').value

        self._pub = self.create_publisher(String, 'aanderaa/data', 10)
        self.get_logger().info(f"AANDERAA node démarré — port={self._port}  baud={self._baud}")

        # Changement de port à chaud depuis l'IHM
        self._reconnect = False
        self.create_subscription(String, 'aanderaa/set_port', self._cb_set_port, 10)

        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _cb_set_port(self, msg: String) -> None:
        try:
            cfg = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"set_port JSON invalide : {e}")
            return
        self._port = cfg.get('port', self._port)
        try:
            self._baud = int(cfg.get('baud', self._baud))
        except (TypeError, ValueError):
            pass
        self._reconnect = True
        self.get_logger().info(f"Changement de port demandé → {self._port} @ {self._baud}")

    def _sleep(self, seconds: float) -> None:
        """Attente interruptible par arrêt du node ou changement de port."""
        for _ in range(int(seconds / 0.1)):
            if not self._running or self._reconnect:
                return
            time.sleep(0.1)

    def _publish(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)

    def _loop(self) -> None:
        while self._running:
            try:
                self._reconnect = False
                conn = serial.Serial(
                    port=self._port, baudrate=self._baud,
                    bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    xonxoff=True, rtscts=False, dsrdtr=False,
                    timeout=0.05,
                )
                self.get_logger().info("Port série ouvert — wake-up...")
                conn.reset_input_buffer()
                conn.write(b'\r')
                resp = read_all(conn, 2.0).decode('ascii', errors='replace')
                if '!' in resp:
                    self.get_logger().info("SST wake-up confirmé")

                r = sst(conn, f'Set Passkey({self._passkey})')
                if '#' not in r:
                    self.get_logger().warn(f"Passkey réponse inattendue : {r.strip()[:40]}")

                while self._running and not self._reconnect:
                    sst(conn, 'Do Sample', wait=6.0)
                    time.sleep(0.3)
                    r_out    = sst(conn, 'Do Output', wait=6.0)
                    raw_data = parse_do_output(r_out)

                    if raw_data:
                        fields = {}
                        for field, (raw_val, unit) in raw_data.items():
                            if field not in SCALAR_FIELDS:
                                continue
                            try:
                                fval = float(raw_val)
                            except ValueError:
                                continue
                            fields[field] = {'value': fval, 'unit': unit, 'display': fmt(raw_val, unit)}
                        self._publish({'status': 'ok', 'timestamp': time.strftime('%H:%M:%S'), 'fields': fields})
                        self.get_logger().info(f"Publié {len(fields)} champs")
                    else:
                        self._publish({'status': 'no_data'})
                        self.get_logger().warn("Aucune donnée parsée")

                    self._sleep(self._interval)

                conn.close()

            except serial.SerialException as e:
                self.get_logger().error(f"Erreur série : {e} — reconnexion dans 5 s")
                self._publish({'status': 'error', 'error': str(e)})
                self._sleep(5.0)
            except Exception as e:
                self.get_logger().error(f"Erreur inattendue : {e}")
                self._sleep(5.0)

    def destroy_node(self) -> None:
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AanderaaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
