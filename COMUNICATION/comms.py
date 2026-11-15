import json
import socket

class CommsBridge:
    """
    Canale di comunicazione e telemetria:
    Supporta sia lo scambio dati diretto in-memory sia il broadcast UDP su localhost (porte 5005 e 5006).
    In questo modo la simulazione, il visualizzatore mappa e il terminale PowerShell possono girare
    nello stesso processo o in finestre PowerShell separate, oppure interfacciarsi con un server web.
    """
    def __init__(self, telem_port=5005, cmd_port=5006, enable_udp=True):
        self.telem_port = telem_port
        self.cmd_port = cmd_port
        self.enable_udp = enable_udp

        self._latest_telemetry = None
        self._latest_command = {"throttle": 0.0, "steer": 0.0, "brake": False}

        # Socket UDP di broadcast (invio telemetria e ricezione comandi)
        self._udp_sock = None
        if self.enable_udp:
            try:
                self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._udp_sock.setblocking(False)
            except Exception:
                self._udp_sock = None

    def send_telemetry(self, telemetry_dict):
        """Trasmette la telemetria dal Rover sia in memoria sia via broadcast UDP."""
        self._latest_telemetry = telemetry_dict
        if self._udp_sock:
            try:
                payload = json.dumps(telemetry_dict).encode('utf-8')
                self._udp_sock.sendto(payload, ("127.0.0.1", self.telem_port))
            except Exception:
                pass

    def get_telemetry(self):
        """Legge l'ultimo pacchetto telemetrico in memoria."""
        return self._latest_telemetry

    def send_command(self, throttle, steer, brake=False):
        """Invia i comandi di spinta motrice (W/A/S/D) al rover."""
        cmd = {
            "throttle": float(throttle),
            "steer": float(steer),
            "brake": bool(brake)
        }
        self._latest_command = cmd
        if self._udp_sock:
            try:
                payload = json.dumps(cmd).encode('utf-8')
                self._udp_sock.sendto(payload, ("127.0.0.1", self.cmd_port))
            except Exception:
                pass

    def get_command(self):
        """Legge l'ultimo comando di pilotaggio ricevuto."""
        return self._latest_command["throttle"], self._latest_command["steer"], self._latest_command["brake"]


class TelemetryClient:
    """
    Client leggero per processi esterni (es. Finestra Mappa o Server Web esterno).
    Si connette alla porta UDP 5005 e riceve la telemetria in tempo reale.
    """
    def __init__(self, port=5005):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", self.port))
        self.sock.setblocking(False)
        self.last_data = None

    def receive_latest(self):
        """Legge tutti i pacchetti pendenti e restituisce il più recente."""
        latest = None
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
                latest = json.loads(data.decode('utf-8'))
            except (BlockingIOError, socket.error):
                break
            except Exception:
                break
        if latest is not None:
            self.last_data = latest
        return self.last_data

    def close(self):
        try:
            self.sock.close()
        except:
            pass
