import numpy as np

class RoverController:
    """
    Modulo di Controllo e Guida del Rover (non-overengineered).
    Riceve le variabili di telemetria, segue una sequenza di waypoint,
    applica un controllo di direzione (P-controller su errore di rotta)
    e adatta la velocità in base all'angolo e alla pendenza del terreno.
    """
    def __init__(self, waypoints, target_radius=2.0, cruise_speed=0.8, kp_heading=1.2, max_slope_limit=30.0):
        self.waypoints = list(waypoints)
        self.current_idx = 0
        self.target_radius = target_radius
        self.cruise_speed = cruise_speed
        self.kp_heading = kp_heading
        self.max_slope_limit = max_slope_limit
        self.mission_complete = False

    def update(self, telemetry):
        """
        Elabora la telemetria ricevuta e calcola i comandi (v_cmd, omega_cmd).
        
        Variabili in ingresso richieste dal rover:
        - telemetry['x']: coordinata X attuale [m]
        - telemetry['y']: coordinata Y attuale [m]
        - telemetry['yaw']: orientamento attuale del rover [rad]
        - telemetry['slope_deg']: pendenza locale del terreno [gradi]
        - telemetry['roll_deg']: inclinazione laterale [gradi]
        """
        if self.mission_complete or self.current_idx >= len(self.waypoints):
            self.mission_complete = True
            return 0.0, 0.0, {"status": "MISSION_COMPLETE", "dist": 0.0, "error_deg": 0.0}

        target_x, target_y = self.waypoints[self.current_idx]
        current_x = telemetry["x"]
        current_y = telemetry["y"]
        current_yaw = telemetry["yaw"]
        slope_deg = telemetry.get("slope_deg", 0.0)
        roll_deg = telemetry.get("roll_deg", 0.0)

        # 1. Distanza euclidea dal waypoint corrente
        dx = target_x - current_x
        dy = target_y - current_y
        dist = np.sqrt(dx**2 + dy**2)

        # 2. Controllo raggiungimento waypoint
        if dist <= self.target_radius:
            self.current_idx += 1
            if self.current_idx >= len(self.waypoints):
                self.mission_complete = True
                return 0.0, 0.0, {"status": "TARGET_REACHED_ALL", "dist": 0.0, "error_deg": 0.0}
            target_x, target_y = self.waypoints[self.current_idx]
            dx = target_x - current_x
            dy = target_y - current_y
            dist = np.sqrt(dx**2 + dy**2)

        # 3. Calcolo angolo desiderato e errore di rotta
        target_yaw = np.arctan2(dy, dx)
        heading_error = (target_yaw - current_yaw + np.pi) % (2.0 * np.pi) - np.pi
        heading_error_deg = float(np.degrees(heading_error))

        # 4. Controllo sterzo (velocità angolare omega)
        omega_cmd = self.kp_heading * heading_error

        # 5. Controllo velocità longitudinale con modulazione di sicurezza
        # Se l'errore di orientamento è ampio (> 45°), rallenta per privilegiare la rotazione
        if abs(heading_error_deg) > 45.0:
            speed = 0.15 * self.cruise_speed
        else:
            speed = self.cruise_speed * max(0.0, np.cos(heading_error))

        # 6. Sicurezza terreno: riduzione velocità su pendenze elevate e stop di sicurezza
        if slope_deg > self.max_slope_limit or abs(roll_deg) > 25.0:
            # Pendenza critica: stop precauzionale per rischio ribaltamento/slittamento
            v_cmd = 0.0
            omega_cmd = 0.0
            status = "CRITICAL_SLOPE_STOP"
        elif slope_deg > 15.0:
            # Pendenza media: fattore di sicurezza per ridurre la velocità
            factor = max(0.2, 1.0 - (slope_deg - 15.0) / 20.0)
            v_cmd = speed * factor
            status = "SLOPE_CAUTION"
        else:
            v_cmd = speed
            status = "NAVIGATING"

        control_info = {
            "status": status,
            "target_idx": self.current_idx,
            "target_coords": (target_x, target_y),
            "distance_to_target": float(dist),
            "heading_error_deg": heading_error_deg,
            "v_cmd": float(v_cmd),
            "omega_cmd": float(omega_cmd)
        }

        return v_cmd, omega_cmd, control_info
