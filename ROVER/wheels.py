import numpy as np

class Wheel:
    """
    Modello fisico di una singola ruota del Rover e interazione terrameccanica con il suolo lunare.
    Calcola rotazione, velocità periferica, slittamento (slip) e forze di trazione/resistenza.
    """
    def __init__(self, name, rel_x, rel_y, radius=0.25, width=0.15, mass=3.0):
        self.name = name
        self.rel_x = float(rel_x)      # Posizione longitudinale rispetto al baricentro [m]
        self.rel_y = float(rel_y)      # Posizione trasversale rispetto al baricentro [m]
        self.radius = float(radius)    # Raggio ruota [m]
        self.width = float(width)      # Larghezza battistrada [m]
        self.mass = float(mass)        # Massa ruota [kg]
        self.inertia = 0.5 * self.mass * (self.radius ** 2) # Momento d'inerzia ruota [kg*m^2]

        # Stato dinamico
        self.omega = 0.0               # Velocità angolare [rad/s]
        self.rpm = 0.0                 # Giri al minuto [RPM]
        self.v_contact = 0.0           # Velocità lineare periferica al contatto [m/s]
        self.slip = 0.0                # Rapporto di slittamento (-1.0 locked, 0.0 rolling, +1.0 spinning)
        self.motor_torque = 0.0        # Coppia motrice applicata dal motore [N*m]
        self.normal_force = 0.0        # Carico verticale normale su questa ruota [N]
        self.tractive_force = 0.0      # Forza di trazione effettiva trasmessa al suolo [N]
        self.rolling_resistance = 0.0  # Forza di resistenza al rotolamento [N]

        # Posizione e terreno
        self.world_x = 0.0
        self.world_y = 0.0
        self.world_z = 0.0
        self.local_slope = 0.0

    def update_physics(self, world_x, world_y, terrain, chassis_v, motor_torque, normal_force, dt):
        """
        Aggiorna la terrameccanica della ruota a contatto con il DEM del terreno.
        """
        self.world_x = world_x
        self.world_y = world_y
        self.motor_torque = float(motor_torque)
        self.normal_force = max(5.0, float(normal_force))  # Minimo 5N per evitare divisioni per zero

        # 1. Lettura quota e pendenza locale dal terreno sotto la ruota
        try:
            terrain_data = terrain.get_terrain_data(self.world_x, self.world_y)
            self.world_z = terrain_data["altitude"]
            self.local_slope = terrain_data["slope_deg"]
        except Exception:
            self.world_z = 0.0
            self.local_slope = 0.0

        # 2. Cinematica ruota
        self.v_contact = self.omega * self.radius

        # 3. Calcolo slittamento (Slip Ratio secondo Bekker-Wong)
        denom = max(abs(self.v_contact), abs(chassis_v), 0.08)
        self.slip = float(np.clip((self.v_contact - chassis_v) / denom, -1.0, 1.0))

        # 4. Modello di aderenza su regolite lunare (curva non-lineare di trazione)
        mu_peak = 0.65  # Coefficiente di attrito massimo regolite compattato
        mu_traction = mu_peak * np.tanh(4.0 * self.slip)
        self.tractive_force = float(mu_traction * self.normal_force)

        # 5. Resistenza al rotolamento sul suolo soffice
        c_rr = 0.04 + 0.03 * np.sin(np.radians(self.local_slope))
        sign_v = np.sign(self.v_contact) if abs(self.v_contact) > 0.01 else np.sign(chassis_v)
        self.rolling_resistance = float(c_rr * self.normal_force * sign_v)

        # 6. Dinamica rotazionale dell'asse ruota: I * d_omega/dt = Torque - F_trazione * R - attriti
        bearing_friction = 0.15 * self.omega
        net_torque = self.motor_torque - (self.tractive_force * self.radius) - (self.rolling_resistance * self.radius) - bearing_friction

        d_omega = (net_torque / self.inertia) * dt
        self.omega += d_omega

        # Limite massimo RPM per motoriduttore del rover
        max_omega = 25.0  # ~240 RPM -> velocità max ~6.2 m/s
        self.omega = float(np.clip(self.omega, -max_omega, max_omega))
        self.rpm = float(self.omega * 60.0 / (2.0 * np.pi))

        # Forza longitudinale netta trasmessa al telaio
        net_longitudinal_force = self.tractive_force - self.rolling_resistance
        return net_longitudinal_force, self.world_z

    def get_telemetry(self):
        """Restituisce lo stato telemetrico della singola ruota."""
        return {
            "name": self.name,
            "rpm": self.rpm,
            "omega": self.omega,
            "v_contact": self.v_contact,
            "slip_pct": self.slip * 100.0,
            "normal_force_n": self.normal_force,
            "tractive_force_n": self.tractive_force,
            "local_slope_deg": self.local_slope,
            "z": self.world_z
        }


class FourWheelAssembly:
    """
    Gruppo 4 ruote motrici del Rover (Front-Left, Front-Right, Rear-Left, Rear-Right).
    Riceve i comandi di spinta motrice (WASD), distribuisce i carichi verticali
    sul suolo in base a beccheggio/rollio ed estrae la forza propulsiva e il momento di sterzo.
    """
    def __init__(self, wheelbase=1.2, track=1.0, wheel_radius=0.25):
        self.wheelbase = float(wheelbase)  # Interasse longitudinale L [m]
        self.track = float(track)          # Carreggiata trasversale W [m]
        self.wheel_radius = float(wheel_radius)

        half_l = self.wheelbase / 2.0
        half_w = self.track / 2.0

        # Istanziazione delle 4 ruote
        self.wheels = {
            "FL": Wheel("FL", +half_l, +half_w, radius=self.wheel_radius),
            "FR": Wheel("FR", +half_l, -half_w, radius=self.wheel_radius),
            "RL": Wheel("RL", -half_l, +half_w, radius=self.wheel_radius),
            "RR": Wheel("RR", -half_l, -half_w, radius=self.wheel_radius),
        }

    def compute_normal_loads(self, total_weight_n, pitch_rad, roll_rad):
        """
        Distribuisce il peso totale (m * g) sulle 4 ruote considerando il trasferimento
        di carico dovuto all'inclinazione su pendenza (beccheggio e rollio).
        """
        base_load = total_weight_n / 4.0

        # In salita (pitch > 0), il carico si sposta sulle ruote posteriori
        delta_pitch = (total_weight_n / 2.0) * np.sin(pitch_rad) * 0.8
        # In inclinazione a destra (roll > 0), il carico si sposta sulle ruote destre
        delta_roll = (total_weight_n / 2.0) * np.sin(roll_rad) * 0.8

        loads = {
            "FL": max(5.0, base_load - delta_pitch - delta_roll),
            "FR": max(5.0, base_load - delta_pitch + delta_roll),
            "RL": max(5.0, base_load + delta_pitch - delta_roll),
            "RR": max(5.0, base_load + delta_pitch + delta_roll),
        }
        return loads

    def apply_wasd_impulses(self, throttle_cmd, steer_cmd, brake_cmd, max_torque=25.0):
        """
        Trasforma gli input di pilotaggio in coppie motrici indipendenti per le ruote
        (funzionamento tipo macchinina radiocomandata 4WD con sterzo/differenziale).
        - throttle_cmd: [-1.0 .. +1.0] (W / S)
        - steer_cmd:    [-1.0 .. +1.0] (A / D, positivo = sinistra)
        - brake_cmd:    True / False (SPAZIO)
        """
        torques = {}

        if brake_cmd:
            # Freno attivo: applica coppia frenante proporzionale alla rotazione
            for name, w in self.wheels.items():
                torques[name] = -np.sign(w.omega) * min(35.0, abs(w.omega) * 5.0)
            return torques

        base_torque = float(throttle_cmd) * max_torque

        # Sterzata differenziale per sterzare sul posto o in curva (skid-steer / RC 4WD)
        # Sterzata a sinistra (A, steer > 0): ruote destre spingono di più, sinistre frenano/rallentano
        steer_diff = float(steer_cmd) * (max_torque * 0.65)

        torques["FL"] = base_torque - steer_diff
        torques["FR"] = base_torque + steer_diff
        torques["RL"] = base_torque - steer_diff
        torques["RR"] = base_torque + steer_diff

        return torques

    def update(self, rover_x, rover_y, rover_yaw, chassis_v, total_weight_n, pitch_rad, roll_rad, torques, terrain, dt):
        """
        Aggiorna tutte e 4 le ruote rispetto al terreno DEM.
        Restituisce la forza longitudinale netta totale, il momento di imbardata e le quote delle 4 ruote.
        """
        loads = self.compute_normal_loads(total_weight_n, pitch_rad, roll_rad)
        wheel_forces = {}
        wheel_z = {}

        cos_yaw = np.cos(rover_yaw)
        sin_yaw = np.sin(rover_yaw)

        for name, wheel in self.wheels.items():
            # Trasformazione coordinate da corpo vettura a coordinate mondo
            wx = rover_x + (wheel.rel_x * cos_yaw - wheel.rel_y * sin_yaw)
            wy = rover_y + (wheel.rel_x * sin_yaw + wheel.rel_y * cos_yaw)

            t_motor = torques.get(name, 0.0)
            f_norm = loads[name]

            f_net, wz = wheel.update_physics(wx, wy, terrain, chassis_v, t_motor, f_norm, dt)
            wheel_forces[name] = f_net
            wheel_z[name] = wz

        # Forza longitudinale totale applicata al telaio
        total_longitudinal_force = sum(wheel_forces.values())

        # Momento di imbardata (Yaw Torque): le forze delle ruote destre rispetto alle sinistre
        # Momento positivo = rotazione verso sinistra (antioraria)
        half_w = self.track / 2.0
        yaw_moment = (
            (wheel_forces["FR"] + wheel_forces["RR"]) -
            (wheel_forces["FL"] + wheel_forces["RL"])
        ) * half_w

        return total_longitudinal_force, yaw_moment, wheel_z

    def get_telemetry(self):
        """Restituisce la telemetria completa delle 4 ruote."""
        return {name: wheel.get_telemetry() for name, wheel in self.wheels.items()}
