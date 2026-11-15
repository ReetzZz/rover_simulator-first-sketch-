import numpy as np
from .TERRAIN.terrain import TerrainMap
from .wheels import FourWheelAssembly

class Rover:
    """
    Simulatore fisico completo del Rover.
    Integra la dinamica del telaio con il gruppo a 4 ruote motrici indipendenti
    a contatto diretto con il terreno DEM (Digital Elevation Model).
    """
    def __init__(self, x=-11000.0, y=-12000.0, yaw_rad=0.0, terrain=None, mass=120.0, gravity=1.62):
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw_rad)
        self.v = 0.0                    # Velocità longitudinale del baricentro [m/s]
        self.omega = 0.0                # Velocità angolare di imbardata [rad/s]

        # Proprietà fisiche del veicolo
        self.mass = float(mass)         # Massa totale rover [kg]
        self.gravity = float(gravity)   # Accelerazione gravità [m/s^2] (1.62 per Luna / 9.81 per Terra)
        self.weight_n = self.mass * self.gravity
        self.wheelbase = 1.2            # Interasse L [m]
        self.track = 1.0                # Carreggiata W [m]
        self.inertia_z = (1.0 / 12.0) * self.mass * (self.wheelbase**2 + self.track**2)

        self.terrain = terrain if terrain is not None else TerrainMap()
        self.wheels = FourWheelAssembly(wheelbase=self.wheelbase, track=self.track, wheel_radius=0.25)

        # Assetto iniziale ricavato dalle 4 ruote sul terreno
        self.pitch_rad = 0.0
        self.roll_rad = 0.0
        self.pitch_deg = 0.0
        self.roll_deg = 0.0
        self.slope_deg = 0.0
        self.z = 0.0
        self.motion_state = "FERMO"

        # Prima valutazione altimetrica
        self._sync_terrain_initial()

    def _sync_terrain_initial(self):
        """Calcola la quota iniziale e l'assetto basato sui 4 punti di contatto."""
        torques = {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0}
        _, _, wheel_z = self.wheels.update(
            self.x, self.y, self.yaw, self.v, self.weight_n,
            self.pitch_rad, self.roll_rad, torques, self.terrain, dt=0.01
        )
        self.z = float(np.mean(list(wheel_z.values())))
        d_pitch = (wheel_z["FL"] + wheel_z["FR"]) - (wheel_z["RL"] + wheel_z["RR"])
        self.pitch_rad = float(np.arctan(d_pitch / (2.0 * self.wheelbase)))
        self.pitch_deg = float(np.degrees(self.pitch_rad))

        d_roll = (wheel_z["FL"] + wheel_z["RL"]) - (wheel_z["FR"] + wheel_z["RR"])
        self.roll_rad = float(np.arctan(d_roll / (2.0 * self.track)))
        self.roll_deg = float(np.degrees(self.roll_rad))
        self.slope_deg = float(np.degrees(np.arctan(np.sqrt(np.tan(self.pitch_rad)**2 + np.tan(self.roll_rad)**2))))

    def step_rc(self, throttle_cmd, steer_cmd, brake_cmd, dt=0.1):
        """
        Esegue il passo fisico della simulazione pilotato da impulsi tipo radiocomando (WASD):
        - throttle_cmd : [-1.0 .. +1.0] (W per avanti, S per retromarcia)
        - steer_cmd    : [-1.0 .. +1.0] (A per sinistra, D per destra)
        - brake_cmd    : True/False     (SPAZIO per frenata decisa)
        - dt           : Passo di integrazione temporale [secondi]
        """
        # 1. Calcolo coppie motrici distribuite alle 4 ruote
        torques = self.wheels.apply_wasd_impulses(throttle_cmd, steer_cmd, brake_cmd, max_torque=28.0)

        # 2. Aggiornamento interazione ruote-suolo sul DEM
        f_longitudinal, yaw_moment, wheel_z = self.wheels.update(
            self.x, self.y, self.yaw, self.v, self.weight_n,
            self.pitch_rad, self.roll_rad, torques, self.terrain, dt
        )

        # 3. Forze resistenti esterne (componente gravitazionale su pendenza + attrito suolo)
        f_gravity_slope = self.weight_n * np.sin(self.pitch_rad)
        f_drag = (12.0 * self.v) + (0.5 * 1.2 * (self.v ** 2) * np.sign(self.v))

        # 4. Accelerazione longitudinale netta del telaio: m * a = F_ruote - F_gravita - F_attrito
        f_net_chassis = f_longitudinal - f_gravity_slope - f_drag
        accel_x = f_net_chassis / self.mass

        # 5. Accelerazione angolare di imbardata (Yaw): I_z * alpha = M_yaw - smorzamento_sterzo
        damping_yaw = 22.0 * self.omega
        accel_yaw = (yaw_moment - damping_yaw) / self.inertia_z

        # 6. Integrazione dinamica
        self.v += accel_x * dt
        self.omega += accel_yaw * dt

        # Smorzamento a bassa velocità per evitare deriva numerica da fermo
        if brake_cmd or (abs(throttle_cmd) < 0.01 and abs(self.v) < 0.05):
            self.v *= 0.7
            if abs(self.v) < 0.01:
                self.v = 0.0

        if abs(steer_cmd) < 0.01 and abs(self.omega) < 0.03:
            self.omega *= 0.7
            if abs(self.omega) < 0.005:
                self.omega = 0.0

        # Saturazione limiti di sicurezza rover reale
        self.v = float(np.clip(self.v, -1.8, 3.2))
        self.omega = float(np.clip(self.omega, -0.85, 0.85))

        # 7. Integrazione cinematica di posizione e rotta sul piano
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw = float((self.yaw + self.omega * dt + np.pi) % (2.0 * np.pi) - np.pi)

        # 8. Calcolo assetto (quota, pitch, roll) derivato dall'elevazione delle 4 ruote
        self.z = float(np.mean(list(wheel_z.values())))

        d_pitch = (wheel_z["FL"] + wheel_z["FR"]) - (wheel_z["RL"] + wheel_z["RR"])
        self.pitch_rad = float(np.arctan(d_pitch / (2.0 * self.wheelbase)))
        self.pitch_deg = float(np.degrees(self.pitch_rad))

        d_roll = (wheel_z["FL"] + wheel_z["RL"]) - (wheel_z["FR"] + wheel_z["RR"])
        self.roll_rad = float(np.arctan(d_roll / (2.0 * self.track)))
        self.roll_deg = float(np.degrees(self.roll_rad))

        self.slope_deg = float(np.degrees(np.arctan(np.sqrt(np.tan(self.pitch_rad)**2 + np.tan(self.roll_rad)**2))))

        # 9. Stato del moto per l'operatore
        if brake_cmd:
            self.motion_state = "FRENATA ATTIVA"
        elif self.v > 0.08:
            if self.omega > 0.08:
                self.motion_state = "AVANTI + CURVA SX"
            elif self.omega < -0.08:
                self.motion_state = "AVANTI + CURVA DX"
            else:
                self.motion_state = "AVANTI"
        elif self.v < -0.08:
            if self.omega > 0.08:
                self.motion_state = "RETROMARCIA + STERZO SX"
            elif self.omega < -0.08:
                self.motion_state = "RETROMARCIA + STERZO DX"
            else:
                self.motion_state = "RETROMARCIA"
        elif abs(self.omega) > 0.08:
            self.motion_state = "ROTAZIONE SUL POSTO SX" if self.omega > 0 else "ROTAZIONE SUL POSTO DX"
        else:
            self.motion_state = "FERMO"

        return self.get_telemetry()

    def step(self, v_cmd, omega_cmd, dt=0.1):
        """Interfaccia di compatibilità per controllori automatici."""
        throttle = float(np.clip(v_cmd / 2.0, -1.0, 1.0))
        steer = float(np.clip(omega_cmd / 0.6, -1.0, 1.0))
        brake = (abs(v_cmd) < 0.05 and abs(omega_cmd) < 0.05)
        return self.step_rc(throttle, steer, brake, dt)

    def get_telemetry(self):
        """Restituisce il pacchetto telemetrico completo serializzabile in JSON."""
        wheel_data = self.wheels.get_telemetry()
        avg_slip = float(np.mean([abs(w["slip_pct"]) for w in wheel_data.values()]))

        return {
            "x": round(self.x, 2),
            "y": round(self.y, 2),
            "z": round(self.z, 2),
            "yaw_rad": round(self.yaw, 3),
            "yaw_deg": round(float(np.degrees(self.yaw)), 1),
            "v": round(self.v, 2),
            "v_kmh": round(self.v * 3.6, 1),
            "omega": round(self.omega, 3),
            "omega_degs": round(float(np.degrees(self.omega)), 1),
            "pitch_deg": round(self.pitch_deg, 1),
            "roll_deg": round(self.roll_deg, 1),
            "slope_deg": round(self.slope_deg, 1),
            "slip_avg_pct": round(avg_slip, 1),
            "motion_state": self.motion_state,
            "wheels": wheel_data
        }
