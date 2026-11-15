"""
Terminale di Controllo Telemetrico e Pilotaggio Radiocomandato del Rover.
Consente all'operatore di pilotare il rover con i tasti W, A, S, D e Spazio (tipo macchinina RC),
calcola in tempo reale la terrameccanica delle 4 ruote sul terreno DEM
e trasmette la telemetria continua via UDP (porta 5005) per la finestra della mappa.
"""
import os
import sys
import time
import msvcrt
import numpy as np

from ROVER.TERRAIN.terrain import TerrainMap
from ROVER.rover import Rover
from COMUNICATION.comms import CommsBridge

def make_gauge(val, max_val, length=16):
    """Genera una barra grafica ASCII per gli strumenti di bordo."""
    ratio = max(0.0, min(1.0, abs(val) / max_val))
    filled = int(round(ratio * length))
    return f"[{'█' * filled}{'░' * (length - filled)}] {val:4.1f}"

def make_steer_gauge(omega, max_omega=0.8, half_len=8):
    """Genera una barra con indicazione centrale di sterzata."""
    ratio = max(-1.0, min(1.0, omega / max_omega))
    pos = int(round(ratio * half_len))
    chars = [' '] * (2 * half_len + 1)
    center = half_len
    chars[center] = '|'
    if pos < 0:
        for i in range(center + pos, center):
            chars[i] = '◄'
    elif pos > 0:
        for i in range(center + 1, center + pos + 1):
            chars[i] = '►'
    return f"[{''.join(chars)}] {omega:+4.2f}"

def get_compass_symbol(yaw_deg):
    deg = (yaw_deg + 360.0) % 360.0
    symbols = ["► E", "↗ NE", "▲ N", "↖ NW", "◄ W", "↙ SW", "▼ S", "↘ SE"]
    idx = int(round(deg / 45.0)) % 8
    return symbols[idx]

def run_telemetry_terminal(start_x=-11000.0, start_y=-12000.0):
    os.system('')  # Abilita sequenze ANSI VT100
    print("[TELEMETRY TERMINAL] Avvio simulatore e connessione al terreno NASA DEM...")

    terrain = TerrainMap()
    rover = Rover(x=start_x, y=start_y, yaw_rad=0.0, terrain=terrain, mass=120.0, gravity=1.62)
    comms = CommsBridge(telem_port=5005, cmd_port=5006, enable_udp=True)

    dt = 0.08  # ~12 Hz
    sim_time = 0.0
    total_dist = 0.0
    prev_pos = (rover.x, rover.y)

    # Variabili di comando radiocomandato (impulsi ruote)
    throttle_cmd = 0.0   # [-1.0 .. +1.0]
    steer_cmd = 0.0      # [-1.0 .. +1.0]
    brake_cmd = False

    # Pulizia iniziale dello schermo
    sys.stdout.write('\033[2J\033[H')
    sys.stdout.flush()

    status_action = "PRONTO AL DECOLLO - PREMI W PER PARTIRE"

    try:
        while True:
            t0 = time.time()

            # 1. Lettura tasti non-bloccante dall'operatore
            steer_pressed = False
            throttle_pressed = False

            while msvcrt.kbhit():
                ch = msvcrt.getch()
                key = ""
                if ch in (b'\x00', b'\xe0'):
                    arrow = msvcrt.getch()
                    if arrow == b'H': key = "w"
                    elif arrow == b'P': key = "s"
                    elif arrow == b'K': key = "a"
                    elif arrow == b'M': key = "d"
                else:
                    try:
                        key = ch.decode('latin1').lower()
                    except:
                        pass

                if key == 'w':
                    throttle_cmd = min(1.0, throttle_cmd + 0.25)
                    brake_cmd = False
                    throttle_pressed = True
                    status_action = "IMPULSO MOTORE: AVANTI (+)"
                elif key == 's':
                    throttle_cmd = max(-0.8, throttle_cmd - 0.25)
                    brake_cmd = False
                    throttle_pressed = True
                    status_action = "IMPULSO MOTORE: RETROMARCIA / FRENO (-)"
                elif key == 'a':
                    steer_cmd = min(1.0, steer_cmd + 0.35)
                    steer_pressed = True
                    status_action = "IMPULSO STERZO: SINISTRA (◄)"
                elif key == 'd':
                    steer_cmd = max(-1.0, steer_cmd - 0.35)
                    steer_pressed = True
                    status_action = "IMPULSO STERZO: DESTRA (►)"
                elif key == ' ':
                    throttle_cmd = 0.0
                    steer_cmd = 0.0
                    brake_cmd = True
                    status_action = "FRENO DI EMERGENZA ATTIVATO (STOP)"
                elif key == 'c':
                    steer_cmd = 0.0
                    status_action = "STERZO RADDRIZZATO"
                elif key == 'q':
                    status_action = "FINE SESSIONE OPERATORE"
                    raise KeyboardInterrupt

            # Rilascio graduale dell'acceleratore e raddrizzamento dello sterzo
            if not throttle_pressed and abs(throttle_cmd) > 0.02:
                throttle_cmd *= 0.88
                if abs(throttle_cmd) < 0.02:
                    throttle_cmd = 0.0

            if not steer_pressed and abs(steer_cmd) > 0.02:
                steer_cmd *= 0.70
                if abs(steer_cmd) < 0.02:
                    steer_cmd = 0.0

            # 2. Passo fisico: il telaio e le 4 ruote interagiscono con il terreno
            telem = rover.step_rc(throttle_cmd, steer_cmd, brake_cmd, dt)
            comms.send_telemetry(telem)

            # Calcolo distanza percorsa
            cur_pos = (rover.x, rover.y)
            total_dist += np.hypot(cur_pos[0] - prev_pos[0], cur_pos[1] - prev_pos[1])
            prev_pos = cur_pos

            # 3. Formattazione allarmi
            alert_banner = "● STATO VEICOLO: NOMINALE (SICURO)"
            if telem["slope_deg"] > 25.0 or abs(telem["roll_deg"]) > 22.0:
                alert_banner = "▲ PERICOLO CRITICO: PENDENZA RIPIDA / RISCHIO RIBALTAMENTO"
            elif telem["slope_deg"] > 16.0:
                alert_banner = "▲ ATTENZIONE: FORTE PENDENZA / RIDURRE SPINTA RUOTE"
            elif telem["slip_avg_pct"] > 35.0:
                alert_banner = "▲ AVVISO: PATTINAMENTO ELEVATO SUL REGOLITE"

            # 4. Formattazione telemetria delle 4 singole ruote
            wheels = telem["wheels"]
            w_fl = wheels["FL"]
            w_fr = wheels["FR"]
            w_rl = wheels["RL"]
            w_rr = wheels["RR"]

            # Indicatori di moto visivi in chiaro
            motion_tag = f" {telem['motion_state']} "
            if "AVANTI" in telem['motion_state']:
                motion_color_box = f"▲▲ {motion_tag} ▲▲"
            elif "RETROMARCIA" in telem['motion_state']:
                motion_color_box = f"▼▼ {motion_tag} ▼▼"
            elif "STERZO" in telem['motion_state'] or "ROTAZIONE" in telem['motion_state']:
                motion_color_box = f"◄► {motion_tag} ◄►"
            else:
                motion_color_box = f"■■ {motion_tag} ■■"

            v_bar = make_gauge(telem["v"], 3.0)
            steer_bar = make_steer_gauge(telem["omega"], 0.8)
            slope_bar = make_gauge(telem["slope_deg"], 35.0)

            mins = int(sim_time) // 60
            secs = sim_time % 60
            compass = get_compass_symbol(telem["yaw_deg"])

            # 5. Cruscotto Console
            screen = f"""\033[H
╔═══════════════════════════════════════════════════════════════════════════════════════╗
║            🎮 POSTAZIONE TELEMETRICA LIVE ROVER (SIMULAZIONE RC NASA DEM)             ║
╠═══════════════════════════════════════════════════════════════════════════════════════╣
  COMANDI RC:  [W] Spinta Avanti   [S] Retro/Freno   [A] Sterza SX   [D] Sterza DX
               [SPAZIO] Freno Stop  [C] Raddrizza     [Q] Termina Missione
╟───────────────────────────────────────────────────────────────────────────────────────╢
  >>> STATO ATTUALE DEL MOTO:  [ {motion_color_box:^38} ] <<<
  Azione Operatore : {status_action:<45}
╟───────────────────────────────────────────────────────────────────────────────────────╢
  [POSIZIONE SUL TERRENO DEM]
    Coordinata X (Est)  : {telem['x']:>10.1f} m     Quota Altimetrica (Z) : {telem['z']:>8.1f} m
    Coordinata Y (Nord) : {telem['y']:>10.1f} m     Distanza Percorsa     : {total_dist:>8.1f} m
    Orientamento (Yaw)  : {telem['yaw_deg']:>10.1f} °     Rotta Bussola         : {compass}
╟───────────────────────────────────────────────────────────────────────────────────────╢
  [DINAMICA E ASSETTO TELAIO]
    Velocita' Reale     : {v_bar} m/s  ({telem['v_kmh']:>4.1f} km/h)
    Velocita' di Sterzo : {steer_bar} rad/s
    Pendenza Terreno    : {slope_bar} °   (Limite Sicuro: 25°)
    Beccheggio (Pitch)  : {telem['pitch_deg']:>+6.1f} °  ({ 'Salita ↗' if telem['pitch_deg'] > 0 else 'Discesa ↘' })
    Rollio (Roll)       : {telem['roll_deg']:>+6.1f} °  ({ 'Inclinazione Dx' if telem['roll_deg'] > 0 else 'Inclinazione Sx' })
    Condizione          : {alert_banner}
╟───────────────────────────────────────────────────────────────────────────────────────╢
  [TELEMETRIA INTERAZIONE RUOTE-SUOLO (TERRAMECCANICA 4WD)]
    RUOTA          GIRI [RPM]   VELOCITA' [m/s]   SLITTAMENTO     TRAZIONE [N]     CARICO [N]
    Front-Left  :  {w_fl['rpm']:>8.1f}     {w_fl['v_contact']:>8.2f}         {w_fl['slip_pct']:>+6.1f} %       {w_fl['tractive_force_n']:>8.1f} N       {w_fl['normal_force_n']:>6.1f} N
    Front-Right :  {w_fr['rpm']:>8.1f}     {w_fr['v_contact']:>8.2f}         {w_fr['slip_pct']:>+6.1f} %       {w_fr['tractive_force_n']:>8.1f} N       {w_fr['normal_force_n']:>6.1f} N
    Rear-Left   :  {w_rl['rpm']:>8.1f}     {w_rl['v_contact']:>8.2f}         {w_rl['slip_pct']:>+6.1f} %       {w_rl['tractive_force_n']:>8.1f} N       {w_rl['normal_force_n']:>6.1f} N
    Rear-Right  :  {w_rr['rpm']:>8.1f}     {w_rr['v_contact']:>8.2f}         {w_rr['slip_pct']:>+6.1f} %       {w_rr['tractive_force_n']:>8.1f} N       {w_rr['normal_force_n']:>6.1f} N
╟───────────────────────────────────────────────────────────────────────────────────────╢
  Tempo di Guida: {mins:02d}:{secs:04.1f} | Broadcast Telemetria UDP: 127.0.0.1:5005 (Attivo)
╚═══════════════════════════════════════════════════════════════════════════════════════╝"""

            sys.stdout.write(screen)
            sys.stdout.flush()

            sim_time += dt
            elapsed = time.time() - t0
            time.sleep(max(0.005, dt - elapsed))

    except KeyboardInterrupt:
        pass

    print("\n\n" + "=" * 70)
    print("           SESSIONE DI GUIDA TERMINATA DALL'OPERATORE")
    print("=" * 70)
    print(f" - Distanza percorsa totale : {total_dist:.1f} m")
    print(f" - Tempo totale di guida    : {sim_time:.1f} s")
    print(f" - Coordinate finali        : X={rover.x:.1f} m, Y={rover.y:.1f} m, Z={rover.z:.1f} m")
    print("=" * 70)

if __name__ == "__main__":
    run_telemetry_terminal()
