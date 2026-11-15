# NASA Lunar Rover Simulation & Telemetry Cockpit (Site01)

Simulatore fisico realistico per Rover lunare 4WD sviluppato per l'integrazione in piattaforme di controllo e siti web/dashboard (ad es. tramite Claude Code o backend FastAPI/WebSocket).

---

## 1. Architettura della Simulazione

Il simulatore modella la fisica reale del rover senza scorciatoie:
```
           +------------------------------------------+
           |       TERRENO LUNARE (DEM NASA LRO)      |
           |   Quota Z, pendenze locali, gradienti    |
           +--------------------+---------------------+
                                | Lettura quota e pendenza sotto ogni ruota
                                v
           +------------------------------------------+
           |        4 RUOTE MOTRICI (FL, FR, RL, RR)  |
           |  - Coppie motrici (WASD / Radiocomando)  |
           |  - Dinamica asse: I * d_omega/dt = T - F*R|
           |  - Slittamento (Slip Ratio Bekker-Wong)  |
           |  - Forze di trazione e rotolamento       |
           |  - Trasferimento di carico (Pitch/Roll)  |
           +--------------------+---------------------+
                                | Risultante forze e coppie di imbardata
                                v
           +------------------------------------------+
           |            DINAMICA DEL TELAIO           |
           |  - Accelerazione lineare e angolare      |
           |  - Integrazione rotta (Yaw) e posizione  |
           |  - Assetto 3D: Quota, Pitch e Roll       |
           +--------------------+---------------------+
                                | Telemetria serializzata JSON
                                v
      +-------------------------+-------------------------+
      | (Porta UDP 5005)                                  | (In-memory)
      v                                                   v
+-------------------------+             +-------------------------------+
|   MAP VIEWER (GUI)      |             | TELEMETRY COCKPIT (PowerShell)|
| Posizione su mappa DEM, |             | Comandi W/A/S/D macchinina RC,|
| rotta e traccia reale   |             | telemetria live e allarmi     |
+-------------------------+             +-------------------------------+
```

---

## 2. Come Avviare la Simulazione

### Metodo 1: Avvio Unificato (Consigliato)
Esegui da PowerShell:
```bash
python main.py
```
Apre automaticamente la finestra grafica della **Mappa Tattica** e attiva il **Cockpit di Pilotaggio** nella console corrente.

### Metodo 2: Due Finestre PowerShell Separate
Puoi aprire due PowerShell distinte:
- **Finestra 1 (Mappa Tattica su DEM)**:
  ```bash
  python map_viewer.py
  ```
- **Finestra 2 (Cockpit Telemetrico con comandi WASD)**:
  ```bash
  python telemetry_terminal.py
  ```
*(Oppure fai doppio clic su `start_dual_powershell.bat` per aprirle entrambe contemporaneamente).*

---

## 3. Comandi di Guida Radiocomandata (WASD)

Il veicolo risponde come una macchinina radiocomandata 4WD:
- **`W`**: Impulso motore AVANTI (+ trazione alle 4 ruote).
- **`S`**: Impulso motore RETROMARCIA / FRENO.
- **`A`**: Sterzata SINISTRA (spinta differenziale ruote destre e rallentamento sinistre).
- **`D`**: Sterzata DESTRA (spinta differenziale ruote sinistre e rallentamento destre).
- **`SPAZIO`**: Freno di stazionamento istantaneo (STOP ruote).
- **`C`**: Raddrizza lo sterzo.
- **`Q`**: Termina la sessione di guida.

---

## 4. Guida per l'Integrazione in altri Progetti / Web (per Claude Code)

La telemetria viene trasmessa in streaming continuo via **UDP JSON su `127.0.0.1:5005`**.
Un server web (ad es. FastAPI, Flask, Node.js WebSocket) può ricevere i dati semplicemente aprendo un socket UDP:

```python
import socket
import json

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 5005))

while True:
    data, _ = sock.recvfrom(65535)
    telemetry = json.loads(data.decode("utf-8"))
    # Invia via WebSocket al frontend web
    print("Posizione:", telemetry["x"], telemetry["y"], "Stato:", telemetry["motion_state"])
```

### Struttura del Pacchetto Telemetrico JSON
```json
{
  "x": -10984.2,
  "y": -11993.5,
  "z": 1944.1,
  "yaw_rad": 0.596,
  "yaw_deg": 34.2,
  "v": 0.85,
  "v_kmh": 3.1,
  "omega": -0.05,
  "omega_degs": -2.9,
  "pitch_deg": 3.1,
  "roll_deg": -1.2,
  "slope_deg": 5.8,
  "slip_avg_pct": 7.4,
  "motion_state": "AVANTI",
  "wheels": {
    "FL": { "rpm": 32.5, "v_contact": 0.85, "slip_pct": 5.2, "tractive_force_n": 38.4, "normal_force_n": 58.2 },
    "FR": { "rpm": 32.5, "v_contact": 0.85, "slip_pct": 5.2, "tractive_force_n": 38.4, "normal_force_n": 58.2 },
    "RL": { "rpm": 32.5, "v_contact": 0.85, "slip_pct": 5.2, "tractive_force_n": 41.2, "normal_force_n": 63.3 },
    "RR": { "rpm": 32.5, "v_contact": 0.85, "slip_pct": 5.2, "tractive_force_n": 41.2, "normal_force_n": 63.3 }
  }
}
```
