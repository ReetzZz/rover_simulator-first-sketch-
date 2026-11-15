"""
Visualizzatore Grafico in Tempo Reale della Mappa DEM del Rover.
Si connette via UDP (porta 5005) e mostra la posizione, la rotta e la scia del rover sul suolo lunare.
Può essere eseguito in una finestra dedicata o lanciato automaticamente da main.py.
"""
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from ROVER.TERRAIN.terrain import TerrainMap
from COMUNICATION.comms import TelemetryClient

def run_map_viewer():
    print("[MAP VIEWER] Caricamento mappa altimetrica DEM NASA Site01...")
    terrain = TerrainMap()
    client = TelemetryClient(port=5005)

    # Regione di visualizzazione iniziale attorno al sito di allunaggio (-11000, -12000)
    center_x, center_y = -11000.0, -12000.0
    view_span = 400.0  # 400 metri per lato

    col_min, row_max = ~terrain.transform * (center_x - view_span, center_y - view_span)
    col_max, row_min = ~terrain.transform * (center_x + view_span, center_y + view_span)
    r0, r1 = max(0, int(row_min)), min(terrain.height, int(row_max))
    c0, c1 = max(0, int(col_min)), min(terrain.width, int(col_max))

    dem_crop = terrain.height_map[r0:r1, c0:c1]
    extent = [
        terrain.transform.c + c0 * terrain.pixel_size_x,
        terrain.transform.c + c1 * terrain.pixel_size_x,
        terrain.transform.f - r1 * terrain.pixel_size_y,
        terrain.transform.f - r0 * terrain.pixel_size_y
    ]

    fig, ax = plt.subplots(figsize=(9, 7))
    fig.canvas.manager.set_window_title("NASA Lunar Site 01 - Posizione Rover in Tempo Reale")

    im = ax.imshow(dem_crop, cmap="terrain", extent=extent, origin="upper")
    cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("Quota Altimetrica [m]")

    # Elementi grafici del rover
    path_x, path_y = [center_x], [center_y]
    trail_line, = ax.plot([], [], 'r-', linewidth=2.2, label="Traccia Rover")
    rover_dot, = ax.plot([center_x], [center_y], 'bo', markersize=8, zorder=6)
    
    # Freccia orientamento del rover (lunghezza ~15 metri sulla mappa per visibilità chiara)
    arrow_len = 15.0
    rover_arrow = ax.arrow(
        center_x, center_y, arrow_len, 0.0,
        head_width=8.0, head_length=6.0, fc='yellow', ec='black', linewidth=1.2, zorder=7
    )

    ax.scatter([center_x], [center_y], c='green', marker='s', s=80, label='Punto di Partenza', zorder=5)
    ax.set_title("Mappa Tattica del Terreno (NASA Site01)", fontsize=12, fontweight="bold")
    ax.set_xlabel("Coordinata Est X [m]")
    ax.set_ylabel("Coordinata Nord Y [m]")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="upper left")

    # HUD Informativo testuale
    hud_text = ax.text(
        0.02, 0.03, "In attesa di connessione telemetrica dal Rover...",
        transform=ax.transAxes, fontsize=10, fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="black", alpha=0.75, edgecolor="cyan"),
        color="white"
    )

    # Riferimento per aggiornare l'arrow
    arrow_container = [rover_arrow]

    def update_frame(frame):
        telemetry = client.receive_latest()
        if not telemetry:
            return trail_line, rover_dot, hud_text

        rx = telemetry["x"]
        ry = telemetry["y"]
        rz = telemetry["z"]
        yaw = telemetry.get("yaw_rad", np.radians(telemetry.get("yaw_deg", 0.0)))
        v = telemetry["v"]
        motion = telemetry.get("motion_state", "FERMO")

        # Aggiornamento traccia
        if len(path_x) == 0 or np.hypot(rx - path_x[-1], ry - path_y[-1]) > 0.8:
            path_x.append(rx)
            path_y.append(ry)
            trail_line.set_data(path_x, path_y)

        rover_dot.set_data([rx], [ry])

        # Ridisegno freccia di orientamento
        arrow_container[0].remove()
        dx = arrow_len * np.cos(yaw)
        dy = arrow_len * np.sin(yaw)
        new_arrow = ax.arrow(
            rx, ry, dx, dy,
            head_width=8.0, head_length=6.0, fc='yellow', ec='black', linewidth=1.2, zorder=7
        )
        arrow_container[0] = new_arrow

        # Aggiornamento HUD
        hud_text.set_text(
            f"ROVER: ({rx:.1f} m, {ry:.1f} m, Quota: {rz:.1f} m) | "
            f"Velocità: {v:.2f} m/s | Stato: {motion}"
        )

        # Pan automatico della vista se il rover si avvicina al bordo
        cur_xlim = ax.get_xlim()
        cur_ylim = ax.get_ylim()
        if not (cur_xlim[0] + 40 < rx < cur_xlim[1] - 40 and cur_ylim[0] + 40 < ry < cur_ylim[1] - 40):
            ax.set_xlim(rx - view_span, rx + view_span)
            ax.set_ylim(ry - view_span, ry + view_span)

        return trail_line, rover_dot, hud_text

    ani = animation.FuncAnimation(fig, update_frame, interval=60, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_map_viewer()
