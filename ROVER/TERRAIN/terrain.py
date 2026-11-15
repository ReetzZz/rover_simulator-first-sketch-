import os
import rasterio
import numpy as np

class TerrainMap:
    """
    Gestione della mappa altimetrica DEM (Digital Elevation Model) NASA/LRO.
    Calcola altitudine, gradienti spaziali, pendenza (slope), stima dello slittamento (slip)
    e assetto del rover (pitch e roll) lungo la rotta.
    """
    def __init__(self, tif_path=None):
        if tif_path is None:
            tif_path = os.path.join(os.path.dirname(__file__), "Site01_final_adj_5mpp_surf.tif")

        if not os.path.exists(tif_path):
            raise FileNotFoundError(f"File DEM non trovato al percorso: {tif_path}")

        # Apertura del raster geografico
        with rasterio.open(tif_path) as terrain:
            self.height_map = terrain.read(1).astype(float)
            self.transform = terrain.transform
            self.nodata = terrain.nodata
            self.pixel_size_x = abs(self.transform.a)
            self.pixel_size_y = abs(self.transform.e)
            self.width = terrain.width
            self.height = terrain.height
            self.bounds = terrain.bounds

        # Gestione valori nulli/mancanti
        if self.nodata is not None:
            self.height_map[self.height_map == self.nodata] = np.nan

        # Calcolo gradienti spaziali:
        # np.gradient restituisce (d_rows, d_cols).
        # In coordinate cartografiche, l'asse Y (Nord) è opposto all'aumento delle righe della matrice raster.
        grad_row, grad_col = np.gradient(
            self.height_map,
            self.pixel_size_y,
            self.pixel_size_x
        )
        self.dz_dx = grad_col        # Gradiente verso Est (+X)
        self.dz_dy = -grad_row       # Gradiente verso Nord (+Y)

        # Calcolo della pendenza complessiva del terreno (modulo del gradiente)
        self.slope_rad = np.arctan(np.sqrt(self.dz_dx**2 + self.dz_dy**2))
        self.slope_deg = np.degrees(self.slope_rad)

        # Modello di slittamento (slip) terrameccanico semplice:
        # Lo slittamento delle ruote aumenta con la pendenza (da ~3% su piano fino a ~50% su pendenze elevate)
        self.slip_map = np.clip(np.sin(self.slope_rad) * 0.5 + 0.03, 0.03, 0.8)

    def get_terrain_data(self, x, y, heading_rad=0.0):
        """
        Dato punto (x, y) in coordinate metriche e l'orientamento del rover (heading in radianti),
        restituisce i parametri del terreno e l'assetto risultante.
        """
        col, row = ~self.transform * (x, y)
        row_idx = int(round(row))
        col_idx = int(round(col))

        if not (0 <= row_idx < self.height and 0 <= col_idx < self.width):
            raise ValueError(f"Rover fuori dalla mappa: x={x:.1f}, y={y:.1f} (riga={row_idx}, colonna={col_idx})")

        altitude = float(self.height_map[row_idx, col_idx])
        slope = float(self.slope_deg[row_idx, col_idx])
        slip = float(self.slip_map[row_idx, col_idx])

        if np.isnan(altitude):
            raise ValueError(f"Dati altimetrici non disponibili alle coordinate ({x:.1f}, {y:.1f})")

        # Proiezione del gradiente sull'asse longitudinale del rover (Beccheggio / Pitch)
        # Pitch > 0: rover con muso inclinato verso l'alto (salita)
        gx = self.dz_dx[row_idx, col_idx]
        gy = self.dz_dy[row_idx, col_idx]
        grad_forward = gx * np.cos(heading_rad) + gy * np.sin(heading_rad)
        pitch_deg = float(np.degrees(np.arctan(grad_forward)))

        # Proiezione del gradiente sull'asse trasversale del rover (Rollio / Roll)
        # Roll > 0: inclinazione a destra
        grad_lateral = -gx * np.sin(heading_rad) + gy * np.cos(heading_rad)
        roll_deg = float(np.degrees(np.arctan(grad_lateral)))

        return {
            "altitude": altitude,
            "slope_deg": slope,
            "slip": slip,
            "pitch_deg": pitch_deg,
            "roll_deg": roll_deg
        }

# Istanza predefinita per chiamate veloci compatibili con lo script originario
_DEFAULT_TERRAIN = None

def get_terrain_data(x, y, heading_rad=0.0):
    global _DEFAULT_TERRAIN
    if _DEFAULT_TERRAIN is None:
        _DEFAULT_TERRAIN = TerrainMap()
    data = _DEFAULT_TERRAIN.get_terrain_data(x, y, heading_rad)
    return data["altitude"], data["slope_deg"], data["slip"]
