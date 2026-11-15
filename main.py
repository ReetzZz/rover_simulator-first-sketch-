"""
Launcher Principale della Simulazione Rover Lunare NASA Site01.
Avvia automaticamente:
1. La finestra della Mappa Tattica (con posizione rover, rotta e scia in tempo reale).
2. Il Cockpit Telemetrico interattivo (dove l'operatore pilota con i tasti W, A, S, D come una macchinina RC).

È anche possibile avviarli separatamente in due diverse PowerShell:
- PowerShell 1 (Mappa)     : python map_viewer.py
- PowerShell 2 (Telemetria): python telemetry_terminal.py
"""
import os
import sys
import time
import subprocess

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    map_script = os.path.join(base_dir, "map_viewer.py")
    terminal_script = os.path.join(base_dir, "telemetry_terminal.py")

    # Controlla argomenti opzionali
    if len(sys.argv) > 1:
        arg = sys.argv[1].lower()
        if arg in ("--map", "-m"):
            from map_viewer import run_map_viewer
            run_map_viewer()
            return
        elif arg in ("--telemetry", "-t"):
            from telemetry_terminal import run_telemetry_terminal
            run_telemetry_terminal()
            return

    print("=" * 70)
    print("      NASA LUNAR ROVER SIMULATOR - AVVIO SISTEMI MULTI-FINESTRA")
    print("=" * 70)
    print("\n[1/2] Apertura finestra grafica della MAPPA TATTICA in corso...")

    map_proc = None
    try:
        # Avvia il visualizzatore mappa in un processo grafico separato
        map_proc = subprocess.Popen(
            [sys.executable, map_script],
            cwd=base_dir
        )
        time.sleep(1.2)  # Breve attesa per permettere alla finestra di aprirsi
    except Exception as e:
        print(f"Attenzione: impossibile avviare automaticamente la finestra mappa: {e}")
        print("Puoi aprirla manualmente in un'altra PowerShell con: python map_viewer.py\n")

    print("[2/2] Avvio del Cockpit Telemetrico in questa console PowerShell...")
    time.sleep(0.5)

    try:
        from telemetry_terminal import run_telemetry_terminal
        run_telemetry_terminal()
    finally:
        if map_proc is not None:
            try:
                map_proc.terminate()
            except:
                pass

if __name__ == "__main__":
    main()
