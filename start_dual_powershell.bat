@echo off
title Avvio Simulatore Rover Lunare
cd /d "%~dp0"
echo Avvio della Mappa e della Console Telemetrica in finestre separate...

start "NASA DEM Map Viewer" powershell -NoExit -Command "python map_viewer.py"
timeout /t 2 /nobreak >nul
start "Rover Telemetry Cockpit (WASD)" powershell -NoExit -Command "python telemetry_terminal.py"
