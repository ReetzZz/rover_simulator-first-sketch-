"""
Modulo di utilita' per l'interfaccia telemetrica e il monitoraggio dello stato del veicolo.
Fornisce funzioni di serializzazione e diagnostica pronte per l'integrazione
con server web (FastAPI, Flask, WebSocket) o dashboard esterne.
"""
import json

def format_telemetry_json(telemetry_dict):
    """Restituisce il payload telemetrico in formato stringa JSON standard."""
    return json.dumps(telemetry_dict, indent=2)

def get_wheel_summary(telemetry_dict):
    """Estrae e riassume le grandezze terrameccaniche delle 4 ruote."""
    wheels = telemetry_dict.get("wheels", {})
    summary = {}
    for name, data in wheels.items():
        summary[name] = {
            "rpm": data.get("rpm", 0.0),
            "v_contact_ms": data.get("v_contact", 0.0),
            "slip_pct": data.get("slip_pct", 0.0),
            "traction_n": data.get("tractive_force_n", 0.0),
            "load_n": data.get("normal_force_n", 0.0)
        }
    return summary
