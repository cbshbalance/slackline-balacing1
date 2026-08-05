import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "v19_bringup"))
sys.path.insert(0, HERE)

from sim_engine import SimEngine
import params_v19 as p19

# Instantiate engine to capture exact parameters used in current 80s balance run
eng = SimEngine()

preset_data = {
    "name": "80s Recovery (움찔 후 복원 명장면)",
    "description": "80여 초 이상 균형을 유지하며 중간 외란/움찔 후 극적으로 복원하는 파라미터 프리셋",
    "timestamp": "2026-07-24",
    "ctrl_mode": eng.ctrl_mode,
    "estimator": eng.estimator,
    "servo_aware": eng.servo_aware,
    "gain_scale": eng.gain_scale,
    "kp_servo": 25.0,
    "kd_servo": 0.4,
    "x0": list(eng.x0),
    "real": eng.real,
    "mismatch": eng.mismatch,
    "params": {
        "W_NOLOAD_DPS": p19.SERVO.get("W_NOLOAD_DPS", 462.0),
        "T_FOLD": p19.CONTROL.get("T_FOLD", 0.06),
        "T_WAIT": p19.CONTROL.get("T_WAIT", 0.02),
        "T_EXTEND": p19.CONTROL.get("T_EXTEND", 0.09),
        "T_REST": p19.CONTROL.get("T_REST", 0.06),
        "A_TRIGGER_DEG": p19.CONTROL.get("A_TRIGGER_DEG", 0.6),
        "RHO": p19.CONTROL.get("RHO", 0.9),
    }
}

target_preset_file = os.path.join(HERE, "preset_recovery_80s.json")

with open(target_preset_file, "w", encoding="utf-8") as f:
    json.dump(preset_data, f, indent=2, ensure_ascii=False)

print(f"Successfully saved preset: {target_preset_file}")
