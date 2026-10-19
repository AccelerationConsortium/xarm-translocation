import os
os.environ["PH_MODEL_PATH"] = r"C:\Users\sdl2\Projects\pH_Module\ph_module\ml\models\xgb_strip_tuned_20260619_203757_rmse0.0424_mae0.0264.pkl"
from ph_module import PHAnalyzer

PI_IP = "172.31.60.20" #old address 172.31.33.8

ph = PHAnalyzer(PI_IP)
try:
    ph.dispense_strip()
    sample = ph.read_ph(well="A1")
    if sample is None:
        print("read_ph failed (returned None)")
    else:
        print("CSV:  ", sample.get("csv_path"))
        print("ML pH:", sample.get("ph_ml"))
finally:
    ph.disconnect()   # always release the Pi for the next user