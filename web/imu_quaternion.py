#!/usr/bin/env python3
import html
import os
from datetime import datetime
from pathlib import Path

DATA_FILE = Path(
    os.environ.get(
        "IMU_QUATERNION_FILE",
        "/var/www/html/imu-data/imu_quaternion.txt",
    )
)
STALE_AFTER_SECONDS = 5


def read_quaternion():
    if not DATA_FILE.exists():
        return None

    lines = DATA_FILE.read_text(encoding="utf-8").splitlines()
    if len(lines) < 3:
        return None

    parts = lines[0].split()
    if len(parts) != 4:
        return None

    mtime = DATA_FILE.stat().st_mtime
    age_seconds = max(0, int(datetime.now().timestamp() - mtime))
    stale = age_seconds > STALE_AFTER_SECONDS

    return {
        "w": parts[0],
        "x": parts[1],
        "y": parts[2],
        "z": parts[3],
        "updated": lines[1],
        "sequence": lines[2],
        "age_seconds": str(age_seconds),
        "stale": stale,
    }


payload = read_quaternion()
print("Content-Type: text/html")
print()
print("<!DOCTYPE html>")
print("<html lang='en'>")
print("<head>")
print("  <meta charset='utf-8'>")
print("  <meta http-equiv='refresh' content='1'>")
print("  <title>IMU Quaternion</title>")
print("  <style>")
print("    body { font-family: Arial, sans-serif; margin: 2rem; background: #f6f7fb; color: #1f2937; }")
print("    h1 { margin-bottom: 0.5rem; }")
print("    .card { background: white; border-radius: 12px; padding: 1.5rem; max-width: 42rem; box-shadow: 0 8px 30px rgba(15, 23, 42, 0.08); }")
print("    .grid { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 1rem; margin-top: 1rem; }")
print("    .label { font-size: 0.85rem; color: #64748b; }")
print("    .value { font-size: 1.4rem; font-weight: 700; }")
print("    .ok { color: #166534; font-weight: 700; }")
print("    .stale { color: #b45309; font-weight: 700; }")
print("    code { background: #eef2ff; padding: 0.15rem 0.35rem; border-radius: 4px; }")
print("  </style>")
print("</head>")
print("<body>")
print("  <div class='card'>")
print("    <h1>Raspberry Pi IMU Quaternion</h1>")
print("    <p>Auto-refreshes every second. Data source: <code>%s</code></p>" % html.escape(str(DATA_FILE)))
if payload is None:
    print("    <p class='stale'>No quaternion file found yet. Start <code>imu_server</code> first.</p>")
else:
    css = 'stale' if payload['stale'] else 'ok'
    status = 'Stale data: imu_server may be stopped.' if payload['stale'] else 'Live data stream detected.'
    print("    <p class='%s'>%s</p>" % (css, html.escape(status)))
    print("    <div class='grid'>")
    for key in ("w", "x", "y", "z"):
        print("      <div><div class='label'>%s</div><div class='value'>%s</div></div>" % (key.upper(), html.escape(payload[key])))
    print("    </div>")
    print("    <p><strong>Updated:</strong> %s</p>" % html.escape(payload["updated"]))
    print("    <p><strong>Sample sequence:</strong> %s</p>" % html.escape(payload["sequence"]))
    print("    <p><strong>File age:</strong> %s second(s)</p>" % html.escape(payload["age_seconds"]))
print("  </div>")
print("</body>")
print("</html>")
