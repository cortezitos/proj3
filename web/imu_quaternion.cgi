#!/usr/bin/env python3
import html
import math
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


def read_payload():
    if not DATA_FILE.exists():
        return None

    lines = DATA_FILE.read_text(encoding="utf-8").splitlines()
    if len(lines) < 3:
        return None

    parts = lines[0].split()
    if len(parts) != 4:
        return None

    try:
        values = [float(part) for part in parts]
    except ValueError:
        return None

    age_seconds = max(0, int(datetime.now().timestamp() - DATA_FILE.stat().st_mtime))
    magnitude = math.sqrt(sum(value * value for value in values))
    return {
        "w": f"{values[0]:.6f}",
        "x": f"{values[1]:.6f}",
        "y": f"{values[2]:.6f}",
        "z": f"{values[3]:.6f}",
        "updated": lines[1],
        "sequence": lines[2],
        "age_seconds": age_seconds,
        "stale": age_seconds > STALE_AFTER_SECONDS,
        "magnitude": f"{magnitude:.6f}",
    }


payload = read_payload()

print("Content-Type: text/html")
print()
print("<!DOCTYPE html>")
print("<html lang='en'>")
print("<head>")
print("  <meta charset='utf-8'>")
print("  <meta http-equiv='refresh' content='1'>")
print("  <meta name='viewport' content='width=device-width, initial-scale=1'>")
print("  <title>RPi IMU Quaternion</title>")
print("  <style>")
print("    :root { --bg: #f3efe4; --ink: #112a46; --muted: #5d6b7a; --card: #fffdf8; --accent: #b55d3d; --ok: #1e6f5c; --warn: #9c5b2e; }")
print("    body { margin: 0; font-family: Georgia, 'Times New Roman', serif; background: radial-gradient(circle at top, #fff8ea 0%, var(--bg) 58%, #e8e0d3 100%); color: var(--ink); }")
print("    .shell { max-width: 860px; margin: 0 auto; padding: 32px 20px 48px; }")
print("    .panel { background: rgba(255, 253, 248, 0.96); border: 1px solid rgba(17, 42, 70, 0.08); border-radius: 18px; padding: 24px; box-shadow: 0 18px 40px rgba(17, 42, 70, 0.10); }")
print("    h1 { margin: 0 0 10px; font-size: clamp(1.8rem, 3vw, 2.7rem); letter-spacing: 0.02em; }")
print("    .lede { margin: 0; color: var(--muted); line-height: 1.5; }")
print("    .status { margin-top: 18px; font-weight: 700; }")
print("    .ok { color: var(--ok); }")
print("    .warn { color: var(--warn); }")
print("    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 14px; margin-top: 22px; }")
print("    .tile { background: white; border: 1px solid rgba(17, 42, 70, 0.08); border-radius: 14px; padding: 14px 16px; }")
print("    .label { color: var(--muted); font-size: 0.85rem; text-transform: uppercase; letter-spacing: 0.08em; }")
print("    .value { margin-top: 8px; font-size: 1.45rem; font-weight: 700; }")
print("    .meta { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 12px; margin-top: 20px; }")
print("    .meta div { background: rgba(181, 93, 61, 0.06); border-radius: 12px; padding: 12px 14px; }")
print("    code { background: rgba(17, 42, 70, 0.08); padding: 2px 6px; border-radius: 6px; }")
print("  </style>")
print("</head>")
print("<body>")
print("  <main class='shell'>")
print("    <section class='panel'>")
print("      <h1>Raspberry Pi IMU Quaternion</h1>")
print("      <p class='lede'>Project 3 Task 1 sensor web page. The Raspberry Pi reads the BerryIMU, runs Madgwick fusion, normalizes the quaternion, and refreshes this CGI page every second.</p>")
print(f"      <p class='lede'>Data file: <code>{html.escape(str(DATA_FILE))}</code></p>")

if payload is None:
    print("      <p class='status warn'>No quaternion file is available yet. Start `imu_web_publisher`, `imu_socket_server`, or `imu_multithread_server` first.</p>")
else:
    status_class = "warn" if payload["stale"] else "ok"
    status_text = "Stale data detected. The IMU publisher is likely stopped." if payload["stale"] else "Live quaternion stream detected."
    print(f"      <p class='status {status_class}'>{html.escape(status_text)}</p>")
    print("      <div class='grid'>")
    for key in ("w", "x", "y", "z"):
        print("        <article class='tile'>")
        print(f"          <div class='label'>{key}</div>")
        print(f"          <div class='value'>{html.escape(payload[key])}</div>")
        print("        </article>")
    print("      </div>")
    print("      <div class='meta'>")
    print(f"        <div><strong>Updated</strong><br>{html.escape(payload['updated'])}</div>")
    print(f"        <div><strong>Sample sequence</strong><br>{html.escape(payload['sequence'])}</div>")
    print(f"        <div><strong>File age</strong><br>{payload['age_seconds']} second(s)</div>")
    print(f"        <div><strong>|q| magnitude</strong><br>{html.escape(payload['magnitude'])}</div>")
    print("      </div>")

print("    </section>")
print("  </main>")
print("</body>")
print("</html>")
