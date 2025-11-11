<!-- README content inserted by Codex -->
# BLINC iM - Foxglove Wind-Turbine Dashboard

## Overview
`src/BLINC_iM_app.py` is the primary entry point for the BLINC iM Foxglove demo.
It renders a wind turbine, tracker, and SP payloads in Foxglove Studio, while a companion
web dashboard exposes live controls, state snapshots, and inline assets (logos).

## Features
- Full Foxglove 3D scene: tower, nacelle, blades, tracker mast, SP bullseyes.
- Dual sensor rig on the tracker (Benewake + Livox) and optional SP scanner (S-Laser + camera).
- Animated yaw oscillation with rate limiting.
- HTTP API + dashboard panel (HTML/JS) with live telemetry and controls.
- Asset pipeline for GTMW/Pixxon logos, served directly by the dashboard.
- Blade-hit counter based on frustum intersections.
- Utility helpers for pan/tilt aiming, ENU->WGS84 conversion, bullseye geometry, and camera HUD mask.

## Requirements
- **Python:** 3.10+
- **Packages:**
  - `foxglove` (Python client for Foxglove Studio)
  - `PyYAML` (only needed for legacy scripts in `archive/`)
- **Foxglove Studio:** desktop or web, able to subscribe to `ws://localhost:8765`

Ensure logos (`logo_GTMW.png`, `logo_Pixxon.png`) live in `assets/`.

Install dependencies:

```bash
python -m venv .venv
.\.venv\Scripts\activate        # Windows PowerShell
pip install foxglove-client pyyaml
```

(Use `source .venv/bin/activate` on macOS/Linux.)

## Quick Start
1. Activate your virtual environment.
2. Run the dashboard:
   ```bash
   python src/BLINC_iM_app.py
   ```
3. Start Foxglove Studio and connect to `ws://localhost:8765`.
4. Open a browser to `http://localhost:8888/` for the control panel.

You'll see console logs confirming the Foxglove server, HTTP server, and asset loads.

Tablet access: You can view the dashboard from a tablet. On iPad, use the Chrome browser and navigate to `http://<PC-IP>:8888`, replacing `<PC-IP>` with the IP address of the computer running the Python script.

Finding your PC IP (for tablet access):
- Windows: run `ipconfig` in PowerShell or Command Prompt and use the IPv4 Address of your active adapter (e.g., `192.168.x.x`).
- macOS/Linux: run `ip addr show` (Linux) or `ifconfig` (macOS) and use the address on your primary interface.
- Ensure the tablet is on the same network and that your firewall allows inbound access on port `8888` for Python.

## Scene & Control Flow
- **Static Scene** (`build_scene_entities`):
  - Tower, nacelle, hub, nose cone, blades
  - Tracker mast + SP mast, bullseyes, ground ring
  - Camera HUD mask + anchor entity for Foxglove panels
- **Transforms**:
  - `publish_world_root`, `publish_scan_frame`, `publish_nacelle_frames`
  - Sensor chains for Benewake, Livox, S-Laser, camera
  - Pan/tilt derived from `solve_pan_tilt_to_target` with sensor offsets
- **Yaw Oscillation**:
  - Modes: `OFF`, `SINE`, `RANDOM`
  - Rate/limit parameters accessible via `/set`
- **Blade Hit Detection**:
  - `_point_in_benewake_fov` tests blade points inside Benewake frustum
  - `compute_live_state` reports per-blade active state and hit counts

## HTTP API
`ParamHTTPHandler` exposes:

| Endpoint     | Purpose                                 |
|--------------|------------------------------------------|
| `/`          | HTML dashboard (tables, controls, SVG)  |
| `/img/gt`    | GTMW logo (served from `assets`)        |
| `/img/px`    | Pixxon logo                             |
| `/get`       | Current parameter snapshot (JSON)       |
| `/state`     | Live state (yaw, hits, GPS, etc.)       |
| `/set?k=v...`  | Update parameters (numbers or strings)  |

Parameters: `yaw_deg`, `yaw_rate_deg_s`, `rotor_rpm`, `rotor_dir`, `model_number`, `scanner_sp`,
`rotor_dia_m`, `hub_ht_m`, `yaw_osc_mode`, `yaw_osc_rate_deg_s`, `yaw_osc_max_abs_deg`,
`yaw_osc_freq_hz`, `yaw_osc_random_step_deg`.

## Expected Outputs
- **Foxglove Studio:** animated turbine, sensor frusta, camera HUD.
- **Browser:** dashboard with live tables, SP selection, yaw controls.
- **Console:**
  - Asset load diagnostics
  - HTTP requests
  - Blade hit counts
  - Param set confirmations
  - Foxglove updates (`FrameTransforms`, `SceneUpdate`)

## Project Structure
```
.
+- src/
|  +- BLINC_iM_app.py              # main entry point
+- assets/
|  +- logo_GTMW.png
|  +- logo_Pixxon.png
+- archive/
|  +- BLINC_APP.py                 # prior dashboard revision
|  +- blinc_im_dashboard.py        # earlier prototype
|  +- No_dashboard/                # turbine-only demos (WTG.py, etc.)
+- docs/                           # (add architecture notes here)
-|  +- media/                      # store screen recordings or demos
+- README.md                       # (this file)
```

Archive scripts reference the same assets directory; they're kept for historical context.

## Demo Video
Place any screen recordings in `docs/media/` (e.g. `docs/media/blinc_im_walkthrough.mp4`) and link them here for easy reference.

Demo video (Google Drive): https://drive.google.com/file/d/1j0dNNrlbGKR9Ewa-T64Gk8PHYNKztZfR/view?usp=drive_link

## Development Notes
- Stick to ASCII for edits; avoid reintroducing non-ASCII debug glyphs.
- Shared helper code is intentionally duplicated in legacy scripts; refactor into `src/common/` if you plan to maintain them.
- Use `__pycache__/` in `.gitignore`.

## Troubleshooting
- **Assets missing:** Check paths in `assets/`.
- **Foxglove won't connect:** Confirm port 8765 free, run script from correct venv.
- **Dashboard not responsive:** Verify port 8888; look for tracebacks in terminal.
- **Yaw oscillation erratic:** Inspect `yaw_osc_*` params via `/get`.
- **Blade hits zero always:** Ensure Benewake frustum parameters match rotor geometry.

## Roadmap Ideas
- Convert HTML to templating system (Jinja2 or static file).
- Extract common geometry/pan-tilt helpers into modules.
- Add CLI (`python -m blinc run --mode tracker`) and configuration files.
- Integrate PyTest for key math routines (`aim_pan_tilt_from_world_delta`, `enu_to_wgs84`).
- Provide Dockerfile for repeatable setup.
