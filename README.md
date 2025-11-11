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
- Dashboard theme toggle (light/dark) with persisted preference per browser.
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

## Dashboard Theming
- The Overview/Controls header now exposes a `DARK MODE` toggle button; clicking it swaps the layout between `theme-light` and `theme-dark` instantly.
- Your selection is stored client-side using `localStorage["blincThemePref"]`, so refreshes or tablet visits reuse the last theme per browser.
- All inline SVG diagrams (SP view, schematic, yaw viz) and HUD chips automatically restyle for the active theme—no Python or Foxglove restart required.
- Clear the key or press the button again to revert; the toggle affects only presentation, never telemetry.

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

### Templated Dashboard (Separate Copy)
- A separate Jinja2-based copy of the web dashboard is available for experimentation without touching the main app.
- Files: `src/BLINC_iM_app_templated.py`, templates in `templates/` (`base.html`, `dashboard.html`).
- Install Jinja2 if needed: `pip install jinja2`
- Run: `python src/BLINC_iM_app_templated.py` (defaults to `http://localhost:8890/`).
- Endpoints: `/` (HTML), `/get`, `/set?k=v`, `/state`, `/img/gt`, `/img/px`.
- Note: This is a minimal demo of server-side templating and does not publish Foxglove topics.

### Full App Copy (Separate Folder)
- A separate entrypoint that runs the original app unchanged is provided in `src/full_app_copy/BLINC_iM_app_copy.py`.
- Purpose: gives you an isolated start point without modifying `src/BLINC_iM_app.py`.
- Run: `python src/full_app_copy/BLINC_iM_app_copy.py`
- Behavior: identical to the original (HTTP dashboard on `:8888`, Foxglove publisher on `:8765`).

### Full App Templated (Proxy UI)
- A templated UI that proxies the original app is in `src/full_app_templated/BLINC_iM_app_templated_full.py`.
- It starts the original script in the background and serves a Jinja2 dashboard.
- Default ports: original `:8888` (unchanged), templated UI `:8891`.
- Run: `python src/full_app_templated/BLINC_iM_app_templated_full.py` then open `http://localhost:8891/`.
- Endpoints proxied: `/get`, `/set?k=v`, `/state`. Assets served locally at `/img/gt`, `/img/px`.
- Notes: set `BLINC_ORIG_PORT` or `BLINC_TPL_PORT` env vars to customize ports.

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
