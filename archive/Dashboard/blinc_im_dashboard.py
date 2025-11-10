# blinc_im_dashboard.py
import math, time, datetime, threading, json, os
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

import foxglove
from foxglove.channels import SceneUpdateChannel, FrameTransformsChannel
from foxglove.schemas import (
    Color, CubePrimitive, Duration,
    FrameTransform, FrameTransforms, Pose, Quaternion,
    SceneEntity, SceneUpdate, Timestamp, Vector3,
    TriangleListPrimitive, Point3,
)

# Optional text labels (depends on your foxglove wheel)
try:
    from foxglove.schemas import TextPrimitive
    HAVE_TEXT = True
except Exception:
    HAVE_TEXT = False

# =========================================================
#                   CONFIG / ASSETS
# =========================================================

# ---- Inline assets for header logos (served via /img/gt and /img/px) ----
_ASSETS = {}
_ASSET_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "assets"))
_ASSET_PATHS = {
    "gt":  os.path.join(_ASSET_ROOT, "logo_GTMW.png"),
    "px":  os.path.join(_ASSET_ROOT, "logo_Pixxon.png"),
}
for key, path in _ASSET_PATHS.items():
    try:
        with open(path, "rb") as f:
            _ASSETS[key] = f.read()
    except Exception as e:
        print(f"[WARN] Could not load asset {key} from {path}: {e}")

# =============== CONTROL KNOBS (defaults) ===============
yaw_deg = 0.0
yaw_rate_deg_s = 0.0
rotor_rpm = 12.0
# +1 = CCW about +X,  -1 = CW about +X
rotor_dir = -1
model_number = "WTG-001"

# --- WTG GPS (INPUT AS N/E = LAT/LON). NO ALTITUDE INPUT ---
wtg_lat_deg = 0.0
wtg_lon_deg = 0.0

# SCANNER POSITION SELECTOR (SP1 / SP2 / SP3 / SP4)
scanner_sp = "SP1"

# SPs start rotated +270° at yaw=0 (saved preference)
sp_yaw_offset_deg = 270.0

# --- Rotor / WTG geometry ---
rotor_dia_m        = 80.0
hub_ht_m           = 80.0
theta_deg_val      = 10.0  # internal

# Blade styling (under Blade Parameters)
blade_thickness_m  = 0.25
blade_root_w_m     = 1.6
blade_tip_w_m      = 0.40

# Targeting: aim at hub center + optional vertical offset (meters)
hub_target_z_offset_m = 20.0

# Ground circle ring
circle_ring_width_m = 0.6
circle_segments     = 256
circle_color        = Color(r=0.10, g=0.45, b=0.12, a=0.9)

# SP bullseye styling
marker_inner_r_m = 0.45
marker_outer_r_m = 0.90
marker_ring_w_m  = 0.10
marker_segments  = 96
marker_color_red = Color(r=1.0, g=0.0, b=0.0, a=1.0)

# Labels (optional)
label_height_m   = 2.0
label_offset_m   = 1.0
label_font_size  = 1.0
label_color      = Color(r=0.0, g=0.0, b=0.0, a=1.0)
label_bg_color   = Color(r=1.0, g=1.0, b=1.0, a=0.9)
label_billboard  = True

# TRACKER marker & mast
tracker_color_blue     = Color(r=0.0, g=0.35, b=1.0, a=1.0)
tracker_ht_m           = 1.0
tracker_mast_radius_m  = 0.30
tracker_mast_color     = Color(r=0.0, g=0.0, b=0.0, a=1.0)

# --- Sensors at TRACKER top (same position/orientation) ---
# Mechanical stack: PAN -> (offset) -> TILT -> (offset) -> SENSOR (+X forward)
pan_to_tilt_offset    = (0.0, 0.05, 0.01)  # (x,y,z) in meters
tilt_to_sensor_offset = (0.0, 0.00, 0.01)

# Benewake (narrow; dynamic to hub)  — stays on TRACKER mast
benewake_haov_deg   = 0.35
benewake_vaov_deg   = 0.15
benewake_near_m     = 0.20
benewake_color      = Color(r=1.0, g=0.0, b=0.0, a=0.45)

# LIVOX (as part of SCANNER; moves to SPx)
livox_haov_deg      = 70.4
livox_vaov_deg      = 74.2
livox_near_m        = 0.20
livox_far_m         = 130.0
livox_color         = Color(r=0.0, g=0.0, b=0.1, a=0.02)

# --- S-Laser + Camera (as part of SCANNER; move to SPx) ---
slaser_haov_deg     = 0.35
slaser_vaov_deg     = 0.15
slaser_near_m       = 0.20
slaser_far_m        = 150.0
slaser_color        = Color(r=1.0, g=0.4, b=0.0, a=0.45)

camera_haov_deg     = 2.8547
camera_vaov_deg     = 2.1388
camera_near_m       = 0.20
camera_far_m        = 150.0
camera_color        = Color(r=1.0, g=1.0, b=1.0, a=0.12)
camera_mask_alpha   = 0.80
camera_mask_pad     = 25.0
camera_mask_dist    = 0.12

# 3D panel suggestion (printed once)
PANEL_PRESET_DISTANCE = 14.0  # meters

# Frame names
WORLD, NACELLE, HUB, SCAN = "world", "nacelle", "hub", "scan_ground"
B1, B2, B3 = "blade_1", "blade_2", "blade_3"
# Tracker (Benewake) sensor chain
B_PAN, B_TILT, B_SENSOR = "benewake_pan", "benewake_tilt", "benewake_sensor"
# Scanner chains (these move to SPx)
L_PAN, L_TILT, L_SENSOR = "livox_pan", "livox_tilt", "livox_sensor"
S_PAN, S_TILT, S_SENSOR = "scanner_slaser_pan", "scanner_slaser_tilt", "scanner_slaser_sensor"
C_SENSOR = "scanner_camera_sensor"   # same pose as S_SENSOR

# ========= param locking & timing for web panel =========
_PARAM_LOCK = threading.Lock()
_START_TIME = time.time()

# ========= blade counter state =========
HIT_VISUAL_GAIN = 25.0
_blade_active = {"B1": False, "B2": False, "B3": False}
_blade_hits = {"B1": 0, "B2": 0, "B3": 0, "total": 0}

# ========= pan/tilt values for dashboard =========
_last_tr_pan_deg = 0.0
_last_tr_tilt_deg = 0.0
_last_sc_pan_deg = 0.0
_last_sc_tilt_deg = 0.0

# --- Yaw oscillation controls ---
yaw_osc_mode = "OFF"        # OFF, SINE, RANDOM
yaw_osc_rate_deg_s = 10.0   # max slew rate contribution (deg/s)
yaw_osc_max_abs_deg = 30.0  # oscillation bound (±deg)
yaw_osc_freq_hz = 0.08      # for SINE (≈12.5 s period)
yaw_osc_random_step_deg = 5.0  # for RANDOM

# Internal osc state
_osc_angle_deg = 0.0
_osc_phase = 0.0
_last_loop_ts = _START_TIME

# ===================== UTILITIES =====================
def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0

def quat_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    r, p, y = roll*0.5, pitch*0.5, yaw*0.5
    sr, cr = math.sin(r), math.cos(r)
    sp, cp = math.sin(p), math.cos(p)
    sy, cy = math.sin(y), math.cos(y)
    return Quaternion(
        x=sr*cp*cy - cr*sp*sy,
        y=cr*sp*cy + sr*cp*sy,
        z=cr*cp*sy - sr*sp*cy,
        w=cr*cp*cy + sr*cp*sy,
    )

def now_ts() -> Timestamp:
    return Timestamp.from_datetime(datetime.datetime.utcnow())

def print_sp1_panel_preset():
    D = PANEL_PRESET_DISTANCE
    print("\n=== FOXGLOVE 3D PRESET FOR FRAME 'SCANNER_CAMERA_SENSOR' ===")
    print("DISPLAY FRAME: SCANNER_CAMERA_SENSOR   |   3D VIEW: ON")
    print(f"OPTION A (LOOK ALONG +X): DISTANCE={D:.3f}, TARGET X={D:.3f}, TARGET Y=0, TARGET Z=0, THETA=180, PHI=90")
    print(f"OPTION B (THETA=90, PHI=90):       DISTANCE={D:.3f}, TARGET X=0, TARGET Y={D:.3f}, TARGET Z=0, THETA=90, PHI=90\n")

# ===================== GEOMETRY HELPERS =====================
def make_filled_disk_tris(cx: float, cy: float, radius: float, segments: int, z=0.0):
    tris = []; center = Point3(x=cx, y=cy, z=z); ring=[]
    for i in range(segments):
        th = 2.0*math.pi*i/segments
        ring.append(Point3(x=cx+radius*math.cos(th), y=cy+radius*math.sin(th), z=z))
    for i in range(segments):
        j=(i+1)%segments
        tris += [center, ring[i], ring[j]]
    return tris

def make_circle_ring_tris(cx: float, cy: float, r_outer: float, ring_w: float, segments: int, z=0.0):
    r_out = max(r_outer, 1e-3)
    r_in  = max(r_outer - max(ring_w,1e-3), 1e-3)
    out_ring, in_ring = [], []
    for i in range(segments):
        th = 2.0*math.pi*i/segments
        c, s = math.cos(th), math.sin(th)
        out_ring.append(Point3(x=cx+r_out*c, y=cy+r_out*s, z=z))
        in_ring.append (Point3(x=cx+r_in *c, y=cy+r_in *s, z=z))
    tris=[]
    for i in range(segments):
        j=(i+1)%segments
        oi, oj = out_ring[i], out_ring[j]
        ii, ij = in_ring[i],  in_ring[j]
        tris += [oi, oj, ij]; tris += [oi, ij, ii]
    return tris

def make_ground_circle_ring(frame_id: str, entity_id: str,
                            radius: float, ring_width: float, segments: int,
                            color: Color, z_offset=0.01) -> SceneEntity:
    tris = make_circle_ring_tris(0.0,0.0,radius, ring_width, segments, z=z_offset)
    return SceneEntity(frame_id=frame_id, id=entity_id, timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=tris, color=color)])

def make_cylinder_mesh_Z(frame_id, entity_id, radius, height, segments=64,
                         color=Color(r=0.85, g=0.85, b=0.9, a=1.0)) -> SceneEntity:
    top_z, bot_z = height, 0.0
    verts_top, verts_bot = [], []
    for i in range(segments):
        th = 2.0 * math.pi * i / segments
        x = radius * math.cos(th); y = radius * math.sin(th)
        verts_top.append(Point3(x=x, y=y, z=top_z))
        verts_bot.append(Point3(x=x, y=y, z=bot_z))
    top_center = Point3(x=0.0, y=0.0, z=top_z)
    bot_center = Point3(x=0.0, y=0.0, z=bot_z)
    tris = []
    for i in range(segments):
        j = (i + 1) % segments
        bi, bj = verts_bot[i], verts_bot[j]
        ti, tj = verts_top[i], verts_top[j]
        tris += [bi, bj, tj]; tris += [bi, tj, ti]
    for i in range(segments):
        j = (i + 1) % segments
        ti, tj = verts_top[i], verts_top[j]
        tris += [top_center, ti, tj]
    for i in range(segments):
        j = (i + 1) % segments
        bi, bj = verts_bot[i], verts_bot[j]
        tris += [bot_center, bj, bi]
    return SceneEntity(frame_id=frame_id, id=entity_id, timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=tris, color=color)])

def make_cylinder_mesh_Z_at(frame_id, entity_id, radius, height, cx, cy,
                            segments=64, color=Color(r=0, g=0, b=0, a=1.0)) -> SceneEntity:
    top_z, bot_z = height, 0.0
    verts_top, verts_bot = [], []
    for i in range(segments):
        th = 2.0*math.pi*i/segments
        x = cx + radius*math.cos(th)
        y = cy + radius*math.sin(th)
        verts_top.append(Point3(x=x, y=y, z=top_z))
        verts_bot.append(Point3(x=x, y=y, z=bot_z))
    top_center = Point3(x=cx, y=cy, z=top_z)
    bot_center = Point3(x=cx, y=cy, z=bot_z)
    tris=[]
    for i in range(segments):
        j=(i+1)%segments
        bi, bj = verts_bot[i], verts_bot[j]
        ti, tj = verts_top[i], verts_top[j]
        tris += [bi, bj, tj]; tris += [bi, tj, ti]
    for i in range(segments):
        j=(i+1)%segments
        ti, tj = verts_top[i], verts_top[j]
        tris += [top_center, ti, tj]
    for i in range(segments):
        j=(i+1)%segments
        bi, bj = verts_bot[i], verts_bot[j]
        tris += [bot_center, bj, bi]
    return SceneEntity(frame_id=frame_id, id=entity_id, timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=tris, color=color)])

def make_cylinder_mesh_X(frame_id, entity_id, radius, length, segments=48,
                         color=Color(r=0.9, g=0.9, b=0.95, a=1.0)) -> SceneEntity:
    xL, xR = -length/2.0, +length/2.0
    ring_L, ring_R = [], []
    for i in range(segments):
        th = 2.0*math.pi*i/segments
        y = radius*math.cos(th); z = radius*math.sin(th)
        ring_L.append(Point3(x=xL, y=y, z=z))
        ring_R.append(Point3(x=xR, y=y, z=z))
    center_L = Point3(x=xL, y=0.0, z=0.0)
    center_R = Point3(x=xR, y=0.0, z=0.0)
    tris=[]
    for i in range(segments):
        j=(i+1)%segments
        li, lj = ring_L[i], ring_L[j]
        ri, rj = ring_R[i], ring_R[j]
        tris += [li, lj, rj]; tris += [li, rj, ri]
    for i in range(segments):
        j=(i+1)%segments
        ri, rj = ring_R[i], ring_R[j]
        tris += [center_R, ri, rj]
    for i in range(segments):
        j=(i+1)%segments
        li, lj = ring_L[i], ring_L[j]
        tris += [center_L, lj, li]
    return SceneEntity(frame_id=frame_id, id=entity_id, timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=tris, color=color)])

def make_cone_mesh_X(frame_id, entity_id, base_radius, length, segments=48,
                     color=Color(r=0.88, g=0.9, b=0.95, a=1.0)) -> SceneEntity:
    x_base, x_tip = 0.0, length
    ring=[]
    for i in range(segments):
        th = 2.0*math.pi*i/segments
        y = base_radius*math.cos(th); z = base_radius*math.sin(th)
        ring.append(Point3(x=x_base, y=y, z=z))
    center_base = Point3(x=x_base, y=0.0, z=0.0)
    tip = Point3(x=x_tip, y=0.0, z=0.0)
    tris=[]
    for i in range(segments):
        j=(i+1)%segments
        vi, vj = ring[i], ring[j]
        tris += [vi, vj, tip]
    for i in range(segments):
        j=(i+1)%segments
        vi, vj = ring[i], ring[j]
        tris += [center_base, vj, vi]
    return SceneEntity(frame_id=frame_id, id=entity_id, timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=tris, color=color)])

def make_blade_entity(frame_id: str, entity_id: str,
                      Lz=18.0, root_w=1.2, tip_w=0.3, thickness=0.15,
                      color=Color(r=0.95, g=0.2, b=0.2, a=1.0)) -> SceneEntity:
    hx = thickness / 2.0
    A  = Point3(x=+hx, y=-root_w/2, z=0.0)
    B  = Point3(x=+hx, y=-tip_w/2,  z=Lz)
    C  = Point3(x=+hx, y=+tip_w/2,  z=Lz)
    D  = Point3(x=+hx, y=+root_w/2, z=0.0)
    A2 = Point3(x=-hx, y=-root_w/2, z=0.0)
    B2 = Point3(x=-hx, y=-tip_w/2,  z=Lz)
    C2 = Point3(x=-hx, y=+tip_w/2,  z=Lz)
    D2 = Point3(x=-hx, y=+root_w/2, z=0.0)
    tris = [
        A,B,C,  A,C,D,
        A2,C2,B2,  A2,D2,C2,
        A,A2,B2,  A,B2,B,
        B,B2,C2,  B,C2,C,
        C,C2,D2,  C,D2,D,
        D,D2,A2,  D,A2,A,
    ]
    return SceneEntity(frame_id=frame_id, id=entity_id, timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=tris, color=color)])

def make_fov_frustum_X(frame_id: str, entity_id: str,
                       near_m: float, far_m: float,
                       haov_deg: float, vaov_deg: float,
                       color: Color) -> SceneEntity:
    near_m = max(near_m, 1e-3)
    far_m  = max(far_m, near_m+1e-3)
    h2 = math.radians(haov_deg*0.5); v2 = math.radians(vaov_deg*0.5)
    y_n, z_n = near_m*math.tan(h2), near_m*math.tan(v2)
    y_f, z_f =  far_m*math.tan(h2),  far_m*math.tan(v2)
    n1=Point3(x=near_m, y=-y_n, z=-z_n); n2=Point3(x=near_m, y= y_n, z=-z_n)
    n3=Point3(x=near_m, y= y_n, z= z_n); n4=Point3(x=near_m, y=-y_n, z= z_n)
    f1=Point3(x= far_m, y=-y_f, z=-z_f); f2=Point3(x= far_m, y= y_f, z=-z_f)
    f3=Point3(x= far_m, y= y_f, z= z_f); f4=Point3(x= far_m, y=-y_f, z= z_f)
    tris=[]
    tris += [n1,n2,n3]; tris += [n1,n3,n4]
    tris += [f1,f3,f2]; tris += [f1,f4,f3]
    tris += [n1,f2,f1]; tris += [n1,n2,f2]
    tris += [n2,f3,f2]; tris += [n2,n3,f3]
    tris += [n3,f4,f3]; tris += [n3,n4,f4]
    tris += [n4,f1,f4]; tris += [n4,n1,f1]
    return SceneEntity(frame_id=frame_id, id=entity_id, timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=tris, color=color)])

def make_camera_view_mask(frame_id: str, entity_id: str,
                          haov_deg: float, vaov_deg: float,
                          dist_m: float, pad_scale: float,
                          alpha: float = 0.8) -> SceneEntity:
    h2 = math.radians(haov_deg*0.5); v2 = math.radians(vaov_deg*0.5)
    wy = dist_m * math.tan(h2)
    wz = dist_m * math.tan(v2)
    pad = (wy + wz) * 0.5 * pad_scale
    def quad(y0, z0, y1, z1):
        p1 = Point3(x=dist_m, y=y0, z=z0)
        p2 = Point3(x=dist_m, y=y1, z=z0)
        p3 = Point3(x=dist_m, y= y1, z= z1)
        p4 = Point3(x=dist_m, y= y0, z= z1)
        return [p1,p2,p3, p1,p3,p4]
    tris = []
    tris += quad(-pad, -pad, -wy, +pad)
    tris += quad(+wy, -pad, +pad, +pad)
    tris += quad(-wy, -pad, +wy, -wz)
    tris += quad(-wy, +wz, +wy, +pad)
    return SceneEntity(
        frame_id=frame_id, id=entity_id, timestamp=now_ts(),
        lifetime=Duration.from_secs(0),
        triangles=[TriangleListPrimitive(points=tris,
                                         color=Color(r=0.0, g=0.0, b=0.0, a=alpha))]
    )

def make_view_anchor_entity(frame_id: str, entity_id: str, dist_m: float,
                            color: Color = Color(r=1.0, g=1.0, b=0.0, a=0.95)) -> SceneEntity:
    s = 0.15
    return SceneEntity(
        frame_id=frame_id, id=entity_id, timestamp=now_ts(),
        lifetime=Duration.from_secs(0),
        cubes=[CubePrimitive(
            pose=Pose(position=Vector3(x=dist_m, y=0.0, z=0.0),
                      orientation=Quaternion(x=0, y=0, z=0, w=1)),
            size=Vector3(x=s, y=s, z=s),
            color=color
        )],
    )

# ===================== SP MARKERS =====================
def _sp_local_points():
    theta_rad = math.radians(theta_deg_val)
    x = rotor_dia_m / 4.0
    y_R = rotor_dia_m * 1.125
    y_T = hub_ht_m * math.tan(theta_rad)
    return {
        "SP1": (-x, +y_R),
        "SP2": (-x, +y_T),
        "SP3": (-x, -y_R),
        "SP4": (+x, -y_T),
    }

def _rotate_to_world(x_local, y_local, yaw_rad):
    sp_rot = yaw_rad + math.pi/2 + math.radians(sp_yaw_offset_deg)
    c, s = math.cos(sp_rot), math.sin(sp_rot)
    return (x_local*c - y_local*s, x_local*s + y_local*c)

def make_bullseye_markers_with_labels(frame_id: str, entity_id: str) -> SceneEntity:
    pts_local = _sp_local_points()
    all_tris = []
    for (px, py) in pts_local.values():
        all_tris += make_filled_disk_tris(px, py, marker_inner_r_m, marker_segments)
        all_tris += make_circle_ring_tris(px, py, marker_outer_r_m, marker_ring_w_m, marker_segments)
    entity_kwargs = dict(
        frame_id=frame_id, id=entity_id, timestamp=now_ts(),
        lifetime=Duration.from_secs(0),
        triangles=[TriangleListPrimitive(points=all_tris, color=marker_color_red)],
    )
    if HAVE_TEXT:
        texts = []
        for name, (px, py) in pts_local.items():
            r = max(math.hypot(px, py), 1e-6)
            ux, uy = px / r, py / r
            lx, ly = px + ux * label_offset_m, py + uy * label_offset_m
            text_kwargs = dict(
                pose=Pose(position=Vector3(x=lx, y=ly, z=label_height_m),
                          orientation=Quaternion(x=0, y=0, z=0, w=1)),
                text=name, font_size=label_font_size, color=label_color,
            )
            try:
                texts.append(TextPrimitive(**text_kwargs,
                                           billboard=label_billboard,
                                           background_color=label_bg_color))
            except TypeError:
                try:
                    texts.append(TextPrimitive(**text_kwargs,
                                               billboard=label_billboard,
                                               background=True, bg_color=label_bg_color))
                except TypeError:
                    texts.append(TextPrimitive(**text_kwargs, billboard=label_billboard))
        entity_kwargs["texts"] = texts
    return SceneEntity(**entity_kwargs)

# ===================== FRAMES / TRANSFORMS =====================
def publish_world_root(tf_ch):
    tf_ch.log(FrameTransforms(transforms=[
        FrameTransform(parent_frame_id="root", child_frame_id=WORLD,
                       translation=Vector3(x=0, y=0, z=0),
                       rotation=Quaternion(x=0,y=0,z=0,w=1)),
    ]))

# --- OLD (replaced): publish_wtg_frames removed in favor of split publishers ---

# NEW: SCAN frame uses steady yaw (no oscillation)
def publish_scan_frame(tf_ch, sp_yaw_rad: float):
    sp_rot = sp_yaw_rad + math.pi/2 + math.radians(sp_yaw_offset_deg)
    tf_ch.log(FrameTransforms(transforms=[
        FrameTransform(parent_frame_id=WORLD, child_frame_id=SCAN,
                       translation=Vector3(x=0, y=0, z=0),
                       rotation=quat_from_euler(0.0, 0.0, sp_rot)),
    ]))

# NEW: Nacelle/Hub use oscillating yaw
def publish_nacelle_frames(tf_ch, yaw_rad: float, tower_h: float, nacelle_l: float):
    nacelle_rot = yaw_rad + math.pi/2
    tf_ch.log(FrameTransforms(transforms=[
        FrameTransform(parent_frame_id=WORLD, child_frame_id=NACELLE,
                       translation=Vector3(x=0, y=0, z=tower_h),
                       rotation=quat_from_euler(0.0, 0.0, nacelle_rot)),
        FrameTransform(parent_frame_id=NACELLE, child_frame_id=HUB,
                       translation=Vector3(x=0.6*nacelle_l, y=0.0, z=0.0),
                       rotation=Quaternion(x=0,y=0,z=0,w=1)),
    ]))

def hub_center_world(yaw_rad: float, nacelle_l: float):
    nacelle_rot = yaw_rad + math.pi/2
    hx_local, hy_local = (0.6*nacelle_l, 0.0)
    hub_x =  hx_local*math.cos(nacelle_rot) - hy_local*math.sin(nacelle_rot)
    hub_y =  hx_local*math.sin(nacelle_rot) + hy_local*math.cos(nacelle_rot)
    hub_z =  hub_ht_m
    return hub_x, hub_y, hub_z

def sp_xy_world_all(yaw_rad: float):
    sp_local = _sp_local_points()
    sp_world = {}
    for name, (lx, ly) in sp_local.items():
        sp_world[name] = (*_rotate_to_world(lx, ly, yaw_rad), 0.0)
    return sp_world

# ===================== SENSOR AIM =====================
def aim_pan_tilt_from_world_delta(dx, dy, dz):
    psi   = math.atan2(dy, dx)
    theta = math.atan2(dz, math.hypot(dx,dy))
    pan   = psi + math.pi/2
    tilt  = -theta
    return pan, tilt

def sensor_origin_with_offsets(pan, tilt):
    p2t_x, p2t_y, p2t_z = pan_to_tilt_offset
    t2s_x, t2s_y, t2s_z = tilt_to_sensor_offset
    c, s = math.cos(pan), math.sin(pan)
    cy, sy = math.cos(tilt), math.sin(tilt)
    ox = p2t_x*c - p2t_y*s
    oy = p2t_x*s + p2t_y*c
    oz = p2t_z
    t2sx =  t2s_x*cy + t2s_z*sy
    t2sz = -t2s_x*sy + t2s_z*cy
    t2sy =  t2s_y
    ox += t2sx*c - t2sy*s
    oy += t2sx*s + t2sy*c
    oz += t2sz
    return ox, oy, oz

def solve_pan_tilt_to_target(pan_base_world, target_world):
    bx, by, bz = pan_base_world
    tx, ty, tz = target_world
    pan, tilt = aim_pan_tilt_from_world_delta(tx - bx, ty - by, tz - bz)
    for _ in range(2):
        ox, oy, oz = sensor_origin_with_offsets(pan, tilt)
        sx, sy, sz = bx + ox, by + oy, bz + oz
        pan, tilt = aim_pan_tilt_from_world_delta(tx - sx, ty - sy, tz - sz)
    return pan, tilt

# ========== small linear algebra for FOV test ==========
def rotZ(a):
    c, s = math.cos(a), math.sin(a)
    return ((c,-s,0.0),(s,c,0.0),(0.0,0.0,1.0))
def rotY(a):
    c, s = math.cos(a), math.sin(a)
    return ((c,0.0,s),(0.0,1.0,0.0),(-s,0.0,c))
def matmul3(A,B):
    return tuple(tuple(sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)) for i in range(3))

# ===================== SCENE BUILD =====================
def build_scene_entities():
    tower_h = hub_ht_m
    blade_len = rotor_dia_m / 2.0
    x_local_sp1 = -rotor_dia_m/4.0
    y_local_sp1 = +rotor_dia_m*1.125

    entities = [
        make_cylinder_mesh_Z(WORLD, "tower_mesh",
                             radius=(2.5/2.0)*0.5, height=tower_h, segments=64,
                             color=Color(r=0.85, g=0.85, b=0.9, a=1.0)),
        SceneEntity(frame_id=NACELLE, id="nacelle_geom", timestamp=now_ts(),
                    lifetime=Duration.from_secs(0),
                    cubes=[CubePrimitive(
                        pose=Pose(position=Vector3(x=0,y=0,z=0),
                                  orientation=Quaternion(x=0,y=0,z=0,w=1)),
                        size=Vector3(x=3.6, y=1.8, z=1.2),
                        color=Color(r=0.2,g=0.6,b=0.9,a=1.0)
                    )]),
        make_cylinder_mesh_X(HUB, "hub_mesh", radius=1.2/2.0, length=0.8, segments=48,
                             color=Color(r=0.92,g=0.92,b=0.95,a=1.0)),
        make_cone_mesh_X(HUB, "nose_cone_mesh", base_radius=(1.2/2.0)*0.6, length=1.0, segments=48,
                         color=Color(r=0.9, g=0.9, b=0.97, a=1.0)),
        make_blade_entity("blade_1","blade1",
                          Lz=blade_len, root_w=blade_root_w_m, tip_w=blade_tip_w_m,
                          thickness=blade_thickness_m, color=Color(r=0.95,g=0.2,b=0.2,a=1.0)),
        make_blade_entity("blade_2","blade2",
                          Lz=blade_len, root_w=blade_root_w_m, tip_w=blade_tip_w_m,
                          thickness=blade_thickness_m, color=Color(r=0.2,g=0.95,b=0.2,a=1.0)),
        make_blade_entity("blade_3","blade3",
                          Lz=blade_len, root_w=blade_root_w_m, tip_w=blade_tip_w_m,
                          thickness=blade_thickness_m, color=Color(r=0.2, g=0.2, b=0.95, a=1.0)),
        make_ground_circle_ring(WORLD, "ground_circle",
                                radius=1.5 * rotor_dia_m,
                                ring_width=circle_ring_width_m, segments=circle_segments,
                                color=circle_color, z_offset=0.01),
        SceneEntity(
            frame_id=WORLD, id="tracker_marker", timestamp=now_ts(),
            lifetime=Duration.from_secs(0),
            triangles=[TriangleListPrimitive(
                points=make_filled_disk_tris(0.0, rotor_dia_m*1.125, marker_outer_r_m, marker_segments, z=0.0),
                color=tracker_color_blue
            )],
        ),
        make_cylinder_mesh_Z_at(WORLD, "tracker_mast",
                                radius=tracker_mast_radius_m, height=tracker_ht_m,
                                cx=0.0, cy=rotor_dia_m*1.125, segments=48, color=tracker_mast_color),
        make_bullseye_markers_with_labels(SCAN, "sp_markers_bullseye"),
        make_cylinder_mesh_Z_at(SCAN, "sp1_mast",
                                radius=tracker_mast_radius_m, height=tracker_ht_m,
                                cx=x_local_sp1, cy=y_local_sp1, segments=48, color=tracker_mast_color),
    ]
    return entities, hub_ht_m, 3.6, 1.0

# ===================== ENU (METERS) -> GPS =====================
def enu_to_wgs84(east_m: float, north_m: float, up_m: float,
                 lat0_deg: float, lon0_deg: float, alt0_m: float=0.0):
    R = 6378137.0
    lat0 = math.radians(lat0_deg)
    dlat = north_m / R
    dlon = east_m / (R * max(1e-9, math.cos(lat0)))
    lat = lat0 + dlat
    lon = math.radians(lon0_deg) + dlon
    return (math.degrees(lat), math.degrees(lon), alt0_m + up_m)

# ===================== Live state for tables =====================
def compute_live_state():
    with _PARAM_LOCK:
        rpm = rotor_rpm
        dia = rotor_dia_m
        hubz = hub_ht_m
        model = model_number
        wtg_lat = wtg_lat_deg
        wtg_lon = wtg_lon_deg
        which = scanner_sp
        active = dict(_blade_active)
        hits = dict(_blade_hits)
        tr_pan_d = _last_tr_pan_deg
        tr_tilt_d = _last_tr_tilt_deg
        sc_pan_d = _last_sc_pan_deg
        sc_tilt_d = _last_sc_tilt_deg
        dir_sign = rotor_dir

    now = time.time()
    t = now - _START_TIME

    # Oscillating yaw (drives nacelle/hub)
    yaw_cmd_deg = yaw_deg + yaw_rate_deg_s * t + _osc_angle_deg
    yaw_cmd_rad = math.radians(yaw_cmd_deg)

    # Steady yaw for SCAN/SP (no oscillation)
    sp_yaw_deg = yaw_deg + yaw_rate_deg_s * t
    sp_yaw_rad = math.radians(sp_yaw_deg)

    hub_x, hub_y, hub_z = hub_center_world(yaw_cmd_rad, 3.6)

    tracker_base = (0.0, dia*1.125, tracker_ht_m)
    system_height_m = hubz + dia/2.0  # blade-tip height

    def d3(a): return float(f"{a:.3f}")

    turbine_gps = enu_to_wgs84(hub_x, hub_y, hub_z, wtg_lat, wtg_lon, 0.0)[:2]
    tracker_gps = enu_to_wgs84(*tracker_base, wtg_lat, wtg_lon, 0.0)[:2]
    sp_world = sp_xy_world_all(sp_yaw_rad)  # decoupled from oscillation
    sp_gps = {k: enu_to_wgs84(v[0], v[1], 0.0, wtg_lat, wtg_lon, 0.0)[:2] for k, v in sp_world.items()}

    return {
        "wtg_info": {
            "model_number": model,
            "rotor_dia_m": d3(dia),
            "hub_ht_m": d3(hubz),
            "system_height_m": d3(system_height_m),
        },
        "status": {
            "rotor_rpm": d3(rpm),
            "current_yaw_deg": d3(wrap_deg(yaw_cmd_deg)),
            "rotor_dir": "CCW" if dir_sign >= 0 else "CW",
        },
        "coordinates_cartesian": {
            "turbine_xyz_m": (d3(hub_x), d3(hub_y), d3(hub_z)),
            "tracker_xyz_m": tuple(map(d3, tracker_base)),
            "SP1_xyz_m": tuple(map(d3, sp_world["SP1"])),
            "SP2_xyz_m": tuple(map(d3, sp_world["SP2"])),
            "SP3_xyz_m": tuple(map(d3, sp_world["SP3"])),
            "SP4_xyz_m": tuple(map(d3, sp_world["SP4"])),
        },
        "coordinates_gps": {
            "turbine_latlon": (d3(turbine_gps[0]), d3(turbine_gps[1])),
            "tracker_latlon": (d3(tracker_gps[0]), d3(tracker_gps[1])),
            "SP1_latlon": (d3(sp_gps["SP1"][0]), d3(sp_gps["SP1"][1])),
            "SP2_latlon": (d3(sp_gps["SP2"][0]), d3(sp_gps["SP2"][1])),
            "SP3_latlon": (d3(sp_gps["SP3"][0]), d3(sp_gps["SP3"][1])),
            "SP4_latlon": (d3(sp_gps["SP4"][0]), d3(sp_gps["SP4"][1])),
        },
        "pt": {
            "scanner": {"pan_deg": d3(sc_pan_d), "tilt_deg": d3(sc_tilt_d)},
            "tracker": {"pan_deg": d3(tr_pan_d), "tilt_deg": d3(tr_tilt_d)},
        },
        "wtg_lat": f"{wtg_lat:.6f}",
        "wtg_lon": f"{wtg_lon:.6f}",
        "scanner_sp": which,
        "blade": {"active": active, "hits": hits},
    }

# ===================== Web Dashboard (HTML/JS) =====================
_HTML = """<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<title>BLINC-IM – BLADE INSPECTION IN MOTION</title>
<style>
 :root {
   --card-bg:#ffffff; --muted:#555; --border:#e2e2e2; --shadow:0 8px 24px rgba(0,0,0,.08);
   --banner-bg: linear-gradient(180deg, #f6f8fb, #eef2f7);
 }
 * { text-transform: uppercase; box-sizing: border-box; }
 body { font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial; margin: 18px; background:#f4f5f7; }

 /* Header bar */
 .hero { background:#5a5a5a; color:#fff; padding:12px 20px; border-radius:10px; box-shadow: var(--shadow); margin-bottom:16px; }
 .header-bar { display:flex; align-items:center; justify-content:space-between; }
 .header-left { font-size:26px; font-weight:800; letter-spacing:.4px; }
 .header-right { display:flex; align-items:center; gap:16px; }
 .logo { height:32px; object-fit:contain; display:block; }
 .greentech { height:30px; }
 .pixxon { height:30px; }
 .divider { width:1px; height:28px; background:rgba(255,255,255,.6); }

 .card { background:var(--card-bg); padding: 12px; border:1px solid var(--border); border-radius:12px; box-shadow: var(--shadow); }
 .banner { background: var(--banner-bg); border:1px solid var(--border); padding:8px 10px; border-radius:8px; margin: -2px -2px 12px; font-size: 14px; font-weight:800; letter-spacing:.4px; }

 /* Mild green for Initial Inputs card */
 .initial-card { background:#e8f5e9; border-color:#cfe9d4; }
 .initial-card .banner { background: linear-gradient(180deg, #eaf7ee, #d9f0e0); border-color:#cfe9d4; }

 .subbanner { background:#f8fafc; border:1px dashed var(--border); padding:6px 8px; border-radius:8px; margin: 6px 0 8px; font-size: 12px; font-weight:800; letter-spacing:.3px; }
 .grid { display:grid; grid-template-columns: 200px 1fr 110px; gap: 10px 12px; align-items:center; }
 .rowlabel { font-weight: 700; font-size: 12px; color:#333; }
 input[type=number], input[type=text], select { padding:6px 8px; border:1px solid var(--border); border-radius:8px; background:#fff; width:100%; max-width:100%; }
 input[type=range] { width: 100%; }
 button { padding:8px 12px; border:1px solid var(--border); border-radius:8px; background:#fff; cursor:pointer; }
 .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }

 table { border-collapse: collapse; width: 100%; }
 th, td { border: 1px solid #ddd; padding: 6px 10px; text-align: left; font-size: 12px; }
 th { background: #f5f5f5; }
 .section { background:#fafafa; font-weight:800; }
 .row-top { display:grid; gap:16px; }
 .row-bottom { display:grid; gap:16px; }
 .row-mid { display:grid; gap:16px; align-items:start; }
 @media (min-width: 1200px){
   .row-top   { grid-template-columns: 360px 1fr 1fr; }
   .row-bottom{ grid-template-columns: 1fr 1fr 1fr 1fr; }
   .row-mid { grid-template-columns: auto auto auto; justify-content: start; }
 }
 .initGrid  { display:grid; grid-template-columns: minmax(120px, 1fr) minmax(120px, 1.2fr); gap: 8px 10px; align-items:center; }
 .bladeGrid { display:grid; grid-template-columns: minmax(180px, 1fr) minmax(120px, 1fr); gap: 10px 12px; align-items:center; margin-top:8px; }

 details.fold { border:1px solid var(--border); border-radius:10px; padding:8px 10px; background:#fff; margin-top:12px; }
 details.fold > summary { cursor:pointer; font-weight:800; list-style:none; padding:6px 8px; background:var(--banner-bg); border:1px solid var(--border); border-radius:8px; }
 details.fold > summary::-webkit-details-marker { display:none; }
 .muted { color: var(--muted); font-size:11px; }

 /* Blade counter (styles condensed) */
 .blade-card{ width:152px; border:2px solid #111; border-radius:14px; overflow:hidden; background:#fff; }
 .blade-head{ padding:13px 16px; font-weight:800; border-bottom:2px solid #111; letter-spacing:.4px; font-size:13px; text-align:left; }
 .blade-body{ padding:18px 12px; border-bottom:2px solid #111; display:flex; flex-direction:column; gap:22px; }
 .blade-row{ display:grid; grid-template-columns: 64px 1fr; align-items:center; }
 .dot{ width:48px; height:48px; border-radius:50%; border:2px solid rgba(0,0,0,.35); display:flex; align-items:center; justify-content:center; font-weight:900; font-size:16px; color:#111; transform: translateX(2px); }
 .dot-red{background:#b71c1c;} .dot-green{background:#9ad18b;} .dot-blue{background:#98b9d9;}
 .dot.idle{filter:brightness(.75);} .dot.active{animation:bladeFlash .25s ease; filter:brightness(1.2);}
 @keyframes bladeFlash { from{box-shadow:0 0 0 0 rgba(0,0,0,.25);} to{box-shadow:0 0 18px 6px rgba(0,0,0,0);} }
 .countBox{ justify-self:start; margin-left:-35px; min-width:44px; padding:7px 25px; text-align:right; border:2px solid #111; border-radius:10px; font-weight:700; font-size:16px; background:#fff; }
 .count-total{ min-width:18px; font-size:18px; }
 .blade-foot{ padding:10px 12px 12px; display:flex; align-items:center; justify-content:space-between; gap:10px; }
 .foot-title{ font-weight:800; font-size:13px; letter-spacing:.4px; }

 /* Schematic */
 .wtg-card{ width: 580; }
 @media (max-width: 1200px){ .wtg-card{ width: 100%; } }
 .svgWrap { display:flex; justify-content:center; }
 svg.schem { width:100%; max-width:260px; height:auto; }
 svg.schem .thin { stroke:#111; stroke-width:2; fill:none; }
 svg.schem .dash { stroke-dasharray:8 8; }
 svg.schem text { font-weight:800; font-size:14px; }
 svg.schem #txtDia, svg.schem #txtHub { font-size:28px; }
 svg.schem #txtLoc { font-size:28px; }

 /* Yaw Orientation card */
 .yaw-card .svgYawWrap{ display:flex; flex-direction:column; gap:8px; justify-content:center; align-items:center; }
 svg.yawviz { width:100%; max-width:420px; height:auto; }
 svg.yawviz .thin{ stroke:#113A5C; stroke-width:2; fill:none; }
 svg.yawviz .ref{ stroke-dasharray:6 6; stroke:#2B6CB0; }
 svg.yawviz .hub{ fill:#111; }
 svg.yawviz .blade{ stroke:#111; stroke-width:3; }
 svg.yawviz text{ font-weight:800; fill:#111; }
 svg.yawviz .nacelle { fill:#2b2b2b; }
 svg.yawviz .blade   { stroke:#111; stroke-width:3; }
 svg.yawviz .hub     { fill:#111; }
 svg.yawviz .marker  { fill:#c01919; stroke:#111; stroke-width:2; }
</style>
</head>
<body>
  <!-- Header -->
  <div class="hero">
    <div class="header-bar">
      <div class="header-left">BLINC-IM</div>
      <div class="header-right">
        <img src="/img/gt" alt="GreenTech" class="logo greentech"/>
        <div class="divider"></div>
        <img src="/img/px" alt="Pixxon AI" class="logo pixxon"/>
      </div>
    </div>
  </div>

  <!-- ROW 1 -->
  <div class="row-top">
    <div class="card initial-card">
      <div class="banner">INITIAL INPUTS</div>
      <div class="initGrid" id="initialGrid"></div>
      <div class="muted" style="margin-top:8px;">ENTER WTG GPS AS N (LAT), E (LON). NO ALTITUDE.</div>
    </div>

    <div class="card">
      <div class="banner">WTG INFO</div>
      <table id="wtg"></table>
    </div>

    <div class="card">
      <div class="banner">STATUS MONITORING</div>
      <table id="status"></table>
    </div>
  </div>

  <!-- ROW 2 -->
  <div class="row-bottom" style="margin-top:16px;">
    <div class="card">
      <div class="banner">LIVE CONTROLS</div>
      <div class="grid" id="grid"></div>

      <details class="fold" id="bladeFold">
        <summary>BLADE PARAMETERS</summary>
        <div class="bladeGrid" id="bladeGrid"></div>
      </details>

      <div style="margin-top:10px">
        <button onclick="refresh()">REFRESH</button>
        <span class="muted">EDIT NUMBERS OR MOVE SLIDERS; UPDATES SEND INSTANTLY.</span>
      </div>
    </div>

    <div class="card">
      <div class="banner">WTG COORDINATES</div>
      <table id="coords"></table>
    </div>

    <div class="card">
      <div class="banner">PT MOTOR DATA</div>
      <div class="subbanner">SCANNER</div>
      <table id="pt_scanner"></table>
      <div class="subbanner">TRACKER</div>
      <table id="pt_tracker"></table>
    </div>

    <!-- NEW: Yaw Oscillation Card -->
    <div class="card">
      <div class="banner">YAW OSCILLATION</div>
      <div class="grid" id="oscGrid"></div>
      <div class="muted" style="margin-top:6px;">
        MODE: OFF disables oscillation. SINE uses amplitude ±MAX and frequency (Hz).
        RANDOM performs a bounded random walk. <b>Yaw rate</b> caps slew speed.
      </div>
    </div>
  </div>

  <!-- ROW 3: WTG schematic + blade counter + yaw orientation -->
  <div class="row-mid" style="margin-top:16px;">
    <div class="card wtg-card">
      <div class="banner">WTG INPUT PARAMETERS</div>
      <div class="svgWrap">
        <svg class="schem" viewBox="0 0 540 720" aria-label="WTG schematic">
          <defs>
            <marker id="arrow" markerWidth="8" markerHeight="8" refX="4" refY="4" orient="auto" markerUnits="userSpaceOnUse">
              <path d="M0,0 L8,4 L0,8 Z" fill="#111" />
            </marker>
          </defs>

          <!-- Rotor disk (static) -->
          <circle cx="270" cy="260" r="160" class="thin dash"/>

          <!-- Tower (static) -->
          <rect x="267" y="260" width="6" height="300" fill="#111"/>

          <!-- BEGIN: parts that rotate with YAW -->
          <g id="yawGroup" transform="rotate(0,270,260)">
            <!-- hub -->
            <circle cx="270" cy="260" r="6" fill="#111"/>
            <!-- three blades; this group rotates about the hub -->
            <line x1="270" y1="260" x2="270" y2="90"  class="thin"/>
            <line x1="270" y1="260" x2="125" y2="340" class="thin"/>
            <line x1="270" y1="260" x2="415" y2="340" class="thin"/>
          </g>
          <!-- END: parts that rotate with YAW -->

          <!-- Nacelle marker (optional, static) -->
          <g transform="translate(270,590)">
            <g transform="scale(0.6)">
              <path d="M0,0 c-12,0 -22,10 -22,22 c0,18 22,42 22,42 s22,-24 22,-42 c0,-12 -10,-22 -22,-22 z" fill="#111"/>
              <circle cx="0" cy="22" r="8" fill="#fff"/>
            </g>
          </g>

          <!-- Rotor dia dimension -->
          <line x1="110" y1="120" x2="110" y2="58"  class="thin"/>
          <line x1="430" y1="120" x2="430" y2="58"  class="thin"/>
          <line x1="110" y1="58"  x2="430" y2="58"  class="thin" marker-start="url(#arrow)" marker-end="url(#arrow)"/>
          <text id="txtDia" x="270" y="38" text-anchor="middle">ROTOR DIA: – M</text>
          <!-- Hub ht dimension -->
          <line x1="120" y1="560" x2="58"  y2="560" class="thin"/>
          <line x1="120" y1="260" x2="58"  y2="260" class="thin"/>
          <line x1="58"  y1="560" x2="58"  y2="260" class="thin" marker-start="url(#arrow)" marker-end="url(#arrow)"/>
          <text id="txtHub" x="28" y="410" text-anchor="middle" transform="rotate(-90,28,410)">HUB HT: – M</text>
          <!-- Location coords centered -->
          <text id="txtLoc" x="50%" y="680" text-anchor="middle">(–, –)</text>
        </svg>
      </div>
    </div>

    <div class="blade-card">
      <div class="blade-head">BLADE COUNTER</div>
      <div class="blade-body">
        <div class="blade-row">
          <div id="b1Dot" class="dot dot-red idle"><span>B1</span></div>
          <div id="b1Count" class="countBox">0</div>
        </div>
        <div class="blade-row">
          <div id="b2Dot" class="dot dot-green idle"><span>B2</span></div>
          <div id="b2Count" class="countBox">0</div>
        </div>
        <div class="blade-row">
          <div id="b3Dot" class="dot dot-blue idle"><span>B3</span></div>
          <div id="b3Count" class="countBox">0</div>
        </div>
      </div>
      <div class="blade-foot">
        <div class="foot-title">TOTAL</div>
        <div id="bladeTotal" class="countBox count-total">0</div>
      </div>
    </div>

    <!-- Yaw Orientation Card (no invert control) -->
    <div class="card yaw-card">
      <div class="banner">WTG ORIENTATION</div>
      <div class="svgYawWrap">
        <svg class="yawviz" id="yawViz" viewBox="0 0 640 520" aria-label="Yaw orientation">
          <rect x="30" y="20" width="580" height="480" rx="60" ry="60" fill="none" stroke="#111" stroke-width="2"/>
          <line x1="320" y1="40" x2="320" y2="480" class="ref" stroke-width="2"/>
          <circle cx="320" cy="260" r="170" class="thin"/>
          <circle cx="320" cy="430" r="8" class="marker"/>

          <g id="yawGroupViz" transform="rotate(0,320,260)">
            <rect x="311" y="232" width="18" height="28" rx="3" class="nacelle"/>
            <circle cx="320" cy="260" r="8" class="hub"/>
            <line x1="140" y1="260" x2="500" y2="260" class="blade"/>
            <line x1="320" y1="260" x2="320" y2="295" class="blade" stroke-width="2"/>
          </g>
          <text id="yawText" x="338" y="302">θ = 0.0°</text>
        </svg>
      </div>
    </div>
  </div>

<script>
const wrapDeg=(a)=>((a+180)%360+360)%360-180;

const FIELDS=[["yaw_deg","YAW (DEG)",-180,180,0.1],["yaw_rate_deg_s","YAW RATE (DEG/S)",-60,60,0.1],["rotor_rpm","ROTOR RPM",0,60,0.1]];
const SELECTS=[["scanner_sp","SCANNER POSITION",["SP1","SP2","SP3","SP4"]]];
const INITIAL_TEXTS=[["model_number","WTG MODEL NUMBER"]];
const INITIAL_NUMS=[["rotor_dia_m","ROTOR DIAMETER (M)",0.1],["hub_ht_m","HUB HEIGHT (M)",0.1],["wtg_lat_deg","WTG GPS N (LAT, °)",0.000001],["wtg_lon_deg","WTG GPS E (LON, °)",0.000001]];
const BLADE_FIELDS=[["blade_thickness_m","BLADE THICKNESS (M)",0.01,0.01],["blade_root_w_m","BLADE ROOT WIDTH (M)",0.01,0.05],["blade_tip_w_m","BLADE TIP WIDTH (M)",0.01,0.02]];

// --- Yaw Oscillation UI ---
const OSC_SELECTS = [
  ["yaw_osc_mode", "OSC MODE", ["OFF","SINE","RANDOM"]],
];
const OSC_FIELDS = [
  ["yaw_osc_rate_deg_s", "YAW RATE (DEG/S)", 0, 60, 0.1],
  ["yaw_osc_max_abs_deg","MAX LIMIT (±DEG)", 0, 180, 1],
];
const OSC_SINE_FIELDS = [
  ["yaw_osc_freq_hz","SINE FREQ (HZ)", 0, 1, 0.005],
];
const OSC_RANDOM_FIELDS = [
  ["yaw_osc_random_step_deg","RANDOM STEP (DEG)", 0, 30, 0.5],
];

const grid=document.getElementById("grid");
const initialGrid=document.getElementById("initialGrid");
const bladeGrid=document.getElementById("bladeGrid");
const tblWTG=document.getElementById("wtg");
const tblSTATUS=document.getElementById("status");
const tblCRD=document.getElementById("coords");
const tblPTScanner=document.getElementById("pt_scanner");
const tblPTTracker=document.getElementById("pt_tracker");
const oscGrid=document.getElementById("oscGrid");

function mkSliderRow(key,label,min,max,step,val){
  const lab=document.createElement("div"); lab.className="rowlabel"; lab.textContent=label;
  const range=document.createElement("input");
  range.type="range"; range.min=min; range.max=max; range.step=step; range.value=val;
  const num=document.createElement("input"); num.type="number"; num.className="mono"; num.min=min; num.max=max; num.step=step; num.value=val;
  range.oninput=()=>{ num.value=range.value; sendNum(key,range.value); };
  num.onchange=()=>{ range.value=num.value; sendNum(key,num.value); };
  grid.appendChild(lab); grid.appendChild(range); grid.appendChild(num);
}
function mkSelectRow(key,label,options,val){
  const lab=document.createElement("div"); lab.className="rowlabel"; lab.textContent=label;
  const sel=document.createElement("select");
  for(const o of options){ const op=document.createElement("option"); op.value=o; op.textContent=o; if(o===val)op.selected=true; sel.appendChild(op); }
  sel.onchange=()=>sendText(key,sel.value);
  const spacer=document.createElement("div");
  grid.appendChild(lab); grid.appendChild(sel); grid.appendChild(spacer);
}
function mkInitText(key,label,val){
  const lab=document.createElement("div"); lab.className="rowlabel"; lab.textContent=label;
  const inp=document.createElement("input"); inp.type="text"; inp.className="mono"; inp.value=val;
  inp.onchange=()=>sendText(key,inp.value);
  initialGrid.appendChild(lab); initialGrid.appendChild(inp);
}
function mkInitNum(key,label,step,val){
  const lab=document.createElement("div"); lab.className="rowlabel"; lab.textContent=label;
  const inp=document.createElement("input"); inp.type="number"; inp.className="mono"; inp.step=step; inp.value=val;
  inp.onchange=()=>sendNum(key,inp.value);
  initialGrid.appendChild(lab); initialGrid.appendChild(inp);
}
function mkBladeNum(key,label,step,min,val){
  const lab=document.createElement("div"); lab.className="rowlabel"; lab.textContent=label;
  const inp=document.createElement("input"); inp.type="number"; inp.className="mono";
  if(step!=null)inp.step=step; if(min!=null)inp.min=min; inp.value=val;
  inp.onchange=()=>sendNum(key,inp.value);
  bladeGrid.appendChild(lab); bladeGrid.appendChild(inp);
}

// Osc rows
function mkOscSliderRow(container, key,label,min,max,step,val){
  const lab=document.createElement("div"); lab.className="rowlabel"; lab.textContent=label;
  const range=document.createElement("input");
  range.type="range"; range.min=min; range.max=max; range.step=step; range.value=val;
  const num=document.createElement("input"); num.type="number"; num.className="mono"; num.min=min; num.max=max; num.step=step; num.value=val;
  range.oninput=()=>{ num.value=range.value; sendNum(key,range.value); };
  num.onchange=()=>{ range.value=num.value; sendNum(key,num.value); };
  container.appendChild(lab); container.appendChild(range); container.appendChild(num);
}
function mkOscSelectRow(container, key,label,options,val){
  const lab=document.createElement("div"); lab.className="rowlabel"; lab.textContent=label;
  const sel=document.createElement("select");
  for(const o of options){ const op=document.createElement("option"); op.value=o; op.textContent=o; if(o===val)op.selected=true; sel.appendChild(op); }
  sel.onchange=()=>sendText(key, sel.value);
  const spacer=document.createElement("div");
  container.appendChild(lab); container.appendChild(sel); container.appendChild(spacer);
}

async function sendNum(key,value){ try{ await fetch(`/set?${encodeURIComponent(key)}=${encodeURIComponent(value)}`);}catch(e){} }
async function sendText(key,value){ try{ await fetch(`/set?${encodeURIComponent(key)}=${encodeURIComponent(value)}`);}catch(e){} }

function renderInfo(s){
  const w=s.wtg_info;
  tblWTG.innerHTML=`
    <tr><th class="section">FIELD</th><th class="section">VALUE</th></tr>
    <tr><td>MODEL NUMBER</td><td class="mono">${w.model_number}</td></tr>
    <tr><td>ROTOR DIA (M)</td><td class="mono">${w.rotor_dia_m}</td></tr>
    <tr><td>HUB HT (M)</td><td class="mono">${w.hub_ht_m}</td></tr>
    <tr><td>SYSTEM HEIGHT (TIP, M)</td><td class="mono">${w.system_height_m}</td></tr>`;
}
function renderStatus(s){
  const d=s.status;
  tblSTATUS.innerHTML=`
    <tr><th class="section">METRIC</th><th class="section">VALUE</th></tr>
    <tr><td>ROTOR RPM</td><td class="mono">${d.rotor_rpm}</td></tr>
    <tr><td>CURRENT YAW (DEG)</td><td class="mono">${d.current_yaw_deg}</td></tr>`;
}
function renderCoords(s){
  const c=s.coordinates_cartesian, g=s.coordinates_gps;
  function vec3(v){ return `(${v[0]}, ${v[1]}, ${v[2]})`; }
  function latlon(v){ return `(${v[0]}, ${v[1]})`; }
  function row(name, cart, gps){ return `<tr><td>${name}</td><td class="mono">${cart}</td><td class="mono">${gps}</td></tr>`; }
  tblCRD.innerHTML=`
    <tr><th class="section"></th><th class="section">CARTESIAN XYZ (M)</th><th class="section">GPS (LAT, LON)</th></tr>
    ${row("TURBINE (HUB)", vec3(c.turbine_xyz_m), latlon(g.turbine_latlon))}
    ${row("TRACKER", vec3(c.tracker_xyz_m), latlon(g.tracker_latlon))}
    ${row("SP1", vec3(c.SP1_xyz_m), latlon(g.SP1_latlon))}
    ${row("SP2", vec3(c.SP2_xyz_m), latlon(g.SP2_latlon))}
    ${row("SP3", vec3(c.SP3_xyz_m), latlon(g.SP3_latlon))}
    ${row("SP4", vec3(c.SP4_xyz_m), latlon(g.SP4_latlon))}
    <tr><td>WTG GPS (N,E)</td><td class="mono" colspan="2">
      <input id="wtgLat" type="number" step="0.000001" min="-90" max="90" style="width:160px"/> ,
      <input id="wtgLon" type="number" step="0.000001" min="-180" max="180" style="width:180px"/>
    </td></tr>`;

  const lat=document.getElementById("wtgLat");
  const lon=document.getElementById("wtgLon");
  lat.value=s.wtg_lat;
  lon.value=s.wtg_lon;
  lat.onchange=()=>sendNum("wtg_lat_deg",lat.value);
  lon.onchange=()=>sendNum("wtg_lon_deg",lon.value);
}

/* PT MOTOR DATA */
function renderPT(s){
  const sc=(s.pt&&s.pt.scanner)||{};
  const tr=(s.pt&&s.pt.tracker)||{};
  const which=s.scanner_sp||"";
  tblPTScanner.innerHTML=`
    <tr><th class="section">METRIC</th><th class="section">VALUE</th></tr>
    <tr><td>SCANNER POSITION</td><td class="mono">${which}</td></tr>
    <tr><td>PAN (DEG)</td><td class="mono">${sc.pan_deg??""}</td></tr>
    <tr><td>TILT (DEG)</td><td class="mono">${sc.tilt_deg??""}</td></tr>`;
  tblPTTracker.innerHTML=`
    <tr><th class="section">METRIC</th><th class="section">VALUE</th></tr>
    <tr><td>PAN (DEG)</td><td class="mono">${tr.pan_deg??""}</td></tr>
    <tr><td>TILT (DEG)</td><td class="mono">${tr.tilt_deg??""}</td></tr>`;
}

/* Blade counter renderer */
function renderBladeCounter(s){
  const b=s.blade||{active:{},hits:{}};
  const setDot=(id,on)=>{ const el=document.getElementById(id); if(!el)return; el.classList.remove("active","idle"); el.classList.add(on?"active":"idle"); };
  setDot("b1Dot",!!b.active.B1);
  setDot("b2Dot",!!b.active.B2);
  setDot("b3Dot",!!b.active.B3);

  document.getElementById("b1Count").textContent=b.hits.B1??0;
  document.getElementById("b2Count").textContent=b.hits.B2??0;
  document.getElementById("b3Count").textContent=b.hits.B3??0;
  document.getElementById("bladeTotal").textContent=b.hits.total??0;
}

/* Schematic labels + yaw-driven rotation (wrapped yaw) */
function renderSchematic(s){
  const dia=s?.wtg_info?.rotor_dia_m??"";
  const hub=s?.wtg_info?.hub_ht_m??"";
  const lat=s?.wtg_lat??"";
  const lon=s?.wtg_lon??"";

  const yawRaw=Number(s?.status?.current_yaw_deg??0);
  const yaw=wrapDeg(yawRaw);

  const tDia=document.getElementById("txtDia");
  const tHub=document.getElementById("txtHub");
  const tLoc=document.getElementById("txtLoc");
  const yawGroup=document.getElementById("yawGroup");

  if(tDia) tDia.textContent=`ROTOR DIA: ${dia} M`;
  if(tHub) tHub.textContent=`HUB HT: ${hub} M`;
  if(tLoc) tLoc.textContent=`(${lat}, ${lon})`;

  if(yawGroup) yawGroup.setAttribute("transform",`rotate(${yaw.toFixed(2)},270,260)`);
}

/* Yaw Orientation card renderer (wrapped) */
function renderYawViz(s){
  const yawRaw=Number(s?.status?.current_yaw_deg??0);
  const ang=wrapDeg(yawRaw);
  const g=document.getElementById("yawGroupViz");
  const t=document.getElementById("yawText");
  if(g) g.setAttribute("transform",`rotate(${ang.toFixed(2)},320,260)`);
  if(t) t.textContent=`θ = ${ang.toFixed(1)}°`;
}

async function refresh(){
  try{
    const pr=await fetch("/get"); const ps=await pr.json();

    // INITIAL INPUTS
    initialGrid.innerHTML="";
    for(const [k,l] of INITIAL_TEXTS) mkInitText(k,l, ps[k]??"");
    for(const [k,l,step] of INITIAL_NUMS) mkInitNum(k,l,step, ps[k]);

    // LIVE CONTROLS
    grid.innerHTML="";
    for(const [k,l,mi,ma,st] of FIELDS) mkSliderRow(k,l,mi,ma,st, ps[k]);
    for(const [k,l,opts] of SELECTS) mkSelectRow(k,l,opts, ps[k]);

    // BLADE PARAMS
    bladeGrid.innerHTML="";
    for(const [k,l,step,min] of BLADE_FIELDS) mkBladeNum(k,l,step,min, ps[k]);

    // OSC CONTROLS
    oscGrid.innerHTML = "";
    for (const [k,l,opts] of OSC_SELECTS) mkOscSelectRow(oscGrid, k, l, opts, ps[k]);
    for (const [k,l,mi,ma,st] of OSC_FIELDS) mkOscSliderRow(oscGrid, k, l, mi, ma, st, ps[k]);
    for (const [k,l,mi,ma,st] of OSC_SINE_FIELDS) mkOscSliderRow(oscGrid, k, l, mi, ma, st, ps[k]);
    for (const [k,l,mi,ma,st] of OSC_RANDOM_FIELDS) mkOscSliderRow(oscGrid, k, l, mi, ma, st, ps[k]);

    const sr=await fetch("/state"); const state=await sr.json();
    renderInfo(state); renderStatus(state); renderCoords(state); renderPT(state);
    renderBladeCounter(state); renderSchematic(state); renderYawViz(state);
  }catch(e){ console.error(e); }
}

setInterval(async ()=>{
  try{
    const sr=await fetch("/state"); const state=await sr.json();
    renderInfo(state); renderStatus(state); renderCoords(state); renderPT(state);
    renderBladeCounter(state); renderSchematic(state); renderYawViz(state);
  }catch(e){}
}, 250);

refresh();
</script>
</body>
</html>"""

# ===================== PARAM SNAPSHOT / SETTER =====================
def get_params_snapshot():
    with _PARAM_LOCK:
        return {
            "yaw_deg": yaw_deg,
            "yaw_rate_deg_s": yaw_rate_deg_s,
            "rotor_rpm": rotor_rpm,
            "rotor_dir": rotor_dir,
            "rotor_dia_m": rotor_dia_m,
            "hub_ht_m": hub_ht_m,
            "blade_thickness_m": blade_thickness_m,
            "blade_root_w_m": blade_root_w_m,
            "blade_tip_w_m": blade_tip_w_m,
            "model_number": model_number,
            "wtg_lat_deg": wtg_lat_deg,
            "wtg_lon_deg": wtg_lon_deg,
            "scanner_sp": scanner_sp,
            # Osc params
            "yaw_osc_mode": yaw_osc_mode,
            "yaw_osc_rate_deg_s": yaw_osc_rate_deg_s,
            "yaw_osc_max_abs_deg": yaw_osc_max_abs_deg,
            "yaw_osc_freq_hz": yaw_osc_freq_hz,
            "yaw_osc_random_step_deg": yaw_osc_random_step_deg,
        }

def _set_param(name: str, val):
    global yaw_deg, yaw_rate_deg_s, rotor_rpm, rotor_dir, rotor_dia_m, hub_ht_m
    global blade_thickness_m, blade_root_w_m, blade_tip_w_m
    global model_number, wtg_lat_deg, wtg_lon_deg, scanner_sp
    global yaw_osc_mode, yaw_osc_rate_deg_s, yaw_osc_max_abs_deg, yaw_osc_freq_hz, yaw_osc_random_step_deg
    with _PARAM_LOCK:
        if name == "model_number":              model_number = str(val)
        elif name == "wtg_lat_deg":             wtg_lat_deg = float(val)
        elif name == "wtg_lon_deg":             wtg_lon_deg = float(val)
        elif name == "yaw_deg":                 yaw_deg = float(val)
        elif name == "yaw_rate_deg_s":          yaw_rate_deg_s = float(val)
        elif name == "rotor_rpm":               rotor_rpm = max(0.0, float(val))
        elif name == "rotor_dir":
            # Accept numbers or strings like "CW"/"CCW"
            try:
                rotor_dir = +1 if float(val) >= 0 else -1
            except Exception:
                s = str(val).strip().upper()
                rotor_dir = -1 if s in ("CW","CLOCKWISE","-1","NEG","-") else +1
        elif name == "rotor_dia_m":             rotor_dia_m = max(1.0, float(val))
        elif name == "hub_ht_m":                hub_ht_m = max(0.5, float(val))
        elif name == "blade_thickness_m":       blade_thickness_m = max(0.01, float(val))
        elif name == "blade_root_w_m":          blade_root_w_m = max(0.05, float(val))
        elif name == "blade_tip_w_m":           blade_tip_w_m = max(0.02, float(val))
        elif name == "scanner_sp":              scanner_sp = str(val).upper() if str(val).upper() in ("SP1","SP2","SP3","SP4") else scanner_sp
        # Osc controls
        elif name == "yaw_osc_mode":
            s = str(val).strip().upper()
            yaw_osc_mode = s if s in ("OFF","SINE","RANDOM") else yaw_osc_mode
        elif name == "yaw_osc_rate_deg_s":
            yaw_osc_rate_deg_s = max(0.0, float(val))
        elif name == "yaw_osc_max_abs_deg":
            yaw_osc_max_abs_deg = max(0.0, min(180.0, float(val)))
        elif name == "yaw_osc_freq_hz":
            yaw_osc_freq_hz = max(0.0, float(val))
        elif name == "yaw_osc_random_step_deg":
            yaw_osc_random_step_deg = max(0.0, float(val))

# ===================== HTTP HANDLER =====================
class ParamHTTPHandler(BaseHTTPRequestHandler):
    def _send_text(self, code, text, ctype="application/json; charset=utf-8"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        if isinstance(text, (dict, list)):
            text = json.dumps(text)
        if isinstance(text, str):
            text = text.encode("utf-8")
        self.wfile.write(text)

    def _send_bin(self, code, data, ctype="application/octet-stream"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(data if data is not None else b"")

    def do_GET(self):
        p = urlparse(self.path)
        if p.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(_HTML.encode("utf-8"))
            return

        # Header logos (served locally)
        if p.path == "/img/gt":
            data = _ASSETS.get("gt")
            return self._send_bin(200, data, "image/png") if data else self._send_text(404, {"error":"GT logo missing"})
        if p.path == "/img/px":
            data = _ASSETS.get("px")
            return self._send_bin(200, data, "image/png") if data else self._send_text(404, {"error":"PIXXON logo missing"})

        if p.path == "/get":
            return self._send_text(200, get_params_snapshot())

        if p.path == "/state":
            return self._send_text(200, compute_live_state())

        if p.path == "/set":
            qs = parse_qs(p.query)
            for k, vals in qs.items():
                v = vals[0]
                if k in ("model_number","scanner_sp","rotor_dir","yaw_osc_mode"):
                    _set_param(k, v)
                else:
                    try:
                        _set_param(k, float(v))
                    except Exception:
                        pass
            return self._send_text(200, {"ok": True})

        return self._send_text(404, {"error":"NOT FOUND"})

def start_param_server(host="127.0.0.1", port=8888):
    httpd = HTTPServer((host, port), ParamHTTPHandler)
    print(f"[DASHBOARD] OPEN http://{host}:{port}")
    httpd.serve_forever()

# ---------- blade/frustum intersection helpers ----------
def _blade_point_world(z_along_m, roll_k, nacelle_rot, hub_xyz):
    z = z_along_m
    y_b = -z * math.sin(roll_k)
    z_b =  z * math.cos(roll_k)
    x_b = 0.0
    c, s = math.cos(nacelle_rot), math.sin(nacelle_rot)
    x_w = x_b*c - y_b*s
    y_w = x_b*s + y_b*c
    z_w = z_b
    return (hub_xyz[0] + x_w, hub_xyz[1] + y_w, hub_xyz[2] + z_w)

def _point_in_benewake_fov(Pw, Sw, Rsw, tanH, tanV, near_m, far_m):
    dx, dy, dz = (Pw[0]-Sw[0], Pw[1]-Sw[1], Pw[2]-Sw[2])
    vx = Rsw[0][0]*dx + Rsw[1][0]*dy + Rsw[2][0]*dz
    vy = Rsw[0][1]*dx + Rsw[1][1]*dy + Rsw[2][1]*dz
    vz = Rsw[0][2]*dx + Rsw[1][2]*dy + Rsw[2][2]*dz
    if vx < near_m or vx > far_m:
        return False
    return (abs(vy) <= tanH * vx) and (abs(vz) <= tanV * vx)

def _update_blade_counters(active_now):
    with _PARAM_LOCK:
        for k in ("B1","B2","B3"):
            if active_now[k] and not _blade_active[k]:
                _blade_hits[k] += 1
                _blade_hits["total"] += 1
            _blade_active[k] = active_now[k]

# ===================== MAIN =====================
if __name__ == "__main__":
    threading.Thread(target=start_param_server, args=("127.0.0.1", 8888), daemon=True).start()

    foxglove.start_server(host="127.0.0.1", port=8765)
    scene_ch = SceneUpdateChannel(topic="/scene")
    tf_ch = FrameTransformsChannel(topic="/tf")

    entities, tower_h, nacelle_l, nose_len = build_scene_entities()
    scene_ch.log(SceneUpdate(entities=entities))

    print_sp1_panel_preset()

    # Live rebuild-on-change guard
    last_rotor = rotor_dia_m
    last_hub   = hub_ht_m
    last_theta = theta_deg_val
    last_b_thk = blade_thickness_m
    last_rootw = blade_root_w_m
    last_tipw  = blade_tip_w_m

    t0 = _START_TIME
    last_scene = 0.0
    last_print = 0.0

    publish_world_root(tf_ch)

    while True:
        # --- timebase + dt for oscillation ---
        now_s = time.time()
        dt = max(0.0, now_s - _last_loop_ts)
        _last_loop_ts = now_s
        t = now_s - t0

        # Live rebuild if sizing knobs changed
        if (rotor_dia_m != last_rotor or hub_ht_m != last_hub or
            theta_deg_val != last_theta or blade_thickness_m != last_b_thk or
            blade_root_w_m != last_rootw or blade_tip_w_m != last_tipw):
            entities, tower_h, nacelle_l, nose_len = build_scene_entities()
            scene_ch.log(SceneUpdate(entities=entities))
            last_rotor, last_hub = rotor_dia_m, hub_ht_m
            last_theta, last_b_thk = theta_deg_val, blade_thickness_m
            last_rootw, last_tipw = blade_root_w_m, blade_tip_w_m

        # Read params
        with _PARAM_LOCK:
            yaw0 = yaw_deg
            yr   = yaw_rate_deg_s
            dia  = rotor_dia_m
            rpm  = rotor_rpm
            which_sp = scanner_sp
            dir_sign = rotor_dir

            # Osc params
            mode = yaw_osc_mode
            rate = float(yaw_osc_rate_deg_s)
            maxabs = float(yaw_osc_max_abs_deg)
            f_hz = float(yaw_osc_freq_hz)
            step_deg = float(yaw_osc_random_step_deg)

        # --- update oscillation state (_osc_angle_deg) ---
        if mode == "SINE" and maxabs > 0.0 and f_hz > 0.0:
            _osc_phase += 2.0 * math.pi * f_hz * dt
            if _osc_phase > 1e6:
                _osc_phase -= 1e6
            target = maxabs * math.sin(_osc_phase)
            err = target - _osc_angle_deg
            max_step = rate * dt if rate > 0 else abs(err)
            delta = max(-max_step, min(max_step, err))
            _osc_angle_deg += delta
        elif mode == "RANDOM" and maxabs > 0.0:
            import random
            proposed = _osc_angle_deg + (random.uniform(-1.0, 1.0) * step_deg)
            proposed = max(-maxabs, min(maxabs, proposed))
            max_step = rate * dt if rate > 0 else abs(proposed - _osc_angle_deg)
            delta = max(-max_step, min(max_step, proposed - _osc_angle_deg))
            _osc_angle_deg += delta
        else:
            # OFF or invalid -> decay toward 0 using rate limit
            if _osc_angle_deg != 0.0:
                sign = 1.0 if _osc_angle_deg < 0 else -1.0
                decay = sign * min(abs(_osc_angle_deg), rate * dt if rate > 0 else abs(_osc_angle_deg))
                _osc_angle_deg += decay

        # ======= DECISION: use two yaw angles =======
        yaw_cmd_deg = yaw0 + yr * t + _osc_angle_deg  # oscillating (nacelle/hub/blades)
        yaw_cmd_rad = math.radians(yaw_cmd_deg)
        sp_yaw_deg  = yaw0 + yr * t                   # steady (SCAN/SP)
        sp_yaw_rad  = math.radians(sp_yaw_deg)

        # Publish frames separately
        publish_scan_frame(tf_ch, sp_yaw_rad)
        publish_nacelle_frames(tf_ch, yaw_cmd_rad, hub_ht_m, 3.6)

        # Hub target (with vertical offset) in WORLD coordinates (oscillating yaw)
        hub_x, hub_y, hub_z = hub_center_world(yaw_cmd_rad, 3.6)
        hub_xyz = (hub_x, hub_y, hub_z)
        target_world = (hub_x, hub_y, hub_z + hub_target_z_offset_m)

        # ---------------- TRACKER (BENEWAKE) ----------------
        tracker_base_world = (0.0, dia*1.125, tracker_ht_m)
        tr_pan, tr_tilt = solve_pan_tilt_to_target(tracker_base_world, target_world)
        tf_ch.log(FrameTransforms(transforms=[
            FrameTransform(parent_frame_id=WORLD, child_frame_id=B_PAN,
                           translation=Vector3(x=tracker_base_world[0], y=tracker_base_world[1], z=tracker_base_world[2]),
                           rotation=quat_from_euler(0.0, 0.0, -math.pi/2 + tr_pan)),
            FrameTransform(parent_frame_id=B_PAN, child_frame_id=B_TILT,
                           translation=Vector3(x=pan_to_tilt_offset[0], y=pan_to_tilt_offset[1], z=pan_to_tilt_offset[2]),
                           rotation=quat_from_euler(0.0, tr_tilt, 0.0)),
            FrameTransform(parent_frame_id=B_TILT, child_frame_id=B_SENSOR,
                           translation=Vector3(x=tilt_to_sensor_offset[0], y=tilt_to_sensor_offset[1], z=tilt_to_sensor_offset[2]),
                           rotation=Quaternion(x=0,y=0,z=0,w=1)),
        ]))

        with _PARAM_LOCK:
            _last_tr_pan_deg  = math.degrees(tr_pan)
            _last_tr_tilt_deg = math.degrees(tr_tilt)

        # Benewake FOV
        ox, oy, oz = sensor_origin_with_offsets(tr_pan, tr_tilt)
        sx, sy, sz = tracker_base_world[0] + ox, tracker_base_world[1] + oy, tracker_base_world[2] + oz
        benewake_origin = (sx, sy, sz)
        dist_to_target = math.sqrt((target_world[0]-sx)**2 + (target_world[1]-sy)**2 + (target_world[2]-sz)**2)
        benewake_far = max(dist_to_target - 0.01, benewake_near_m + 1e-3)

        R_pan = rotZ(-math.pi/2 + tr_pan)
        R_tilt = rotY(tr_tilt)
        R_sw = matmul3(R_pan, R_tilt)

        scene_ch.log(SceneUpdate(entities=[
            make_fov_frustum_X(B_SENSOR, "fov_benewake",
                               near_m=benewake_near_m, far_m=benewake_far,
                               haov_deg=benewake_haov_deg, vaov_deg=benewake_vaov_deg,
                               color=benewake_color),
        ]))

        # ---------------- SCANNER (LIVOX + S-LASER + CAMERA) ----------------
        sp_world = sp_xy_world_all(sp_yaw_rad)  # decoupled from oscillation
        sbx, sby, sbz = (*sp_world[which_sp][:2], tracker_ht_m)
        scanner_base_world = (sbx, sby, sbz)

        sc_pan, sc_tilt = solve_pan_tilt_to_target(scanner_base_world, target_world)

        tf_ch.log(FrameTransforms(transforms=[
            FrameTransform(parent_frame_id=WORLD, child_frame_id=L_PAN,
                        translation=Vector3(x=scanner_base_world[0], y=scanner_base_world[1], z=scanner_base_world[2]),
                        rotation=quat_from_euler(0.0, 0.0, -math.pi/2 + sc_pan)),
            FrameTransform(parent_frame_id=L_PAN, child_frame_id=L_TILT,
                        translation=Vector3(x=pan_to_tilt_offset[0], y=pan_to_tilt_offset[1], z=pan_to_tilt_offset[2]),
                        rotation=quat_from_euler(0.0, sc_tilt, 0.0)),
            FrameTransform(parent_frame_id=L_TILT, child_frame_id=L_SENSOR,
                        translation=Vector3(x=tilt_to_sensor_offset[0], y=tilt_to_sensor_offset[1], z=tilt_to_sensor_offset[2]),
                        rotation=Quaternion(x=0,y=0,z=0,w=1)),
        ]))

        tf_ch.log(FrameTransforms(transforms=[
            FrameTransform(parent_frame_id=WORLD, child_frame_id=S_PAN,
                           translation=Vector3(x=scanner_base_world[0], y=scanner_base_world[1], z=scanner_base_world[2]),
                           rotation=quat_from_euler(0.0, 0.0, -math.pi/2 + sc_pan)),
            FrameTransform(parent_frame_id=S_PAN, child_frame_id=S_TILT,
                           translation=Vector3(x=pan_to_tilt_offset[0], y=pan_to_tilt_offset[1], z=pan_to_tilt_offset[2]),
                           rotation=quat_from_euler(0.0, sc_tilt, 0.0)),
            FrameTransform(parent_frame_id=S_TILT, child_frame_id=S_SENSOR,
                           translation=Vector3(x=tilt_to_sensor_offset[0], y=tilt_to_sensor_offset[1], z=tilt_to_sensor_offset[2]),
                           rotation=Quaternion(x=0,y=0,z=0,w=1)),
            FrameTransform(parent_frame_id=S_SENSOR, child_frame_id=C_SENSOR,
                           translation=Vector3(x=0.0, y=0.0, z=0.0),
                           rotation=Quaternion(x=0,y=0,z=0,w=1)),
        ]))

        with _PARAM_LOCK:
            _last_sc_pan_deg  = math.degrees(sc_pan)
            _last_sc_tilt_deg = math.degrees(sc_tilt)

        scene_ch.log(SceneUpdate(entities=[
            make_fov_frustum_X(L_SENSOR, "fov_livox",
                               near_m=livox_near_m, far_m=livox_far_m,
                               haov_deg=livox_haov_deg, vaov_deg=livox_vaov_deg,
                               color=livox_color),
            make_fov_frustum_X(S_SENSOR, "fov_scanner_slaser",
                               near_m=slaser_near_m, far_m=slaser_far_m,
                               haov_deg=slaser_haov_deg, vaov_deg=slaser_vaov_deg,
                               color=slaser_color),
            make_fov_frustum_X(C_SENSOR, "scanner_camera_fov",
                               near_m=camera_near_m, far_m=camera_far_m,
                               haov_deg=camera_haov_deg, vaov_deg=camera_vaov_deg,
                               color=camera_color),
            make_camera_view_mask(C_SENSOR, "scanner_camera_mask",
                                  haov_deg=camera_haov_deg, vaov_deg=camera_vaov_deg,
                                  dist_m=camera_mask_dist, pad_scale=camera_mask_pad,
                                  alpha=camera_mask_alpha),
            make_view_anchor_entity(C_SENSOR, "scanner_cam_target_anchor", PANEL_PRESET_DISTANCE),
        ]))

        # ============ BLADE ↔ BENEWAKE INTERSECTION & COUNTER ============
        tanH = math.tan(math.radians(benewake_haov_deg * 0.5)) * HIT_VISUAL_GAIN
        tanV = math.tan(math.radians(benewake_vaov_deg * 0.5)) * HIT_VISUAL_GAIN
        nacelle_rot = yaw_cmd_rad + math.pi/2
        blade_len = rotor_dia_m / 2.0

        samples = [blade_len*s for s in (0.15, 0.30, 0.45, 0.60, 0.75, 0.90)]
        omega = 2.0*math.pi*(rpm/60.0)
        roll_base = dir_sign * omega * t  # direction control (+1 CCW, -1 CW)

        flags = {}
        for name, roll_off in (("B1",0.0), ("B2", 2.0*math.pi/3.0), ("B3", 4.0*math.pi/3.0)):
            hit = False
            roll_k = roll_base + roll_off
            for z in samples:
                Pw = _blade_point_world(z, roll_k, nacelle_rot, hub_xyz)
                if _point_in_benewake_fov(Pw, benewake_origin, R_sw, tanH, tanV, benewake_near_m, benewake_far):
                    hit = True; break
            flags[name] = hit

        _update_blade_counters(flags)

        # Spin blades for viz
        tf_ch.log(FrameTransforms(transforms=[
            FrameTransform(parent_frame_id=HUB, child_frame_id=B1,
                           translation=Vector3(x=0,y=0,z=0),
                           rotation=quat_from_euler(roll_base, 0, 0)),
            FrameTransform(parent_frame_id=HUB, child_frame_id=B2,
                           translation=Vector3(x=0,y=0,z=0),
                           rotation=quat_from_euler(roll_base + 2.0*math.pi/3.0, 0, 0)),
            FrameTransform(parent_frame_id=HUB, child_frame_id=B3,
                           translation=Vector3(x=0,y=0,z=0),
                           rotation=quat_from_euler(roll_base + 4.0*math.pi/3.0, 0, 0)),
        ]))

        # Console readout
        if now_s - last_print >= 0.25:
            with _PARAM_LOCK:
                b1h = _blade_hits["B1"]; b2h = _blade_hits["B2"]; b3h = _blade_hits["B3"]; tot = _blade_hits["total"]
                trp, trt = _last_tr_pan_deg, _last_tr_tilt_deg
                scp, sct = _last_sc_pan_deg, _last_sc_tilt_deg
            print(f"[BLADE HIT] B1:{b1h}  B2:{b2h}  B3:{b3h}  TOTAL:{tot}  |  RPM {rpm:.1f}  YAW {wrap_deg(yaw_cmd_deg):+.2f}°  |  TR(P,T)=({trp:.2f},{trt:.2f})  SC(P,T)=({scp:.2f},{sct:.2f})")
            last_print = now_s

        # Periodic resend (late joiners)
        if now_s - last_scene > 1.0:
            scene_ch.log(SceneUpdate(entities=entities))
            last_scene = now_s

        publish_world_root(tf_ch)
        time.sleep(1.0/30.0)
