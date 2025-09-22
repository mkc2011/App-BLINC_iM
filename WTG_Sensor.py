import math, time, datetime
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

# ============== CONTROL KNOBS ==============
# --- Yaw control (absolute + optional rate) ---
yaw_deg = 0.0         # 0° means nacelle faces world +Y (by design offset below)
yaw_rate_deg_s = 0.0  # deg/s

# SPs start rotated +270° at yaw=0 (your saved preference)
sp_yaw_offset_deg = 270.0

# --- Rotor control ---
rotor_rpm = 12.0
rot_dir   = -1.0      # +1=CCW about +X (looking forward), -1=CW

# --- Geometry / markers ---
rotor_dia_m   = 36.0
hub_ht_m      = 80.0
theta_deg_val = 10.0

# SP bullseye styling
marker_inner_r_m = 0.45
marker_outer_r_m = 0.90   # OUTER radius (so diameter = 2*0.90)
marker_ring_w_m  = 0.10
marker_segments  = 96
marker_color_red = Color(r=1.0, g=0.0, b=0.0, a=1.0)

# Label styling (for SP names / geo labels)
label_height_m   = 2.0
label_offset_m   = 1.0
label_font_size  = 1.2
label_color      = Color(r=0.0, g=0.0, b=0.0, a=1.0)     # black text
label_bg_color   = Color(r=1.0, g=1.0, b=1.0, a=1.0)     # white background
label_billboard  = True

# Optional thin poles under labels (these yaw with SPs since they’re in SCAN)
draw_label_poles    = True
label_pole_radius_m = 0.03

# Ground circle ring
circle_ring_width_m = 0.6
circle_segments     = 256
circle_color        = Color(r=0.10, g=0.45, b=0.12, a=0.9)  # visible dark green

# TRACKER marker (fixed in world)
tracker_color_blue  = Color(r=0.0, g=0.35, b=1.0, a=1.0)

# --- Tracker mast (solid black) ---
tracker_ht_m       = 1.0                  # mast height above ground
tracker_radius_m   = 0.30                 # mast radius
tracker_color_black = Color(r=0.0, g=0.0, b=0.0, a=1.0)

# --- Pan/Tilt geometry & aim target ---
hub_target_z_offset_m = 20.0              # meters ABOVE hub center to aim (0 for hub center)

# Pan->Tilt and Tilt->Sensor mechanical offsets (expressed in the parent frame)
pt_offset      = (0.0, 0.05, 0.01)        # pan axis to tilt axis (x,y,z) in meters
sensor_offset  = (0.0, 0.0, 0.01)         # tilt axis to sensor origin (x,y,z) in meters

# Frame names for clarity
PAN    = "pan_axis"
TILT   = "tilt_axis"
SENSOR = "sensor_tracker"
TRACKER_BASE = "tracker_base"             # at (0, rotor_dia*1.125, 0)

# --- Sensor FOV (anchored at SENSOR) ---
sensor_enable            = True
sensor_haov_deg          = 0.35   # Horizontal AOV (full angle)
sensor_vaov_deg          = 0.15   # Vertical AOV (full angle)
sensor_working_dist_m    = 120.0  # default far distance
sensor_near_m            = 0.20   # Small near plane to avoid degenerate apex
sensor_color             = Color(r=1.0, g=0.0, b=0.0, a=0.35)  # mild transparent red

# --- Site / Geodesy (optional; used for dynamic geo labels) ---
base_lat_deg = 13.08268   # <— ENTER your tower base latitude (5 dp)
base_lon_deg = 80.27072   # <— ENTER your tower base longitude (5 dp)
use_dynamic_sp_labels = True

# ============== Helpers ==============
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

# ---------- Geodesy helpers (for geo labels) ----------
def _meters_per_deg_lat(lat_deg: float) -> float:
    phi = math.radians(lat_deg)
    return (111132.92
            - 559.82 * math.cos(2*phi)
            +   1.175 * math.cos(4*phi)
            -   0.0023 * math.cos(6*phi))

def _meters_per_deg_lon(lat_deg: float) -> float:
    phi = math.radians(lat_deg)
    return (111412.84 * math.cos(phi)
            -     93.5 * math.cos(3*phi)
            +      0.118 * math.cos(5*phi))

def _rotate_xy(x_east: float, y_north: float, yaw_deg: float) -> tuple[float, float]:
    th = math.radians(yaw_deg)
    c, s = math.cos(th), math.sin(th)
    return (x_east * c - y_north * s,   # x'
            x_east * s + y_north * c)   # y'

def get_marker_geocoords(base_lat_deg: float, base_lon_deg: float,
                         yaw_deg_now: float,
                         round_dp: int = 5) -> dict:
    """
    Returns lat/lon (rounded) for TRACKER and SP1..SP4.
    World axes: +X=East, +Y=North.
    TRACKER ignores yaw; SPs rotate by yaw + 90° + sp_yaw_offset_deg.
    """
    x = rotor_dia_m / 4.0
    y_R = rotor_dia_m * 1.125
    y_T = hub_ht_m * math.tan(math.radians(theta_deg_val))

    sp_local = {
        "SP1": (-x, +y_R),
        "SP2": (-x, +y_T),
        "SP3": (-x, -y_R),
        "SP4": ( +x, -y_T),
    }

    tracker_xy = (0.0, y_R)  # fixed

    # apply SP yaw
    sp_rot_deg = yaw_deg_now + 90.0 + sp_yaw_offset_deg
    sp_world = {name: _rotate_xy(xe, yn, sp_rot_deg) for name, (xe, yn) in sp_local.items()}

    m_per_deg_lat = _meters_per_deg_lat(base_lat_deg)
    m_per_deg_lon = _meters_per_deg_lon(base_lat_deg)

    def to_latlon(dx_east_m: float, dy_north_m: float) -> tuple[float, float]:
        dlat = dy_north_m / m_per_deg_lat
        dlon = dx_east_m / m_per_deg_lon
        return (round(base_lat_deg + dlat, round_dp),
                round(base_lon_deg + dlon, round_dp))

    out = {}
    out["TRACKER"] = to_latlon(tracker_xy[0], tracker_xy[1])
    for name, (xe, yn) in sp_world.items():
        out[name] = to_latlon(xe, yn)
    return out

# ============== Mesh helpers ==============
def make_filled_disk_tris(cx: float, cy: float, radius: float, segments: int):
    tris = []
    center = Point3(x=cx, y=cy, z=0.0)
    ring = []
    for i in range(segments):
        th = (2.0 * math.pi * i) / segments
        ring.append(Point3(x=cx + radius*math.cos(th),
                           y=cy + radius*math.sin(th),
                           z=0.0))
    for i in range(segments):
        j = (i + 1) % segments
        tris += [center, ring[i], ring[j]]
    return tris

def make_circle_ring_tris(cx: float, cy: float, r_outer: float, ring_w: float, segments: int):
    r_out = max(r_outer, 0.001)
    r_in  = max(r_outer - max(ring_w, 0.001), 0.001)
    ring_out, ring_in = [], []
    for i in range(segments):
        th = (2.0 * math.pi * i) / segments
        c, s = math.cos(th), math.sin(th)
        ring_out.append(Point3(x=cx + r_out*c, y=cy + r_out*s, z=0.0))
        ring_in.append (Point3(x=cx + r_in *c, y=cy + r_in *s, z=0.0))
    tris = []
    for i in range(segments):
        j = (i + 1) % segments
        oi, oj = ring_out[i], ring_out[j]
        ii, ij = ring_in[i],  ring_in[j]
        tris += [oi, oj, ij]
        tris += [oi, ij, ii]
    return tris

# --- Sensor FOV frustum along +X (near/far rectangles), colored translucent ---
def make_fov_frustum_X(frame_id: str, entity_id: str,
                       near_m: float, far_m: float,
                       haov_deg: float, vaov_deg: float,
                       color: Color) -> SceneEntity:
    near_m = max(near_m, 1e-3)
    far_m  = max(far_m, near_m + 1e-3)

    h_half = math.radians(haov_deg * 0.5)
    v_half = math.radians(vaov_deg * 0.5)

    # half extents in Y (horizontal) and Z (vertical)
    y_n = near_m * math.tan(h_half)
    z_n = near_m * math.tan(v_half)
    y_f =  far_m * math.tan(h_half)
    z_f =  far_m * math.tan(v_half)

    # Near rectangle (x=near)
    n1 = Point3(x=near_m, y=-y_n, z=-z_n)
    n2 = Point3(x=near_m, y= y_n, z=-z_n)
    n3 = Point3(x=near_m, y= y_n, z= z_n)
    n4 = Point3(x=near_m, y=-y_n, z= z_n)
    # Far rectangle (x=far)
    f1 = Point3(x=far_m, y=-y_f, z=-z_f)
    f2 = Point3(x=far_m, y= y_f, z=-z_f)
    f3 = Point3(x=far_m, y= y_f, z= z_f)
    f4 = Point3(x=far_m, y=-y_f, z= z_f)

    tris = []
    # near face
    tris += [n1, n2, n3]; tris += [n1, n3, n4]
    # far face
    tris += [f1, f3, f2]; tris += [f1, f4, f3]
    # sides
    tris += [n1, f2, f1]; tris += [n1, n2, f2]   # bottom (-Z)
    tris += [n2, f3, f2]; tris += [n2, n3, f3]   # +Y
    tris += [n3, f4, f3]; tris += [n3, n4, f4]   # +Z
    tris += [n4, f1, f4]; tris += [n4, n1, f1]   # -Y

    return SceneEntity(
        frame_id=frame_id, id=entity_id,
        timestamp=now_ts(), lifetime=Duration.from_secs(0),
        triangles=[TriangleListPrimitive(points=tris, color=color)]
    )

# --- SP bullseyes (authored at yaw=0 local coords; frame controls yaw) ---
def make_bullseye_markers_with_labels(frame_id: str, entity_id: str,
                                      rotor_dia: float, hub_ht: float, theta_deg: float,
                                      inner_r: float, outer_r: float, ring_w: float,
                                      segments: int,
                                      color: Color,
                                      label_offset: float, font_size: float,
                                      label_color: Color, label_bg_color: Color) -> SceneEntity:
    theta_rad = math.radians(theta_deg)
    x = rotor_dia / 4.0
    y_R = rotor_dia * 1.125
    y_T = hub_ht * math.tan(theta_rad)

    pts = [
        (-x,  +y_R, 0.0),  # SP1
        (-x,  +y_T, 0.0),  # SP2
        (-x,  -y_R, 0.0),  # SP3
        ( x,  -y_T, 0.0),  # SP4
    ]
    labels = ["SP1", "SP2", "SP3", "SP4"]

    all_tris = []
    for (px, py, _) in pts:
        all_tris += make_filled_disk_tris(px, py, inner_r, segments)
        all_tris += make_circle_ring_tris(px, py, outer_r, ring_w, segments)

    entity_kwargs = dict(
        frame_id=frame_id,
        id=entity_id,
        timestamp=now_ts(),
        lifetime=Duration.from_secs(0),
        triangles=[TriangleListPrimitive(points=all_tris, color=color)],
    )

    # Static SP name labels ONLY if we are NOT drawing dynamic geo labels
    if HAVE_TEXT and not use_dynamic_sp_labels:
        texts = []
        for (px, py, _), name in zip(pts, labels):
            r = max(math.hypot(px, py), 1e-6)
            ux, uy = px / r, py / r
            lx, ly = px + ux * label_offset, py + uy * label_offset
            text_kwargs = dict(
                pose=Pose(position=Vector3(x=lx, y=ly, z=label_height_m),
                          orientation=Quaternion(x=0, y=0, z=0, w=1)),
                text=name, font_size=font_size, color=label_color,
            )
            try:
                texts.append(TextPrimitive(**text_kwargs,
                                           billboard=label_billboard,
                                           background_color=label_bg_color))
            except TypeError:
                try:
                    texts.append(TextPrimitive(**text_kwargs,
                                               billboard=label_billboard,
                                               background=True,
                                               bg_color=label_bg_color))
                except TypeError:
                    texts.append(TextPrimitive(**text_kwargs,
                                               billboard=label_billboard))
        entity_kwargs["texts"] = texts

    return SceneEntity(**entity_kwargs)

def make_label_poles_entity(frame_id: str, entity_id: str,
                            pts_xy, height_m: float, radius_m: float,
                            color: Color = Color(r=1.0, g=1.0, b=1.0, a=0.8),
                            segments: int = 24) -> SceneEntity:
    poles_tris = []
    for (px, py) in pts_xy:
        ring_top, ring_bot = [], []
        for i in range(segments):
            th = (2.0 * math.pi * i) / segments
            cx, cy = math.cos(th), math.sin(th)
            ring_top.append(Point3(x=px + radius_m*cx, y=py + radius_m*cy, z=height_m))
            ring_bot.append(Point3(x=px + radius_m*cx, y=py + radius_m*cy, z=0.0))
        for i in range(segments):
            j = (i + 1) % segments
            bi, bj = ring_bot[i], ring_bot[j]
            ti, tj = ring_top[i], ring_top[j]
            poles_tris += [bi, bj, tj]
            poles_tris += [bi, tj, ti]
    return SceneEntity(
        frame_id=frame_id, id=entity_id,
        timestamp=now_ts(), lifetime=Duration.from_secs(0),
        triangles=[TriangleListPrimitive(points=poles_tris, color=color)],
    )

def make_ground_circle_ring(frame_id: str, entity_id: str,
                            radius: float, ring_width: float = 0.5,
                            segments: int = 256,
                            color: Color = Color(r=0.10, g=0.45, b=0.12, a=0.9),
                            z_offset: float = 0.01,
                            double_sided: bool = True) -> SceneEntity:
    r_out = max(radius, 0.001)
    r_in  = max(radius - max(ring_width, 0.001), 0.001)
    out_ring, in_ring = [], []
    for i in range(segments):
        th = (2.0 * math.pi * i) / segments
        c, s = math.cos(th), math.sin(th)
        out_ring.append(Point3(x=r_out*c, y=r_out*s, z=z_offset))
        in_ring.append (Point3(x=r_in *c, y=r_in *s, z=z_offset))
    tris = []
    for i in range(segments):
        j = (i + 1) % segments
        oi, oj = out_ring[i], out_ring[j]
        ii, ij = in_ring[i],  in_ring[j]
        tris += [oi, oj, ij]
        tris += [oi, ij, ii]
        if double_sided:
            tris += [oi, ij, oj]
            tris += [oi, ii, ij]
    return SceneEntity(
        frame_id=frame_id, id=entity_id,
        timestamp=now_ts(), lifetime=Duration.from_secs(0),
        triangles=[TriangleListPrimitive(points=tris, color=color)],
    )

# ============== Scene build ==============
def build_scene_entities():
    WORLD, NACELLE, HUB, SCAN = "world", "nacelle", "hub", "scan_ground"
    B1, B2, B3 = "blade_1", "blade_2", "blade_3"

    tower_h, tower_d = 30.0, 2.5
    tower_radius = (tower_d / 2.0) * 0.5
    nacelle_l, nacelle_w, nacelle_h = 3.6, 1.8, 1.2
    hub_d, hub_len = 1.2, 0.8
    nose_len = 1.0
    nose_base_radius = (hub_d/2.0) * 0.6

    now = now_ts()
    entities = [
        # Tower
        make_cylinder_mesh_Z(
            frame_id=WORLD, entity_id="tower_mesh",
            radius=tower_radius, height=tower_h, segments=64,
            color=Color(r=0.85, g=0.85, b=0.9, a=1.0),
        ),
        # Nacelle
        SceneEntity(
            frame_id=NACELLE, id="nacelle_geom", timestamp=now,
            lifetime=Duration.from_secs(0),
            cubes=[CubePrimitive(
                pose=Pose(position=Vector3(x=0.0, y=0.0, z=0.0),
                          orientation=Quaternion(x=0,y=0,z=0,w=1)),
                size=Vector3(x=nacelle_l, y=nacelle_w, z=nacelle_h),
                color=Color(r=0.2, g=0.6, b=0.9, a=1.0),
            )],
        ),
        # Hub + nose
        make_cylinder_mesh_X(
            frame_id=HUB, entity_id="hub_mesh",
            radius=hub_d/2.0, length=hub_len, segments=48,
            color=Color(r=0.92, g=0.92, b=0.95, a=1.0),
        ),
        make_cone_mesh_X(
            frame_id=HUB, entity_id="nose_cone_mesh",
            base_radius= nose_base_radius, length=nose_len, segments=48,
            color=Color(r=0.9, g=0.9, b=0.97, a=1.0),
        ),
        # Blades
        make_blade_entity("blade_1", "blade1", color=Color(r=0.95,g=0.2,b=0.2,a=1.0)),
        make_blade_entity("blade_2", "blade2", color=Color(r=0.2,g=0.95,b=0.2,a=1.0)),
        make_blade_entity("blade_3", "blade3", color=Color(r=0.2, g=0.2, b=0.95, a=1.0)),
        # Ground circle at tower base
        make_ground_circle_ring(
            frame_id=WORLD, entity_id="ground_circle",
            radius=1.5 * rotor_dia_m,
            ring_width=circle_ring_width_m,
            segments=circle_segments,
            color=circle_color,
            z_offset=0.01
        ),
        # SP markers in SCAN (so they follow yaw)
        make_bullseye_markers_with_labels(
            frame_id=SCAN, entity_id="sp_markers_bullseye",
            rotor_dia=rotor_dia_m, hub_ht=hub_ht_m, theta_deg=theta_deg_val,
            inner_r=marker_inner_r_m, outer_r=marker_outer_r_m, ring_w=marker_ring_w_m,
            segments=marker_segments, color=marker_color_red,
            label_offset=label_offset_m, font_size=label_font_size,
            label_color=label_color, label_bg_color=label_bg_color
        ),
    ]

    if draw_label_poles:
        pts_xy = [(-rotor_dia_m/4.0,  +rotor_dia_m*1.125),
                  (-rotor_dia_m/4.0,  +hub_ht_m*math.tan(math.radians(theta_deg_val))),
                  (-rotor_dia_m/4.0,  -rotor_dia_m*1.125),
                  ( +rotor_dia_m/4.0, -hub_ht_m*math.tan(math.radians(theta_deg_val)))]
        entities.append(make_label_poles_entity(
            frame_id=SCAN, entity_id="sp_label_poles",
            pts_xy=pts_xy, height_m=label_height_m, radius_m=label_pole_radius_m
        ))

    # TRACKER base frame has its own mast and blue top disk
    # Mast (solid black) lives in TRACKER_BASE frame, base at z=0..tracker_ht_m
    entities.append(
        make_cylinder_mesh_Z(
            frame_id=TRACKER_BASE, entity_id="tracker_mast",
            radius=tracker_radius_m, height=tracker_ht_m, segments=48,
            color=tracker_color_black
        )
    )
    # Blue solid disk marker at mast top (for easy visibility)
    tracker_top_y = rotor_dia_m * 1.125
    tracker_disk_tris = make_filled_disk_tris(0.0, 0.0, marker_outer_r_m, marker_segments)
    entities.append(SceneEntity(
        frame_id=TRACKER_BASE, id="tracker_top_disk",
        timestamp=now, lifetime=Duration.from_secs(0),
        triangles=[TriangleListPrimitive(points=tracker_disk_tris, color=tracker_color_blue)]
    ))

    return entities, tower_h, nacelle_l, nose_len

# ---- Dynamic geo labels (SPs & TRACKER) ----
def _mk_text(pose, text, size_m, color, billboard=True, bg=Color(r=1,g=1,b=1,a=0.85)):
    if not HAVE_TEXT:
        return None
    try:
        return TextPrimitive(pose=pose, text=text, font_size=size_m, color=color,
                             billboard=billboard, background_color=bg)
    except TypeError:
        try:
            return TextPrimitive(pose=pose, text=text, font_size=size_m, color=color,
                                 billboard=billboard, background=True, bg_color=bg)
        except TypeError:
            return TextPrimitive(pose=pose, text=text, font_size=size_m, color=color,
                                 billboard=billboard)

def make_sp_geo_labels_entity(frame_id_scan: str,
                              yaw_now_deg: float,
                              label_height_m: float,
                              label_offset_m: float,
                              text_size_m: float = None,
                              color: Color = None,
                              entity_id: str = "sp_geo_labels") -> SceneEntity:
    if not HAVE_TEXT or not use_dynamic_sp_labels:
        return SceneEntity(frame_id=frame_id_scan, id=entity_id, timestamp=now_ts(),
                           lifetime=Duration.from_secs(0),
                           triangles=[TriangleListPrimitive(points=[], color=Color(r=0,g=0,b=0,a=0))])

    text_size_m = text_size_m or label_font_size
    color = color or label_color

    x = rotor_dia_m / 4.0
    y_R = rotor_dia_m * 1.125
    y_T = hub_ht_m * math.tan(math.radians(theta_deg_val))
    pts = [(-x, +y_R), (-x, +y_T), (-x, -y_R), ( +x, -y_T)]
    names = ["SP1", "SP2", "SP3", "SP4"]

    geo = get_marker_geocoords(base_lat_deg, base_lon_deg, yaw_now_deg)

    texts = []
    for (px, py), name in zip(pts, names):
        r = max(math.hypot(px, py), 1e-6)
        ux, uy = px / r, py / r
        lx, ly = px + ux * label_offset_m, py + uy * label_offset_m
        lat, lon = geo[name]
        txt = f"{name}: {lat:.5f}, {lon:.5f}"
        pose = Pose(position=Vector3(x=lx, y=ly, z=label_height_m),
                    orientation=Quaternion(x=0,y=0,z=0,w=1))
        tp = _mk_text(pose, txt, text_size_m, color)
        if tp: texts.append(tp)

    return SceneEntity(frame_id=frame_id_scan, id=entity_id,
                       timestamp=now_ts(), lifetime=Duration.from_secs(0),
                       texts=texts)

def make_tracker_geo_label_entity(frame_id_world: str,
                                  label_height_m: float,
                                  label_offset_m: float,
                                  text_size_m: float = None,
                                  color: Color = None,
                                  entity_id: str = "tracker_geo_label") -> SceneEntity:
    if not HAVE_TEXT:
        return SceneEntity(frame_id=frame_id_world, id=entity_id, timestamp=now_ts(),
                           lifetime=Duration.from_secs(0),
                           triangles=[TriangleListPrimitive(points=[], color=Color(r=0,g=0,b=0,a=0))])

    text_size_m = text_size_m or label_font_size
    color = color or label_color

    # tracker base top (in WORLD it's at (0, y_R, tracker_ht_m))
    px, py = 0.0, rotor_dia_m * 1.125
    r = max(math.hypot(px, py), 1e-6)
    ux, uy = px / r, py / r
    lx, ly = px + ux * label_offset_m, py + uy * label_offset_m

    lat, lon = get_marker_geocoords(base_lat_deg, base_lon_deg, yaw_deg_now=0.0)["TRACKER"]
    txt = f"TRACKER: {lat:.5f}, {lon:.5f}"
    pose = Pose(position=Vector3(x=lx, y=ly, z=label_height_m),
                orientation=Quaternion(x=0,y=0,z=0,w=1))
    tp = _mk_text(pose, txt, text_size_m, color)

    return SceneEntity(frame_id=frame_id_world, id=entity_id,
                       timestamp=now_ts(), lifetime=Duration.from_secs(0),
                       texts=[tp] if tp else [])

# ============== Main loop ==============
if __name__ == "__main__":
    foxglove.start_server(host="127.0.0.1", port=8765)
    scene_ch = SceneUpdateChannel(topic="/scene")
    tf_ch = FrameTransformsChannel(topic="/tf")

    entities, tower_h, nacelle_l, nose_len = build_scene_entities()
    scene_ch.log(SceneUpdate(entities=entities))

    dt = 1.0 / 30.0
    t0, last_scene = time.time(), 0.0

    WORLD, NACELLE, HUB, SCAN = "world", "nacelle", "hub", "scan_ground"
    B1, B2, B3 = "blade_1", "blade_2", "blade_3"

    # --- Static root->world and world->tracker_base
    def publish_static_world_and_tracker():
        tf_ch.log(FrameTransforms(transforms=[
            FrameTransform(parent_frame_id="root", child_frame_id=WORLD,
                           translation=Vector3(x=0, y=0, z=0),
                           rotation=Quaternion(x=0,y=0,z=0,w=1)),
            # TRACKER_BASE sits at (0, y_R, 0)
            FrameTransform(parent_frame_id=WORLD, child_frame_id=TRACKER_BASE,
                           translation=Vector3(x=0.0, y=rotor_dia_m*1.125, z=0.0),
                           rotation=Quaternion(x=0,y=0,z=0,w=1)),
        ]))

    # world->nacelle and world->scan (SPs) update with yaw
    def publish_nacelle_and_scan(yaw_rad: float):
        # nacelle: yaw=0 faces +Y (offset +pi/2 keeps your convention)
        nacelle_rot = yaw_rad + math.pi/2
        # SPs: same base, plus extra configurable offset
        sp_rot = yaw_rad + math.pi/2 + math.radians(sp_yaw_offset_deg)

        tf_ch.log(FrameTransforms(transforms=[
            FrameTransform(parent_frame_id=WORLD, child_frame_id=SCAN,
                           translation=Vector3(x=0, y=0, z=0),
                           rotation=quat_from_euler(0.0, 0.0, sp_rot)),
            FrameTransform(parent_frame_id=WORLD, child_frame_id=NACELLE,
                           translation=Vector3(x=0, y=0, z=tower_h),
                           rotation=quat_from_euler(0.0, 0.0, nacelle_rot)),
            FrameTransform(parent_frame_id=NACELLE, child_frame_id=HUB,
                           translation=Vector3(x=nacelle_l*0.6, y=0, z=0),
                           rotation=Quaternion(x=0,y=0,z=0,w=1)),
        ]))

    publish_static_world_and_tracker()

    while True:
        t = time.time() - t0

        # Absolute yaw (deg -> rad)
        yaw_cmd_deg = yaw_deg + yaw_rate_deg_s * t
        yaw_cmd_rad = math.radians(yaw_cmd_deg)
        publish_nacelle_and_scan(yaw_cmd_rad)

        # --- SENSOR: pan/tilt chain with offsets; aim above hub center ---
        if sensor_enable:
            # PAN base (WORLD) is at top of tracker mast:
            # TRACKER_BASE is at (0, y_R, 0), PAN is translated by (0,0,tracker_ht_m) in that frame
            pan_base_world = (0.0, rotor_dia_m * 1.125, tracker_ht_m)

            # Current hub center (WORLD) given nacelle yaw
            nacelle_rot = yaw_cmd_rad + math.pi / 2.0
            hub_off_x = nacelle_l * 0.6
            hub_x = math.cos(nacelle_rot) * hub_off_x
            hub_y = math.sin(nacelle_rot) * hub_off_x
            hub_z = tower_h  # nacelle origin height

            # Target ABOVE hub center by hub_target_z_offset_m
            target = (hub_x, hub_y, hub_z + hub_target_z_offset_m)

            # ---------- Solve pan (about +Z at pan base) ----------
            dx0 = target[0] - pan_base_world[0]
            dy0 = target[1] - pan_base_world[1]
            # Zero pan looks along +Y; pan measured from +Y toward +X
            pan = math.atan2(dy0, dx0) - math.pi/2.0

            c, s = math.cos(pan), math.sin(pan)

            # Tilt axis position in WORLD after pan: pan_base + Rz(pan)*pt_offset
            pt = pt_offset
            tilt_world = (
                pan_base_world[0] + c * pt[0] - s * pt[1],
                pan_base_world[1] + s * pt[0] + c * pt[1],
                pan_base_world[2] + pt[2],
            )

            # Vector from tilt axis to target, expressed in PAN frame (undo pan)
            vx = target[0] - tilt_world[0]
            vy = target[1] - tilt_world[1]
            vz = target[2] - tilt_world[2]
            vx_p =  c * vx + s * vy
            vy_p = -s * vx + c * vy
            vz_p =  vz

            # ---------- Solve tilt (about +X in the pan-rotated frame) ----------
            # With tilt=0, forward is +Y. Tilting up increases +Z.
            tilt = math.atan2(vz_p, vy_p)

            # ---------- Publish TF chain: WORLD->TRACKER_BASE->PAN->TILT->SENSOR ----------
            tf_ch.log(FrameTransforms(transforms=[
                # WORLD -> TRACKER_BASE already published in static; re-send for late joiners
                FrameTransform(
                    parent_frame_id=WORLD, child_frame_id=TRACKER_BASE,
                    translation=Vector3(x=0.0, y=rotor_dia_m*1.125, z=0.0),
                    rotation=Quaternion(x=0,y=0,z=0,w=1),
                ),
                # TRACKER_BASE -> PAN (translate up to mast top, rotate about Z)
                FrameTransform(
                    parent_frame_id=TRACKER_BASE, child_frame_id=PAN,
                    translation=Vector3(x=0.0, y=0.0, z=tracker_ht_m),
                    rotation=quat_from_euler(0.0, 0.0, pan),
                ),
                # PAN -> TILT (mechanical offset, then tilt about +X)
                FrameTransform(
                    parent_frame_id=PAN, child_frame_id=TILT,
                    translation=Vector3(x=pt[0], y=pt[1], z=pt[2]),
                    rotation=quat_from_euler(tilt, 0.0, 0.0),
                ),
                # TILT -> SENSOR (small sensor mount offset, then +90° about Z to map +X->+Y)
                FrameTransform(
                    parent_frame_id=TILT, child_frame_id=SENSOR,
                    translation=Vector3(x=sensor_offset[0], y=sensor_offset[1], z=sensor_offset[2]),
                    rotation=quat_from_euler(0.0, 0.0, math.pi/2.0),
                ),
            ]))

            # Compute sensor origin in WORLD for frustum length (near->target distance)
            # Rx(tilt)*sensor_offset
            sox, soy, soz = sensor_offset
            cT, sT = math.cos(tilt), math.sin(tilt)
            sox_t = sox
            soy_t = soy * cT - soz * sT
            soz_t = soy * sT + soz * cT
            # Rz(pan)*(...) to world XY
            sox_w = c * sox_t - s * soy_t
            soy_w = s * sox_t + c * soy_t
            soz_w = soz_t
            sensor_world = (tilt_world[0] + sox_w, tilt_world[1] + soy_w, tilt_world[2] + soz_w)

            # Set frustum length to reach target
            dx = target[0] - sensor_world[0]
            dy = target[1] - sensor_world[1]
            dz = target[2] - sensor_world[2]
            dist_to_target = max(sensor_near_m + 0.1, math.sqrt(dx*dx + dy*dy + dz*dz))

            scene_ch.log(SceneUpdate(entities=[
                make_fov_frustum_X(
                    frame_id=SENSOR,
                    entity_id="sensor_fov_body",
                    near_m=sensor_near_m,
                    far_m=dist_to_target if math.isfinite(dist_to_target) else sensor_working_dist_m,
                    haov_deg=sensor_haov_deg,
                    vaov_deg=sensor_vaov_deg,
                    color=sensor_color
                )
            ]))

        # Spin blades about +X, 120° apart
        omega = rot_dir * 2.0 * math.pi * (rotor_rpm / 60.0)
        roll = omega * t
        tf_ch.log(FrameTransforms(transforms=[
            FrameTransform(parent_frame_id=HUB, child_frame_id=B1,
                           translation=Vector3(x=0,y=0,z=0),
                           rotation=quat_from_euler(roll, 0, 0)),
            FrameTransform(parent_frame_id=HUB, child_frame_id=B2,
                           translation=Vector3(x=0,y=0,z=0),
                           rotation=quat_from_euler(roll + 2.0*math.pi/3.0, 0, 0)),
            FrameTransform(parent_frame_id=HUB, child_frame_id=B3,
                           translation=Vector3(x=0,y=0,z=0),
                           rotation=quat_from_euler(roll + 4.0*math.pi/3.0, 0, 0)),
        ]))

        # Dynamic geo labels (SPs follow yaw in SCAN; TRACKER fixed in WORLD)
        if HAVE_TEXT:
            scene_ch.log(SceneUpdate(entities=[
                make_sp_geo_labels_entity(
                    frame_id_scan=SCAN,
                    yaw_now_deg=yaw_cmd_deg,
                    label_height_m=label_height_m,
                    label_offset_m=label_offset_m,
                    text_size_m=label_font_size,
                    color=label_color,
                    entity_id="sp_geo_labels"
                ),
                make_tracker_geo_label_entity(
                    frame_id_world=WORLD,
                    label_height_m=label_height_m,
                    label_offset_m=label_offset_m,
                    text_size_m=label_font_size,
                    color=label_color,
                    entity_id="tracker_geo_label"
                )
            ]))

        # Re-send static scene occasionally (helps late joins)
        now_s = time.time()
        if now_s - last_scene > 1.0:
            scene_ch.log(SceneUpdate(entities=entities))
            last_scene = now_s

        publish_static_world_and_tracker()
        time.sleep(dt)
