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
yaw_deg = 0.0         # 0° means nacelle faces world +Y (kept by +pi/2 offset below)
yaw_rate_deg_s = 0.0  # deg/s

# SPs start rotated +270° at yaw=0 (your saved preference)
sp_yaw_offset_deg = 270.0

# --- Rotor control ---
rotor_rpm = 12.0
rot_dir   = -1.0      # +1=CCW about +X (looking forward), -1=CW

# --- Geometry ---
rotor_dia_m   = 36.0
hub_ht_m      = 80.0   # informational; actual NACELLE TF uses tower_h below
theta_deg_val = 10.0

# Ground circle ring
circle_ring_width_m = 0.6
circle_segments     = 256
circle_color        = Color(r=0.10, g=0.45, b=0.12, a=0.9)  # visible dark green

# SP bullseye styling
marker_inner_r_m = 0.45
marker_outer_r_m = 0.90   # OUTER radius (so diameter = 2*0.90)
marker_ring_w_m  = 0.10
marker_segments  = 96
marker_color_red = Color(r=1.0, g=0.0, b=0.0, a=1.0)

# Label styling (optional if TextPrimitive exists; only SP names here)
label_height_m   = 2.0
label_offset_m   = 1.0
label_font_size  = 1.2
label_color      = Color(r=0.0, g=0.0, b=0.0, a=1.0)     # black text
label_bg_color   = Color(r=1.0, g=1.0, b=1.0, a=1.0)     # white background
label_billboard  = True

# Optional thin poles under labels (these yaw with SPs since they’re in SCAN)
draw_label_poles    = True
label_pole_radius_m = 0.03

# TRACKER marker (fixed in world)
tracker_color_blue  = Color(r=0.0, g=0.35, b=1.0, a=1.0)
tracker_ht_m        = 1.0   # TRACKER mast height above ground

# TRACKER mast cylinder
tracker_mast_radius_m = 0.30
tracker_mast_color    = Color(r=0.0, g=0.0, b=0.0, a=1.0)  # opaque black

# --- Sensor FOV (anchored at TRACKER) ---
sensor_enable            = True
sensor_haov_deg          = 0.35   # Horizontal AOV (full angle)
sensor_vaov_deg          = 0.15   # Vertical AOV (full angle)
sensor_working_dist_m    = 120.0  # Far distance (extend length)
sensor_near_m            = 0.20   # Small near plane to avoid degenerate apex
sensor_color             = Color(r=1.0, g=0.0, b=0.0, a=0.35)  # mild transparent red

# Mechanical stack: PAN -> (offset) -> TILT -> (offset) -> SENSOR (+X forward)
pan_to_tilt_offset   = (0.0, 0.05, 0.01)  # (x,y,z) in meters
tilt_to_sensor_offset= (0.0, 0.0, 0.01)   # (x,y,z) in meters

# Targeting: aim at hub center plus optional vertical offset (meters)
hub_target_z_offset_m = 0.0  # set to 20.0 to aim 20 m above hub center, etc.

# Frame names for sensor chain
PAN, TILT, SENSOR = "pan_stage", "tilt_stage", "sensor_tracker"

# ============== Helpers ==============
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

def make_cylinder_mesh_Z(frame_id, entity_id, radius, height, segments=64,
                         color=Color(r=0.85, g=0.85, b=0.9, a=1.0)) -> SceneEntity:
    top_z, bot_z = height, 0.0
    verts_top, verts_bot = [], []
    for i in range(segments):
        th = (2.0 * math.pi * i) / segments
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
        tris += [bi, bj, tj]
        tris += [bi, tj, ti]
    for i in range(segments):
        j = (i + 1) % segments
        ti, tj = verts_top[i], verts_top[j]
        tris += [top_center, ti, tj]
    for i in range(segments):
        j = (i + 1) % segments
        bi, bj = verts_bot[i], verts_bot[j]
        tris += [bot_center, bj, bi]
    return SceneEntity(
        frame_id=frame_id, id=entity_id,
        timestamp=now_ts(),
        lifetime=Duration.from_secs(0),
        triangles=[TriangleListPrimitive(points=tris, color=color)],
    )

def make_cylinder_mesh_Z_at(frame_id, entity_id, radius, height, cx, cy,
                            segments=64, color=Color(r=0, g=0, b=0, a=1.0)) -> SceneEntity:
    """Vertical cylinder whose axis is at (cx, cy), from z=0 to z=height."""
    top_z, bot_z = height, 0.0
    verts_top, verts_bot = [], []
    for i in range(segments):
        th = (2.0 * math.pi * i) / segments
        x = cx + radius * math.cos(th)
        y = cy + radius * math.sin(th)
        verts_top.append(Point3(x=x, y=y, z=top_z))
        verts_bot.append(Point3(x=x, y=y, z=bot_z))
    top_center = Point3(x=cx, y=cy, z=top_z)
    bot_center = Point3(x=cx, y=cy, z=bot_z)
    tris = []
    for i in range(segments):
        j = (i + 1) % segments
        bi, bj = verts_bot[i], verts_bot[j]
        ti, tj = verts_top[i], verts_top[j]
        tris += [bi, bj, tj]
        tris += [bi, tj, ti]
    for i in range(segments):
        j = (i + 1) % segments
        ti, tj = verts_top[i], verts_top[j]
        tris += [top_center, ti, tj]
    for i in range(segments):
        j = (i + 1) % segments
        bi, bj = verts_bot[i], verts_bot[j]
        tris += [bot_center, bj, bi]
    return SceneEntity(
        frame_id=frame_id, id=entity_id,
        timestamp=now_ts(), lifetime=Duration.from_secs(0),
        triangles=[TriangleListPrimitive(points=tris, color=color)],
    )

def make_cylinder_mesh_X(frame_id, entity_id, radius, length, segments=48,
                         color=Color(r=0.9, g=0.9, b=0.95, a=1.0)) -> SceneEntity:
    xL = -length/2.0
    xR = +length/2.0
    ring_L, ring_R = [], []
    for i in range(segments):
        th = (2.0 * math.pi * i) / segments
        y = radius * math.cos(th); z = radius * math.sin(th)
        ring_L.append(Point3(x=xL, y=y, z=z))
        ring_R.append(Point3(x=xR, y=y, z=z))
    center_L = Point3(x=xL, y=0.0, z=0.0)
    center_R = Point3(x=xR, y=0.0, z=0.0)

    tris = []
    for i in range(segments):
        j = (i + 1) % segments
        li, lj = ring_L[i], ring_L[j]
        ri, rj = ring_R[i], ring_R[j]
        tris += [li, lj, rj]
        tris += [li, rj, ri]
    for i in range(segments):
        j = (i + 1) % segments
        ri, rj = ring_R[i], ring_R[j]
        tris += [center_R, ri, rj]
    for i in range(segments):
        j = (i + 1) % segments
        li, lj = ring_L[i], ring_L[j]
        tris += [center_L, lj, li]
    return SceneEntity(
        frame_id=frame_id, id=entity_id,
        timestamp=now_ts(),
        lifetime=Duration.from_secs(0),
        triangles=[TriangleListPrimitive(points=tris, color=color)],
    )

def make_cone_mesh_X(frame_id, entity_id, base_radius, length, segments=48,
                     color=Color(r=0.88, g=0.9, b=0.95, a=1.0)) -> SceneEntity:
    x_base = 0.0
    x_tip  = length
    ring = []
    for i in range(segments):
        th = (2.0 * math.pi * i) / segments
        y = base_radius * math.cos(th); z = base_radius * math.sin(th)
        ring.append(Point3(x=x_base, y=y, z=z))
    center_base = Point3(x=x_base, y=0.0, z=0.0)
    tip = Point3(x=x_tip, y=0.0, z=0.0)
    tris = []
    for i in range(segments):
        j = (i + 1) % segments
        vi, vj = ring[i], ring[j]
        tris += [vi, vj, tip]
    for i in range(segments):
        j = (i + 1) % segments
        vi, vj = ring[i], ring[j]
        tris += [center_base, vj, vi]
    return SceneEntity(
        frame_id=frame_id, id=entity_id,
        timestamp=now_ts(),
        lifetime=Duration.from_secs(0),
        triangles=[TriangleListPrimitive(points=tris, color=color)],
    )

# -------- Blade (triangular prism) along +Z; spins about +X --------
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
    return SceneEntity(
        frame_id=frame_id, id=entity_id,
        timestamp=now_ts(), lifetime=Duration.from_secs(0),
        triangles=[TriangleListPrimitive(points=tris, color=color)],
    )

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
    tris += [n1, f2, f1]; tris += [n1, n2, f2]   # -Z
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

    if HAVE_TEXT:
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

# ============== Scene build ==============
def build_scene_entities():
    WORLD, NACELLE, HUB, SCAN = "world", "nacelle", "hub", "scan_ground"
    B1, B2, B3 = "blade_1", "blade_2", "blade_3"

    tower_h, tower_d = 30.0, 2.5          # << NACELLE TF uses tower_h as Z (consistency!)
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
    ]

    # SP markers authored in SCAN so they follow yaw
    bullseyes = make_bullseye_markers_with_labels(
        frame_id=SCAN, entity_id="sp_markers_bullseye",
        rotor_dia=rotor_dia_m, hub_ht=hub_ht_m, theta_deg=theta_deg_val,
        inner_r=marker_inner_r_m, outer_r=marker_outer_r_m, ring_w=marker_ring_w_m,
        segments=marker_segments, color=marker_color_red,
        label_offset=label_offset_m, font_size=label_font_size,
        label_color=label_color, label_bg_color=label_bg_color
    )
    entities.append(bullseyes)

    if draw_label_poles:
        pts_xy = [(-rotor_dia_m/4.0,  +rotor_dia_m*1.125),
                  (-rotor_dia_m/4.0,  +hub_ht_m*math.tan(math.radians(theta_deg_val))),
                  (-rotor_dia_m/4.0,  -rotor_dia_m*1.125),
                  ( +rotor_dia_m/4.0, -hub_ht_m*math.tan(math.radians(theta_deg_val)))]
        entities.append(make_label_poles_entity(
            frame_id=SCAN, entity_id="sp_label_poles",
            pts_xy=pts_xy, height_m=label_height_m, radius_m=label_pole_radius_m
        ))

    # TRACKER: solid blue disk at (0, R*1.125), z=0
    px, py = 0.0, rotor_dia_m * 1.125
    tracker_tris = make_filled_disk_tris(px, py, marker_outer_r_m, marker_segments)
    entities.append(SceneEntity(
        frame_id=WORLD, id="tracker_marker", timestamp=now_ts(),
        lifetime=Duration.from_secs(0),
        triangles=[TriangleListPrimitive(points=tracker_tris, color=tracker_color_blue)],
    ))

    # TRACKER mast: opaque black cylinder of height = tracker_ht_m, radius = 0.3
    entities.append(make_cylinder_mesh_Z_at(
        frame_id=WORLD, entity_id="tracker_mast",
        radius=tracker_mast_radius_m, height=tracker_ht_m,
        cx=0.0, cy=py, segments=48, color=tracker_mast_color
    ))

    return entities, tower_h, nacelle_l, nose_len

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

    def publish_root_world():
        tf_ch.log(FrameTransforms(transforms=[
            FrameTransform(parent_frame_id="root", child_frame_id=WORLD,
                           translation=Vector3(x=0, y=0, z=0),
                           rotation=Quaternion(x=0,y=0,z=0,w=1)),
        ]))

    def publish_nacelle_and_scan(yaw_rad: float):
        # nacelle: yaw=0 faces +Y (keep +pi/2 convention)
        nacelle_rot = yaw_rad + math.pi/2
        # SPs: same base, plus your saved +270° offset
        sp_rot = yaw_rad + math.pi/2 + math.radians(sp_yaw_offset_deg)

        tf_ch.log(FrameTransforms(transforms=[
            # SPs yaw about ground with extra offset baked in
            FrameTransform(parent_frame_id=WORLD, child_frame_id=SCAN,
                           translation=Vector3(x=0, y=0, z=0),
                           rotation=quat_from_euler(0.0, 0.0, sp_rot)),

            # Nacelle yaw at tower top (use tower_h for Z!)
            FrameTransform(parent_frame_id=WORLD, child_frame_id=NACELLE,
                           translation=Vector3(x=0, y=0, z=tower_h),
                           rotation=quat_from_euler(0.0, 0.0, nacelle_rot)),

            # Hub forward +X in nacelle frame
            FrameTransform(parent_frame_id=NACELLE, child_frame_id=HUB,
                           translation=Vector3(x=nacelle_l*0.6, y=0, z=0),
                           rotation=Quaternion(x=0, y=0, z=0, w=1)),
        ]))

    publish_root_world()

    while True:
        t = time.time() - t0

        # Absolute yaw (deg -> rad)
        yaw_cmd_deg = yaw_deg + yaw_rate_deg_s * t
        yaw_cmd_rad = math.radians(yaw_cmd_deg)
        publish_nacelle_and_scan(yaw_cmd_rad)

        # Rotor phase
        omega = rot_dir * 2.0 * math.pi * (rotor_rpm / 60.0)
        roll = omega * t

        # --- SENSOR: PAN→TILT→SENSOR chain anchored at TRACKER top ---
        # Tracker position (WORLD). Keep your existing tracker_y and tracker_ht.
        tracker_x = 0.0
        tracker_y = rotor_dia_m * 1.125
        tracker_z = tracker_ht_m  # top of mast

        # Compute current hub center in WORLD (consistent with NACELLE TF!)
        nacelle_rot = yaw_cmd_rad + math.pi/2
        hx_local, hy_local = (nacelle_l*0.6, 0.0)
        hub_x =  hx_local*math.cos(nacelle_rot) - hy_local*math.sin(nacelle_rot)
        hub_y =  hx_local*math.sin(nacelle_rot) + hy_local*math.cos(nacelle_rot)
        hub_z =  tower_h

        # Aim point (hub + vertical offset)
        target_world = (hub_x, hub_y, hub_z + hub_target_z_offset_m)

        # Direction from pan base (TRACKER top) to target
        dx = target_world[0] - tracker_x
        dy = target_world[1] - tracker_y
        dz = target_world[2] - tracker_z

        # Yaw/pitch that align +X with the target
        psi   = math.atan2(dy, dx)                         # yaw from +X
        theta = math.atan2(dz, math.hypot(dx, dy))         # pitch about +Y

        # --- Rebased pan: 0° = look along −Y (inline to turbine from tracker)
        # Angle from −Y is psi + 90°.
        pan_motor_rad  = psi + math.pi/2
        tilt_motor_rad = -theta

        # Human-friendly readout
        print(f"Pan: {wrap_deg(math.degrees(pan_motor_rad)):.2f} deg, "
              f"Tilt: {-math.degrees(tilt_motor_rad):.2f} deg")

        # Publish PAN -> TILT -> SENSOR chain with mechanical offsets
        tf_ch.log(FrameTransforms(transforms=[
            # WORLD -> PAN
            # Base orientation yaw of -90° makes +X align with -Y when pan=0
            FrameTransform(
                parent_frame_id=WORLD, child_frame_id=PAN,
                translation=Vector3(x=tracker_x, y=tracker_y, z=tracker_z),
                rotation=quat_from_euler(0.0, 0.0, -math.pi/2 + pan_motor_rad),
            ),
            # PAN -> TILT (apply your real offsets, then tilt about +Y)
            FrameTransform(
                parent_frame_id=PAN, child_frame_id=TILT,
                translation=Vector3(x=pan_to_tilt_offset[0],
                                    y=pan_to_tilt_offset[1],
                                    z=pan_to_tilt_offset[2]),
                rotation=quat_from_euler(0.0, tilt_motor_rad, 0.0),
            ),
            # TILT -> SENSOR (apply sensor offset, no extra rotation)
            FrameTransform(
                parent_frame_id=TILT, child_frame_id=SENSOR,
                translation=Vector3(x=tilt_to_sensor_offset[0],
                                    y=tilt_to_sensor_offset[1],
                                    z=tilt_to_sensor_offset[2]),
                rotation=Quaternion(x=0, y=0, z=0, w=1),
            ),
        ]))

        # Draw/update the translucent red FOV frustum (points along +X of SENSOR)
        if sensor_enable:
            scene_ch.log(SceneUpdate(entities=[
                make_fov_frustum_X(
                    frame_id=SENSOR,
                    entity_id="sensor_fov_body",
                    near_m=sensor_near_m,
                    far_m=sensor_working_dist_m,
                    haov_deg=sensor_haov_deg,
                    vaov_deg=sensor_vaov_deg,
                    color=sensor_color
                )
            ]))

        # Spin blades about +X, 120° apart
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

        # Re-send scene occasionally (helps late joins)
        now_s = time.time()
        if now_s - last_scene > 1.0:
            scene_ch.log(SceneUpdate(entities=entities))
            last_scene = now_s

        publish_root_world()
        time.sleep(dt)
