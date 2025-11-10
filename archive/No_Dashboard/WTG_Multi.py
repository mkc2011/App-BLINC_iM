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

# ===================== CONTROL KNOBS =====================
# --- Yaw control (absolute + optional rate) ---
yaw_deg = 0.0
yaw_rate_deg_s = 0.0

# SPs start rotated +270° at yaw=0 (your saved preference)
sp_yaw_offset_deg = 270.0

# --- Rotor / WTG geometry ---
rotor_dia_m        = 80.0      # change me; blades auto-resize
hub_ht_m           = 80.0      # tower height; nacelle Z
theta_deg_val      = 10.0

# Rotor spin
rotor_rpm = 12.0               # now used below
rot_dir   = -1.0               # +1=CCW about +X (looking forward), -1=CW

# Blade styling
blade_thickness_m  = 0.25      # change to make blades chunkier
blade_root_w_m     = 1.6
blade_tip_w_m      = 0.40

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
draw_label_poles = False
label_pole_radius_m = 0.03

# TRACKER marker & mast
tracker_color_blue     = Color(r=0.0, g=0.35, b=1.0, a=1.0)
tracker_ht_m           = 1.0
tracker_mast_radius_m  = 0.30
tracker_mast_color     = Color(r=0.0, g=0.0, b=0.0, a=1.0)

# --- Sensors at TRACKER top (same position/orientation) ---
# Mechanical stack: PAN -> (offset) -> TILT -> (offset) -> SENSOR (+X forward)
pan_to_tilt_offset    = (0.0, 0.05, 0.01)  # (x,y,z) in meters
tilt_to_sensor_offset = (0.0, 0.00, 0.01)

# Benewake (narrow; dynamic to hub)
benewake_haov_deg   = 0.35
benewake_vaov_deg   = 0.15
benewake_near_m     = 0.20
benewake_color      = Color(r=1.0, g=0.0, b=0.0, a=0.45)

# LIVOX (wide; fixed 150 m)
livox_haov_deg      = 70.4
livox_vaov_deg      = 74.2
livox_near_m        = 0.20
livox_far_m         = 150.0
livox_color         = Color(r=0.1, g=0.8, b=1.0, a=0.10)

# Targeting: aim at hub center + optional vertical offset (meters)
hub_target_z_offset_m = 0.0


# Debug helpers (optional)
DEBUG_SHOW_SENSOR_ORIGIN_CUBES = True
DEBUG_ORIGIN_CUBE_SIZE = 0.25

# Frame names
WORLD, NACELLE, HUB, SCAN = "world", "nacelle", "hub", "scan_ground"
B1, B2, B3 = "blade_1", "blade_2", "blade_3"
# Benewake frames
B_PAN, B_TILT, B_SENSOR = "benewake_pan", "benewake_tilt", "benewake_sensor"
# LIVOX frames
L_PAN, L_TILT, L_SENSOR = "livox_pan", "livox_tilt", "livox_sensor"

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

# --- tiny helper: sensor bodies so you can see them
def make_sensor_body_X(frame_id: str, entity_id: str,
                       body_len=0.40, body_r=0.08, tip_len=0.30, tip_r=0.12,
                       color_body=Color(r=0.15,g=0.15,b=0.15,a=1.0),
                       color_tip =Color(r=0.95,g=0.85,b=0.10,a=1.0)) -> list[SceneEntity]:
    return [
        make_cylinder_mesh_X(frame_id, entity_id + "__body",
                             radius=body_r, length=body_len, segments=32, color=color_body),
        make_cone_mesh_X(frame_id, entity_id + "__tip",
                         base_radius=tip_r, length=tip_len, segments=32, color=color_tip)
    ]

# ===================== SP MARKERS =====================
def make_bullseye_markers_with_labels(frame_id: str, entity_id: str) -> SceneEntity:
    theta_rad = math.radians(theta_deg_val)
    x = rotor_dia_m / 4.0
    y_R = rotor_dia_m * 1.125
    y_T = hub_ht_m * math.tan(theta_rad)

    pts = [
        (-x,  +y_R, 0.0),  # SP1
        (-x,  +y_T, 0.0),  # SP2
        (-x,  -y_R, 0.0),  # SP3
        ( +x, -y_T, 0.0),  # SP4
    ]
    labels = ["SP1", "SP2", "SP3", "SP4"]

    all_tris = []
    for (px, py, _) in pts:
        all_tris += make_filled_disk_tris(px, py, marker_inner_r_m, marker_segments)
        all_tris += make_circle_ring_tris(px, py, marker_outer_r_m, marker_ring_w_m, marker_segments)

    entity_kwargs = dict(
        frame_id=frame_id, id=entity_id, timestamp=now_ts(),
        lifetime=Duration.from_secs(0),
        triangles=[TriangleListPrimitive(points=all_tris, color=marker_color_red)],
    )

    if HAVE_TEXT:
        texts = []
        for (px, py, _), name in zip(pts, labels):
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

def publish_wtg_frames(tf_ch, yaw_rad: float, tower_h: float, nacelle_l: float):
    nacelle_rot = yaw_rad + math.pi/2
    sp_rot = yaw_rad + math.pi/2 + math.radians(sp_yaw_offset_deg)

    tf_ch.log(FrameTransforms(transforms=[
        FrameTransform(parent_frame_id=WORLD, child_frame_id=SCAN,
                       translation=Vector3(x=0, y=0, z=0),
                       rotation=quat_from_euler(0.0, 0.0, sp_rot)),
        FrameTransform(parent_frame_id=WORLD, child_frame_id=NACELLE,
                       translation=Vector3(x=0, y=0, z=tower_h),
                       rotation=quat_from_euler(0.0, 0.0, nacelle_rot)),
        FrameTransform(parent_frame_id=NACELLE, child_frame_id=HUB,
                       translation=Vector3(x=0.6*nacelle_l, y=0.0, z=0.0),
                       rotation=Quaternion(x=0,y=0,z=0,w=1)),
    ]))

def hub_center_world(yaw_rad: float, nacelle_l: float):
    """Current hub center in WORLD frame."""
    nacelle_rot = yaw_rad + math.pi/2
    hx_local, hy_local = (0.6*nacelle_l, 0.0)
    hub_x =  hx_local*math.cos(nacelle_rot) - hy_local*math.sin(nacelle_rot)
    hub_y =  hx_local*math.sin(nacelle_rot) + hy_local*math.cos(nacelle_rot)
    hub_z =  hub_ht_m
    return hub_x, hub_y, hub_z

# ===================== SCENE BUILD =====================
def build_scene_entities():
    tower_h = hub_ht_m
    nacelle_l, nacelle_w, nacelle_h = 3.6, 1.8, 1.2
    hub_d, hub_len = 1.2, 0.8
    nose_len = 1.0
    nose_base_radius = (hub_d/2.0)*0.6

    blade_len = rotor_dia_m / 2.0

    entities = [
        # Tower
        make_cylinder_mesh_Z(WORLD, "tower_mesh",
                             radius=(2.5/2.0)*0.5, height=tower_h, segments=64,
                             color=Color(r=0.85, g=0.85, b=0.9, a=1.0)),

        # Nacelle as a rectangular box
        SceneEntity(frame_id=NACELLE, id="nacelle_geom", timestamp=now_ts(),
                    lifetime=Duration.from_secs(0),
                    cubes=[CubePrimitive(
                        pose=Pose(position=Vector3(x=0,y=0,z=0),
                                  orientation=Quaternion(x=0,y=0,z=0,w=1)),
                        size=Vector3(x=nacelle_l, y=nacelle_w, z=nacelle_h),
                        color=Color(r=0.2,g=0.6,b=0.9,a=1.0)
                    )]),

        # Hub + nose cone
        make_cylinder_mesh_X(HUB, "hub_mesh", radius=hub_d/2.0, length=hub_len, segments=48,
                             color=Color(r=0.92,g=0.92,b=0.95,a=1.0)),
        make_cone_mesh_X(HUB, "nose_cone_mesh", base_radius=nose_base_radius, length=nose_len, segments=48,
                         color=Color(r=0.9, g=0.9, b=0.97, a=1.0)),

        # Blades (length tracks rotor_dia, thickness from knob)
        make_blade_entity("blade_1","blade1",
                          Lz=blade_len, root_w=blade_root_w_m, tip_w=blade_tip_w_m,
                          thickness=blade_thickness_m, color=Color(r=0.95,g=0.2,b=0.2,a=1.0)),
        make_blade_entity("blade_2","blade2",
                          Lz=blade_len, root_w=blade_root_w_m, tip_w=blade_tip_w_m,
                          thickness=blade_thickness_m, color=Color(r=0.2,g=0.95,b=0.2,a=1.0)),
        make_blade_entity("blade_3","blade3",
                          Lz=blade_len, root_w=blade_root_w_m, tip_w=blade_tip_w_m,
                          thickness=blade_thickness_m, color=Color(r=0.2,g=0.2,b=0.95,a=1.0)),

        # Ground ring scales with rotor_dia
        make_ground_circle_ring(WORLD, "ground_circle",
                                radius=1.5 * rotor_dia_m,
                                ring_width=circle_ring_width_m, segments=circle_segments,
                                color=circle_color, z_offset=0.01),

        # Tracker ground disk (moves with rotor_dia)
        SceneEntity(
            frame_id=WORLD, id="tracker_marker", timestamp=now_ts(),
            lifetime=Duration.from_secs(0),
            triangles=[TriangleListPrimitive(
                points=make_filled_disk_tris(0.0, rotor_dia_m*1.125, marker_outer_r_m, marker_segments, z=0.0),
                color=tracker_color_blue
            )],
        ),

        # Tracker mast cylinder at (x=0, y=1.125*D), height = tracker_ht_m
        make_cylinder_mesh_Z_at(WORLD, "tracker_mast",
                                radius=tracker_mast_radius_m, height=tracker_ht_m,
                                cx=0.0, cy=rotor_dia_m*1.125, segments=48, color=tracker_mast_color),

        # SPs in SCAN (so they yaw with turbine)
        make_bullseye_markers_with_labels(SCAN, "sp_markers_bullseye"),
    ]

    return entities, tower_h, nacelle_l, nose_len

# ===================== SENSOR AIM =====================
def aim_pan_tilt_from_world_delta(dx, dy, dz):
    """Pan zero points along -Y. Sensor forward is +X.
       Return (pan, tilt) radians to aim +X from origin to (dx,dy,dz)."""
    psi   = math.atan2(dy, dx)                    # yaw from +X toward target
    theta = math.atan2(dz, math.hypot(dx,dy))     # pitch about +Y (up positive)
    pan   = psi + math.pi/2                       # 0 pan = -Y
    tilt  = -theta                                # positive dz -> negative tilt (nose up)
    return pan, tilt

def sensor_origin_with_offsets(pan, tilt):
    """Origin of the SENSOR frame relative to the pan base, including both offsets."""
    p2t_x, p2t_y, p2t_z = pan_to_tilt_offset
    t2s_x, t2s_y, t2s_z = tilt_to_sensor_offset

    c, s = math.cos(pan), math.sin(pan)
    cy, sy = math.cos(tilt), math.sin(tilt)

    # pan rotates p2t in XY
    ox = p2t_x*c - p2t_y*s
    oy = p2t_x*s + p2t_y*c
    oz = p2t_z

    # tilt rotates t2s in XZ, then pan rotates XY
    t2sx =  t2s_x*cy + t2s_z*sy
    t2sz = -t2s_x*sy + t2s_z*cy
    t2sy =  t2s_y

    ox += t2sx*c - t2sy*s
    oy += t2sx*s + t2sy*c
    oz += t2sz
    return ox, oy, oz

def solve_pan_tilt_to_target(pan_base_world, target_world):
    """Tracker top is the pan base. Solve pan/tilt including mechanical offsets."""
    bx, by, bz = pan_base_world
    tx, ty, tz = target_world

    # First guess ignoring offsets
    pan, tilt = aim_pan_tilt_from_world_delta(tx - bx, ty - by, tz - bz)

    # Refine including offsets
    for _ in range(2):
        ox, oy, oz = sensor_origin_with_offsets(pan, tilt)
        sx, sy, sz = bx + ox, by + oy, bz + oz
        pan, tilt = aim_pan_tilt_from_world_delta(tx - sx, ty - sy, tz - sz)
    return pan, tilt

# ===================== MAIN =====================
if __name__ == "__main__":
    foxglove.start_server(host="127.0.0.1", port=8765)
    scene_ch = SceneUpdateChannel(topic="/scene")
    tf_ch = FrameTransformsChannel(topic="/tf")

    entities, tower_h, nacelle_l, nose_len = build_scene_entities()
    scene_ch.log(SceneUpdate(entities=entities))

    # Rebuild-on-change guard
    last_rotor = rotor_dia_m
    last_hub   = hub_ht_m
    last_theta = theta_deg_val
    last_b_thk = blade_thickness_m
    last_rootw = blade_root_w_m
    last_tipw  = blade_tip_w_m

    dt = 1.0/30.0
    t0 = time.time()
    last_scene = 0.0
    last_print = 0.0

    publish_world_root(tf_ch)

    while True:
        t = time.time() - t0

        # Live rebuild if the knobs changed
        if (rotor_dia_m != last_rotor or hub_ht_m != last_hub or
            theta_deg_val != last_theta or blade_thickness_m != last_b_thk or
            blade_root_w_m != last_rootw or blade_tip_w_m != last_tipw):
            entities, tower_h, nacelle_l, nose_len = build_scene_entities()
            scene_ch.log(SceneUpdate(entities=entities))
            last_rotor, last_hub = rotor_dia_m, hub_ht_m
            last_theta, last_b_thk = theta_deg_val, blade_thickness_m
            last_rootw, last_tipw = blade_root_w_m, blade_tip_w_m

        # Turbine yaw
        yaw_cmd_deg = yaw_deg + yaw_rate_deg_s * t
        yaw_cmd_rad = math.radians(yaw_cmd_deg)
        publish_wtg_frames(tf_ch, yaw_cmd_rad, tower_h, nacelle_l)

        # Hub target (with vertical offset)
        hub_x, hub_y, hub_z = hub_center_world(yaw_cmd_rad, nacelle_l)
        target_world = (hub_x, hub_y, hub_z + hub_target_z_offset_m)

        # Tracker top (pan base)
        pan_base_world = (0.0, rotor_dia_m*1.125, tracker_ht_m)

        # Solve common pan/tilt for both sensors
        pan_cmd, tilt_cmd = solve_pan_tilt_to_target(pan_base_world, target_world)

        # Human-readable readout (4x/s)
        now_s = time.time()
        if now_s - last_print >= 0.25:
            print(f"Pan: {wrap_deg(math.degrees(pan_cmd)):.2f} deg, "
                  f"Tilt: {wrap_deg(math.degrees(tilt_cmd)):.2f} deg  |  "
                  f"RotorDia: {rotor_dia_m:.2f} m, HubHt: {hub_ht_m:.2f} m")
            last_print = now_s

        # Publish PAN->TILT->SENSOR chains (Benewake & LIVOX share pan/tilt)
        # Base orientation yaw of -90° makes +X align with -Y when pan=0
        tf_ch.log(FrameTransforms(transforms=[
            # Benewake
            FrameTransform(parent_frame_id=WORLD, child_frame_id=B_PAN,
                           translation=Vector3(x=pan_base_world[0], y=pan_base_world[1], z=pan_base_world[2]),
                           rotation=quat_from_euler(0.0, 0.0, -math.pi/2 + pan_cmd)),
            FrameTransform(parent_frame_id=B_PAN, child_frame_id=B_TILT,
                           translation=Vector3(x=pan_to_tilt_offset[0], y=pan_to_tilt_offset[1], z=pan_to_tilt_offset[2]),
                           rotation=quat_from_euler(0.0, tilt_cmd, 0.0)),
            FrameTransform(parent_frame_id=B_TILT, child_frame_id=B_SENSOR,
                           translation=Vector3(x=tilt_to_sensor_offset[0], y=tilt_to_sensor_offset[1], z=tilt_to_sensor_offset[2]),
                           rotation=Quaternion(x=0,y=0,z=0,w=1)),
            # LIVOX (identical chain; same base, same pan/tilt)
            FrameTransform(parent_frame_id=WORLD, child_frame_id=L_PAN,
                           translation=Vector3(x=pan_base_world[0], y=pan_base_world[1], z=pan_base_world[2]),
                           rotation=quat_from_euler(0.0, 0.0, -math.pi/2 + pan_cmd)),
            FrameTransform(parent_frame_id=L_PAN, child_frame_id=L_TILT,
                           translation=Vector3(x=pan_to_tilt_offset[0], y=pan_to_tilt_offset[1], z=pan_to_tilt_offset[2]),
                           rotation=quat_from_euler(0.0, tilt_cmd, 0.0)),
            FrameTransform(parent_frame_id=L_TILT, child_frame_id=L_SENSOR,
                           translation=Vector3(x=tilt_to_sensor_offset[0], y=tilt_to_sensor_offset[1], z=tilt_to_sensor_offset[2]),
                           rotation=Quaternion(x=0,y=0,z=0,w=1)),
        ]))

        # Compute dynamic far distance to hub for Benewake (from SENSOR origin)
        ox, oy, oz = sensor_origin_with_offsets(pan_cmd, tilt_cmd)
        sx, sy, sz = pan_base_world[0] + ox, pan_base_world[1] + oy, pan_base_world[2] + oz
        dist_to_target = math.sqrt((target_world[0]-sx)**2 + (target_world[1]-sy)**2 + (target_world[2]-sz)**2)
        benewake_far = max(dist_to_target - 0.01, benewake_near_m + 1e-3)

        # Draw/update FOVs + simple sensor bodies + optional origin cubes
        entities_frame = [
            make_fov_frustum_X(B_SENSOR, "fov_benewake",
                               near_m=benewake_near_m, far_m=benewake_far,
                               haov_deg=benewake_haov_deg, vaov_deg=benewake_vaov_deg,
                               color=benewake_color),
            make_fov_frustum_X(L_SENSOR, "fov_livox",
                               near_m=livox_near_m, far_m=livox_far_m,
                               haov_deg=livox_haov_deg, vaov_deg=livox_vaov_deg,
                               color=livox_color),
            *make_sensor_body_X(B_SENSOR, "benewake_body"),
            *make_sensor_body_X(L_SENSOR, "livox_body"),
        ]
        if DEBUG_SHOW_SENSOR_ORIGIN_CUBES:
            entities_frame += [
                SceneEntity(frame_id=B_SENSOR, id="benewake_origin", timestamp=now_ts(),
                            lifetime=Duration.from_secs(0),
                            cubes=[CubePrimitive(
                                pose=Pose(position=Vector3(x=0,y=0,z=0),
                                          orientation=Quaternion(x=0,y=0,z=0,w=1)),
                                size=Vector3(x=DEBUG_ORIGIN_CUBE_SIZE, y=DEBUG_ORIGIN_CUBE_SIZE, z=DEBUG_ORIGIN_CUBE_SIZE),
                                color=Color(r=1.0,g=0.0,b=1.0,a=1.0)
                            )]),
                SceneEntity(frame_id=L_SENSOR, id="livox_origin", timestamp=now_ts(),
                            lifetime=Duration.from_secs(0),
                            cubes=[CubePrimitive(
                                pose=Pose(position=Vector3(x=0,y=0,z=0),
                                          orientation=Quaternion(x=0,y=0,z=0,w=1)),
                                size=Vector3(x=DEBUG_ORIGIN_CUBE_SIZE, y=DEBUG_ORIGIN_CUBE_SIZE, z=DEBUG_ORIGIN_CUBE_SIZE),
                                color=Color(r=1.0,g=0.0,b=1.0,a=1.0)
                            )]),
            ]
        scene_ch.log(SceneUpdate(entities=entities_frame))

        # Spin blades about +X, 120° apart (now uses knobs)
        omega = rot_dir * 2.0*math.pi * (rotor_rpm/60.0)
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

        # Periodically resend static scene (late joiners)
        if now_s - last_scene > 1.0:
            scene_ch.log(SceneUpdate(entities=entities))
            last_scene = now_s

        publish_world_root(tf_ch)
        time.sleep(1.0/30.0)
