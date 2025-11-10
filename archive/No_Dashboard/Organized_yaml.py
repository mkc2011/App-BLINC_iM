import math, time, datetime, os
import yaml  # pip install pyyaml

import foxglove
from foxglove.channels import SceneUpdateChannel, FrameTransformsChannel
from foxglove.schemas import (
    Color, CubePrimitive, Duration,
    FrameTransform, FrameTransforms, Pose, Quaternion,
    SceneEntity, SceneUpdate, Timestamp, Vector3,
    TriangleListPrimitive, Point3,
)

# Optional labels (not used for HUD here)
try:
    from foxglove.schemas import TextPrimitive
    HAVE_TEXT = True
except Exception:
    HAVE_TEXT = False

# ===================== WTG / SCENE CONTROLS =====================
yaw_deg = 0.0
yaw_rate_deg_s = 0.0
sp_yaw_offset_deg = 270.0   # SPs start 270° offset at yaw=0 (your saved pref)

rotor_rpm = 12.0
rot_dir   = -1.0

# Tunables that should update geometry live
rotor_dia_m   = 80.0
hub_ht_m      = 80.0
theta_deg_val = 10.0

# Tracker stand
tracker_ht_m  = 1.0
tracker_post_radius_m = 0.3

# Ground ring
circle_ring_width_m = 0.6
circle_segments     = 256
circle_color        = Color(r=0.10, g=0.45, b=0.12, a=0.9)

# SP bullseyes
marker_inner_r_m = 0.45
marker_outer_r_m = 0.90
marker_ring_w_m  = 0.10
marker_segments  = 96
marker_color_red = Color(r=1.0, g=0.0, b=0.0, a=1.0)

# Labels (kept OFF to reduce clutter)
label_height_m   = 2.0
label_offset_m   = 1.0
label_font_size  = 1.0
label_color      = Color(r=0.0, g=0.0, b=0.0, a=1.0)
label_bg_color   = Color(r=1.0, g=1.0, b=1.0, a=0.9)
label_billboard  = True
draw_label_poles = False

# Debug helpers
DEBUG_SHOW_SENSOR_ORIGIN_CUBES = True
DEBUG_ORIGIN_CUBE_SIZE = 0.25

# Force-aim all sensors to the hub (overrides YAML aim settings)
FORCE_AIM_ALL_SENSORS_TO_HUB = True

# Terminal print throttle for pan/tilt
PRINT_EVERY_SEC = 0.5
ANGLE_PRINT_EPS_DEG = 0.05
_last_print = {}  # key -> (t_last, pan_deg, tilt_deg)

# ===================== UTILITIES =====================
def now_ts() -> Timestamp:
    return Timestamp.from_datetime(datetime.datetime.utcnow())

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

def norm3(x,y,z): return math.sqrt(x*x+y*y+z*z)

def sanitize(s):
    return ''.join((ch.lower() if ch.isalnum() else '_') for ch in s)


def floatify(v):
    """Parse numbers even if YAML uses comma decimal (e.g., '2,8547')."""
    if isinstance(v, (int, float)): return float(v)
    s = str(v).strip().replace(',', '.')
    return float(s)

# ===================== BASIC GEOMETRY =====================
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

def make_cylinder_mesh_Z(frame_id, entity_id, radius, height, segments=64,
                         color=Color(r=0.85, g=0.85, b=0.9, a=1.0),
                         cx=0.0, cy=0.0, z0=0.0) -> SceneEntity:
    top_z, bot_z = z0 + height, z0
    verts_top, verts_bot = [], []
    for i in range(segments):
        th = 2.0*math.pi*i/segments
        x = cx + radius*math.cos(th); y = cy + radius*math.sin(th)
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
    hx = thickness/2.0
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

def make_sensor_body_X(frame_id: str, entity_id: str,
                       body_len=0.40, body_r=0.08, tip_len=0.30, tip_r=0.12,
                       color_body=Color(r=0.15,g=0.15,b=0.15,a=1.0),
                       color_tip =Color(r=0.95,g=0.85,b=0.10,a=1.0)) -> list[SceneEntity]:
    ents = []
    ents.append(make_cylinder_mesh_X(frame_id, entity_id + "__body",
                                     radius=body_r, length=body_len, segments=32, color=color_body))
    ents.append(make_cone_mesh_X(frame_id, entity_id + "__tip",
                                 base_radius=tip_r, length=tip_len, segments=32, color=color_tip))
    return ents

# ===================== SP MARKERS & RING =====================
def make_bullseye_markers(frame_id: str, entity_id: str) -> SceneEntity:
    x = rotor_dia_m/4.0
    yR = rotor_dia_m*1.125
    yT = hub_ht_m*math.tan(math.radians(theta_deg_val))
    pts = [(-x, +yR, 0.0), (-x, +yT, 0.0), (-x, -yR, 0.0), ( +x, -yT, 0.0)]
    all_tris=[]
    for (px,py,pz) in pts:
        all_tris += make_filled_disk_tris(px,py,marker_inner_r_m, marker_segments, z=0.0)
        all_tris += make_circle_ring_tris(px,py,marker_outer_r_m, marker_ring_w_m, marker_segments, z=0.0)
    return SceneEntity(frame_id=frame_id, id=entity_id, timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=all_tris, color=marker_color_red)])

def make_ground_circle_ring(frame_id: str, entity_id: str,
                            radius: float, ring_width: float, segments: int,
                            color: Color, z_offset=0.01) -> SceneEntity:
    tris = make_circle_ring_tris(0.0,0.0,radius, ring_width, segments, z=z_offset)
    return SceneEntity(frame_id=frame_id, id=entity_id, timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=tris, color=color)])

# ===================== YAML (defaults) =====================
DEFAULT_SENSORS_YAML = """
meta:
  units: { position: meters, angles: degrees }
  conventions:
    sensor_forward_axis: +X
    rotation_order: pan_then_tilt
    pan_axis: +Z
    tilt_axis: +Y
    pan_zero_world_direction: -Y
    tilt_zero_world_direction: +0Z

systems:
  - name: Tracker
    frame: world
    origin:
      x: 0.0
      y: "{{ rotor_dia_m * 1.125 }}"
      z: "{{ tracker_ht_m }}"
    sensors:
      - name: Benewake
        type: single-line-laser
        color_rgba: [1.0, 0.0, 0.0, 0.55]
        mounts:
          pan_to_tilt_offset:    { x: 0.0,  y: 0.05, z: 0.01 }
          tilt_to_sensor_offset: { x: 0.0,  y: 0.00, z: 0.01 }
        orientation: { pan_deg: 0.0, tilt_deg: 0.0 }
        fov: { haov_deg: 0.35, vaov_deg: 0.15 }
        range: { near_m: 0.20, far_m: dynamic_to_target }
        aim:
          target: hub_center
          target_offset: { x: 0.0, y: 0.0, z: 0.0 }

  - name: Scanner
    frame: world
    origin: { x: -15.0, y: 10.0, z: 2.0 }
    sensors:
      - name: Camera
        type: rgb_camera
        color_rgba: [1.0, 1.0, 1.0, 0.35]
        mounts:
          pan_to_tilt_offset:    { x: 0.0,  y: 0.00, z: 0.00 }
          tilt_to_sensor_offset: { x: 0.0,  y: 0.00, z: 0.00 }
        orientation: { pan_deg: 0.0, tilt_deg: 0.0 }
        fov: { haov_deg: 2.8547, vaov_deg: 2.1388 }
        range: { near_m: 0.20, far_m: dynamic_to_target }
        aim:
          target: hub_center
          target_offset: { x: 0.0, y: 0.0, z: 0.0 }

      - name: S-Laser
        type: single-line-laser
        color_rgba: [1.0, 0.4, 0.0, 0.55]
        mounts:
          pan_to_tilt_offset:    { x: 0.0,  y: 0.03, z: 0.01 }
          tilt_to_sensor_offset: { x: 0.0,  y: 0.00, z: 0.01 }
        orientation: { pan_deg: 0.0, tilt_deg: 0.0 }
        fov: { haov_deg: 0.35, vaov_deg: 0.15 }
        range: { near_m: 0.20, far_m: dynamic_to_target }
        aim:
          target: hub_center
          target_offset: { x: 0.0, y: 0.0, z: 0.0 }
"""

def _eval_template(val, context):
    if isinstance(val, str) and val.strip().startswith("{{") and val.strip().endswith("}}"):
        expr = val.strip()[2:-2].strip()
        return eval(expr, {"__builtins__": {}}, context)
    return val

def load_config():
    cfg_path = "sensors.yaml"
    if os.path.exists(cfg_path):
        with open(cfg_path, "r") as f:
            cfg = yaml.safe_load(f)
    else:
        cfg = yaml.safe_load(DEFAULT_SENSORS_YAML)
    refs = {
        "rotor_dia_m": rotor_dia_m,
        "hub_ht_m": hub_ht_m,
        "tracker_ht_m": tracker_ht_m,
        "math": math,
    }
    for sys in cfg.get("systems", []):
        org = sys.get("origin", {})
        for k in ("x","y","z"):
            org[k] = _eval_template(org.get(k), refs)
        sys["origin"] = org
    return cfg

# ===================== WTG & FRAMES =====================
def world_of_hub_center(yaw_rad, nacelle_l):
    # hub center offset along +X of nacelle, with yaw offset so yaw=0 faces +Y
    nacelle_rot = yaw_rad + math.pi/2
    off = 0.6*nacelle_l
    ox, oy = off*math.cos(nacelle_rot), off*math.sin(nacelle_rot)
    return ox, oy, hub_ht_m

def publish_wtg_tfs(tf_ch, yaw_rad, tower_h, nacelle_l):
    WORLD, NACELLE, HUB = "world","nacelle","hub"
    nacelle_rot = yaw_rad + math.pi/2
    tf_ch.log(FrameTransforms(transforms=[
        FrameTransform(parent_frame_id="root", child_frame_id=WORLD,
                       translation=Vector3(x=0,y=0,z=0),
                       rotation=Quaternion(x=0,y=0,z=0,w=1)),
        FrameTransform(parent_frame_id=WORLD, child_frame_id="scan_ground",
                       translation=Vector3(x=0,y=0,z=0),
                       rotation=quat_from_euler(0,0, yaw_rad + math.pi/2 + math.radians(sp_yaw_offset_deg))),
        FrameTransform(parent_frame_id=WORLD, child_frame_id=NACELLE,
                       translation=Vector3(x=0,y=0,z=tower_h),
                       rotation=quat_from_euler(0,0, nacelle_rot)),
        FrameTransform(parent_frame_id=NACELLE, child_frame_id=HUB,
                       translation=Vector3(x=0.6*nacelle_l, y=0, z=0),
                       rotation=Quaternion(x=0,y=0,z=0,w=1)),
    ]))

def publish_system_frames(tf_ch, systems_cfg):
    transforms = []
    for sys in systems_cfg:
        fr = f"sys_{sanitize(sys['name'])}"
        org = sys["origin"]
        transforms.append(
            FrameTransform(parent_frame_id="world", child_frame_id=fr,
                           translation=Vector3(x=float(org["x"]), y=float(org["y"]), z=float(org["z"])),
                           rotation=Quaternion(x=0,y=0,z=0,w=1))
        )
    if transforms:
        tf_ch.log(FrameTransforms(transforms=transforms))

# ===================== SCENE BUILD =====================
def make_bullseyes_entity(): return make_bullseye_markers("scan_ground", "sp_markers_bullseye")

def build_scene_entities():
    WORLD, NACELLE, HUB = "world","nacelle","hub"
    B1,B2,B3 = "blade_1","blade_2","blade_3"

    tower_h, tower_d = hub_ht_m, 2.5
    tower_radius = (tower_d/2.0)*0.5
    nacelle_l, nacelle_w, nacelle_h = 3.6, 1.8, 1.2
    hub_d, hub_len = 1.2, 0.8
    nose_len = 1.0
    nose_base_radius = (hub_d/2.0)*0.6

    ents = [
        # Tower at hub_ht_m
        make_cylinder_mesh_Z(WORLD, "tower_mesh", tower_radius, tower_h, 64,
                             Color(r=0.85,g=0.85,b=0.9,a=1.0), cx=0.0, cy=0.0, z0=0.0),
        # Nacelle, hub & nose (in their frames)
        SceneEntity(frame_id=NACELLE, id="nacelle_geom", timestamp=now_ts(),
                    lifetime=Duration.from_secs(0),
                    cubes=[CubePrimitive(
                        pose=Pose(position=Vector3(x=0,y=0,z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)),
                        size=Vector3(x=nacelle_l, y=nacelle_w, z=nacelle_h),
                        color=Color(r=0.2,g=0.6,b=0.9,a=1.0)
                    )]),
        make_cylinder_mesh_X("hub", "hub_mesh", radius=hub_d/2.0, length=hub_len, segments=48,
                             color=Color(r=0.92,g=0.92,b=0.95,a=1.0)),
        make_cone_mesh_X("hub", "nose_cone_mesh", base_radius=nose_base_radius, length=nose_len, segments=48,
                         color=Color(r=0.9,g=0.9,b=0.97,a=1.0)),
        make_blade_entity("blade_1","blade1", color=Color(r=0.95,g=0.2,b=0.2,a=1.0)),
        make_blade_entity("blade_2","blade2", color=Color(r=0.2,g=0.95,b=0.2,a=1.0)),
        make_blade_entity("blade_3","blade3", color=Color(r=0.2,g=0.2,b=0.95,a=1.0)),
        # Ground ring
        make_ground_circle_ring(WORLD, "ground_circle", radius=1.5*rotor_dia_m,
                                ring_width=circle_ring_width_m, segments=circle_segments,
                                color=circle_color, z_offset=0.01),
        # Tracker post at (0, 1.125*D, 0) height tracker_ht_m
        make_cylinder_mesh_Z(WORLD, "tracker_post", tracker_post_radius_m, tracker_ht_m, 48,
                             color=Color(r=0.0,g=0.0,b=0.0,a=1.0),
                             cx=0.0, cy=rotor_dia_m*1.125, z0=0.0),
        # Tracker ground disk (blue)
        SceneEntity(
            frame_id=WORLD, id="tracker_marker", timestamp=now_ts(),
            lifetime=Duration.from_secs(0),
            triangles=[TriangleListPrimitive(
                points=make_filled_disk_tris(0.0, rotor_dia_m*1.125, marker_outer_r_m, marker_segments, z=0.0),
                color=Color(r=0.0,g=0.35,b=1.0,a=1.0)
            )],
        ),
        # SPs (in yaw-following ground frame)
        make_bullseyes_entity(),
    ]
    return ents, tower_h, nacelle_l, nose_len

# ===================== AIM / PAN-TILT =====================
def aim_pan_tilt_from_world_delta(dx, dy, dz):
    """
    pan = 0 => look along +Y (parallel to ground)
    +tilt should visually pitch UP, but since +Y rotation pitches DOWN,
    we negate the geometric pitch here.
    """
    pan  = math.atan2(dy, dx) - math.pi/2          # align 0 pan to +Y
    tilt = -math.atan2(dz, math.hypot(dx, dy))     # negate so +dz -> visual nose up
    return pan, tilt




def _print_pan_tilt(system_name, sensor_name, pan_rad, tilt_rad):
    key = f"{system_name}/{sensor_name}"
    tnow = time.time()
    pan_deg = math.degrees(pan_rad)
    tilt_deg = math.degrees(tilt_rad)
    prev = _last_print.get(key)
    if (prev is None or
        tnow - prev[0] >= PRINT_EVERY_SEC or
        abs(prev[1]-pan_deg) > ANGLE_PRINT_EPS_DEG or
        abs(prev[2]-tilt_deg) > ANGLE_PRINT_EPS_DEG):
        print(f"[{key}] pan={pan_deg:+.2f} deg, tilt={tilt_deg:+.2f} deg")
        _last_print[key] = (tnow, pan_deg, tilt_deg)

def publish_sensor_chain(scene_ch, tf_ch, system_name, sensor_cfg, hub_target_world):
    sys_frame = f"sys_{sanitize(system_name)}"
    sname = sensor_cfg["name"]
    base = f"{sanitize(system_name)}__{sanitize(sname)}"
    pan_frame  = base + "__pan"
    tilt_frame = base + "__tilt"
    sens_frame = base + "__sensor"

    p2t = sensor_cfg["mounts"]["pan_to_tilt_offset"]
    t2s = sensor_cfg["mounts"]["tilt_to_sensor_offset"]
    p2t_x, p2t_y, p2t_z = float(p2t["x"]), float(p2t["y"]), float(p2t["z"])
    t2s_x, t2s_y, t2s_z = float(t2s["x"]), float(t2s["y"]), float(t2s["z"])

    pan_cmd  = math.radians(floatify(sensor_cfg["orientation"].get("pan_deg", 0.0)))
    tilt_cmd = math.radians(floatify(sensor_cfg["orientation"].get("tilt_deg", 0.0)))

    # Always aim to hub if forced, else use YAML aim if present
    aim = sensor_cfg.get("aim", {})
    use_aim = FORCE_AIM_ALL_SENSORS_TO_HUB or (aim.get("target", "") == "hub_center")
    toff = aim.get("target_offset", {"x":0,"y":0,"z":0})
    tx, ty, tz = hub_target_world
    tx += float(toff.get("x",0.0)); ty += float(toff.get("y",0.0)); tz += float(toff.get("z",0.0))

    # compute sensor origin including link offsets (function of pan/tilt)
    def sensor_origin_world(pan, tilt):
        c, s = math.cos(pan), math.sin(pan)
        cy, sy = math.cos(tilt), math.sin(tilt)
        # pan rotates pan->tilt offset in XY
        ox = p2t_x*c - p2t_y*s
        oy = p2t_x*s + p2t_y*c
        oz = p2t_z
        # tilt about +Y rotates sensor offset in XZ, then pan rotates in XY
        t2sx =  t2s_x*cy + t2s_z*sy
        t2sz = -t2s_x*sy + t2s_z*cy
        t2sy =  t2s_y
        ox += t2sx*c - t2sy*s
        oy += t2sx*s + t2sy*c
        oz += t2sz
        return ox, oy, oz

    if use_aim:
        # 2-pass solve to account for offsets
        sx, sy, sz = sensor_origin_world(pan_cmd, tilt_cmd)
        pan_cmd, tilt_cmd = aim_pan_tilt_from_world_delta(tx - sx, ty - sy, tz - sz)
        sx, sy, sz = sensor_origin_world(pan_cmd, tilt_cmd)
        pan_cmd, tilt_cmd = aim_pan_tilt_from_world_delta(tx - sx, ty - sy, tz - sz)

    # Report angles (deg) in terminal
    _print_pan_tilt(system_name, sname, pan_cmd, tilt_cmd)

    # Publish TF chain (under system frame)
    tf_ch.log(FrameTransforms(transforms=[
        FrameTransform(parent_frame_id=sys_frame, child_frame_id=pan_frame,
                       translation=Vector3(x=0,y=0,z=0),
                       rotation=quat_from_euler(0,0, pan_cmd)),
        FrameTransform(parent_frame_id=pan_frame, child_frame_id=tilt_frame,
                       translation=Vector3(x=p2t_x, y=p2t_y, z=p2t_z),
                       rotation=quat_from_euler(0, tilt_cmd, 0)),
        FrameTransform(parent_frame_id=tilt_frame, child_frame_id=sens_frame,
                       translation=Vector3(x=t2s_x, y=t2s_y, z=t2s_z),
                       rotation=Quaternion(x=0,y=0,z=0,w=1)),
    ]))

    # Visible sensor body + FOV (frustum stops at hub if 'dynamic_to_target')
    body_entities = make_sensor_body_X(sens_frame, entity_id=f"sensor_body__{base}",
                                       body_len=0.40, body_r=0.08, tip_len=0.30, tip_r=0.12)

    fov = sensor_cfg["fov"]; rng = sensor_cfg["range"]
    near_m = floatify(rng["near_m"])
    far_setting = rng.get("far_m", 120.0)
    if isinstance(far_setting, str) and far_setting.strip() == "dynamic_to_target":
        sx, sy, sz = sensor_origin_world(pan_cmd, tilt_cmd)
        far_m = max(norm3(tx - sx, ty - sy, tz - sz) - 0.01, near_m + 1e-3)
    else:
        far_m = floatify(far_setting)

    fov_entity = make_fov_frustum_X(
        frame_id=sens_frame, entity_id=f"fov__{base}",
        near_m=near_m, far_m=far_m,
        haov_deg=floatify(fov["haov_deg"]), vaov_deg=floatify(fov["vaov_deg"]),
        color=Color(r=floatify(sensor_cfg["color_rgba"][0]),
                    g=floatify(sensor_cfg["color_rgba"][1]),
                    b=floatify(sensor_cfg["color_rgba"][2]),
                    a=floatify(sensor_cfg["color_rgba"][3]))
    )

    ents = [fov_entity] + body_entities

    # Bright origin cube to guarantee visibility
    if DEBUG_SHOW_SENSOR_ORIGIN_CUBES:
        ents.append(SceneEntity(
            frame_id=sens_frame, id=f"origin_cube__{base}", timestamp=now_ts(),
            lifetime=Duration.from_secs(0),
            cubes=[CubePrimitive(
                pose=Pose(position=Vector3(x=0,y=0,z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)),
                size=Vector3(x=DEBUG_ORIGIN_CUBE_SIZE, y=DEBUG_ORIGIN_CUBE_SIZE, z=DEBUG_ORIGIN_CUBE_SIZE),
                color=Color(r=1.0,g=0.0,b=1.0,a=1.0)  # magenta
            )]
        ))

    scene_ch.log(SceneUpdate(entities=ents))

# ===================== MAIN =====================
if __name__ == "__main__":
    cfg = load_config()

    foxglove.start_server(host="127.0.0.1", port=8765)
    scene_ch = SceneUpdateChannel(topic="/scene")
    tf_ch = FrameTransformsChannel(topic="/tf")

    entities, tower_h, nacelle_l, nose_len = build_scene_entities()
    scene_ch.log(SceneUpdate(entities=entities))

    dt = 1.0/30.0
    t0, last_scene = time.time(), 0.0

    WORLD, NACELLE, HUB = "world","nacelle","hub"
    B1,B2,B3 = "blade_1","blade_2","blade_3"

    while True:
        t = time.time() - t0

        # Yaw
        yaw_cmd_deg = yaw_deg + yaw_rate_deg_s * t
        yaw_cmd_rad = math.radians(yaw_cmd_deg)
        publish_wtg_tfs(tf_ch, yaw_cmd_rad, hub_ht_m, 3.6)

        # REPUBLISH system base frames every frame
        publish_system_frames(tf_ch, cfg.get("systems", []))

        # Current hub center in WORLD
        hub_cx, hub_cy, hub_cz = world_of_hub_center(yaw_cmd_rad, 3.6)
        hub_target_world = (hub_cx, hub_cy, hub_cz)

        # Publish all sensors (auto-aim + print pan/tilt)
        for sys in cfg.get("systems", []):
            sname = sys["name"]
            for sensor in sys.get("sensors", []):
                publish_sensor_chain(scene_ch, tf_ch, sname, sensor, hub_target_world)

        # Rotor spin
        omega = rot_dir * 2.0*math.pi * (rotor_rpm/60.0)
        roll = omega * t
        tf_ch.log(FrameTransforms(transforms=[
            FrameTransform(parent_frame_id=HUB, child_frame_id=B1,
                           translation=Vector3(x=0,y=0,z=0),
                           rotation=quat_from_euler(roll,0,0)),
            FrameTransform(parent_frame_id=HUB, child_frame_id=B2,
                           translation=Vector3(x=0,y=0,z=0),
                           rotation=quat_from_euler(roll + 2.0*math.pi/3.0,0,0)),
            FrameTransform(parent_frame_id=HUB, child_frame_id=B3,
                           translation=Vector3(x=0,y=0,z=0),
                           rotation=quat_from_euler(roll + 4.0*math.pi/3.0,0,0)),
        ]))

        # Re-send static scene occasionally (late joiners)
        now_s = time.time()
        if now_s - last_scene > 1.0:
            scene_ch.log(SceneUpdate(entities=entities))
            last_scene = now_s

        time.sleep(dt)
