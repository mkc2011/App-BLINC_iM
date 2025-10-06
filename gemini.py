import math, time, datetime, json
import foxglove
# 1. CORRECTED IMPORT: 'Channel' comes from the top-level 'foxglove' module.
from foxglove import Channel
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
yaw_rate_deg_s = 1.0  # Set a small rate to see coordinates change

# SPs start rotated +270° at yaw=0 (your saved preference)
sp_yaw_offset_deg = 270.0

# --- Rotor / WTG geometry ---
rotor_dia_m        = 80.0      # change me; blades & rings auto-resize
hub_ht_m           = 80.0      # tower height; nacelle Z
theta_deg_val      = 10.0

# ... (The rest of the script's CONTROL KNOBS and helper functions are unchanged) ...
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

# ====== 3D panel preset for sp1_camera_sensor ======
def print_sp1_panel_preset():
    D = PANEL_PRESET_DISTANCE
    print("\n=== Foxglove 3D preset for frame 'sp1_camera_sensor' ===")
    print("Display frame: sp1_camera_sensor   |   3D view: On")
    print(f"Option A (look along +X): Distance={D:.3f}, Target X={D:.3f}, Target Y=0, Target Z=0, Theta=180, Phi=90")
    print(f"Option B (Theta=90, Phi=90):      Distance={D:.3f}, Target X=0, Target Y={D:.3f}, Target Z=0, Theta=90, Phi=90\n")

# ===================== GEOMETRY HELPERS (UNCHANGED) =====================
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
        p3 = Point3(x=dist_m, y=y1, z=z1)
        p4 = Point3(x=dist_m, y=y0, z=z1)
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

# ===================== SP MARKERS (UNCHANGED) =====================
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

# ===================== FRAMES / TRANSFORMS (UNCHANGED) =====================
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
    nacelle_rot = yaw_rad + math.pi/2
    hx_local, hy_local = (0.6*nacelle_l, 0.0)
    hub_x =  hx_local*math.cos(nacelle_rot) - hy_local*math.sin(nacelle_rot)
    hub_y =  hx_local*math.sin(nacelle_rot) + hy_local*math.cos(nacelle_rot)
    hub_z =  hub_ht_m
    return hub_x, hub_y, hub_z
def sp1_xy_world(yaw_rad: float):
    sp_rot = yaw_rad + math.pi/2 + math.radians(sp_yaw_offset_deg)
    x_local = -rotor_dia_m/4.0
    y_local = +rotor_dia_m*1.125
    c, s = math.cos(sp_rot), math.sin(sp_rot)
    xw = x_local*c - y_local*s
    yw = x_local*s + y_local*c
    return xw, yw

# ===================== SENSOR AIM (UNCHANGED) =====================
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

# ===================== SCENE BUILD (UNCHANGED) =====================
def build_scene_entities():
    tower_h = hub_ht_m
    nacelle_l, nacelle_w, nacelle_h = 3.6, 1.8, 1.2
    hub_d, hub_len = 1.2, 0.8
    nose_len = 1.0
    nose_base_radius = (hub_d/2.0)*0.6
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
                        size=Vector3(x=nacelle_l, y=nacelle_w, z=nacelle_h),
                        color=Color(r=0.2,g=0.6,b=0.9,a=1.0)
                    )]),
        make_cylinder_mesh_X(HUB, "hub_mesh", radius=hub_d/2.0, length=hub_len, segments=48,
                             color=Color(r=0.92,g=0.92,b=0.95,a=1.0)),
        make_cone_mesh_X(HUB, "nose_cone_mesh", base_radius=nose_base_radius, length=nose_len, segments=48,
                         color=Color(r=0.9,g=0.9,b=0.97,a=1.0)),
        make_blade_entity("blade_1","blade1",
                          Lz=blade_len, root_w=blade_root_w_m, tip_w=blade_tip_w_m,
                          thickness=blade_thickness_m, color=Color(r=0.95,g=0.2,b=0.2,a=1.0)),
        make_blade_entity("blade_2","blade2",
                          Lz=blade_len, root_w=blade_root_w_m, tip_w=blade_tip_w_m,
                          thickness=blade_thickness_m, color=Color(r=0.2,g=0.95,b=0.2,a=1.0)),
        make_blade_entity("blade_3","blade3",
                          Lz=blade_len, root_w=blade_root_w_m, tip_w=blade_tip_w_m,
                          thickness=blade_thickness_m, color=Color(r=0.2,g=0.2,b=0.95,a=1.0)),
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
    return entities, tower_h, nacelle_l, nose_len

# ===================== STATUS PANEL =====================
# --- Helper function to calculate SP world coordinates for the panel ---
def get_sp_locations_world(yaw_rad: float, r_dia: float, h_hub: float, th_deg: float, yaw_off_deg: float):
    sp_rot = yaw_rad + math.pi/2 + math.radians(yaw_off_deg)
    c, s = math.cos(sp_rot), math.sin(sp_rot)
    
    # Local coordinates from make_bullseye_markers_with_labels
    x = r_dia / 4.0
    y_R = r_dia * 1.125
    y_T = h_hub * math.tan(math.radians(th_deg))
    
    pts_local = [
        (-x, +y_R), # SP1
        (-x, +y_T), # SP2
        (-x, -y_R), # SP3
        (+x, -y_T), # SP4
    ]

    pts_world = []
    for xl, yl in pts_local:
        xw = xl*c - yl*s
        yw = xl*s + yl*c
        pts_world.append((xw, yw))
    
    return pts_world

# --- Define the schema for our status message ---
STATUS_SCHEMA = {
    "type": "object",
    "properties": {
        "rotor_dia_m": {"type": "number", "description": "Rotor Diameter (m)"},
        "hub_ht_m": {"type": "number", "description": "Hub Height (m)"},
        "system_height_m": {"type": "number", "description": "Max tip height (m)"},
        "rotor_rpm": {"type": "number", "description": "Rotor Speed (RPM)"},
        "yaw_deg": {"type": "number", "description": "Turbine Yaw (deg)"},
        "turbine_xyz": {"type": "string", "description": "Turbine Base Coords"},
        "sp1_xyz": {"type": "string", "description": "SP1 World Coords"},
        "sp2_xyz": {"type": "string", "description": "SP2 World Coords"},
        "sp3_xyz": {"type": "string", "description": "SP3 World Coords"},
        "sp4_xyz": {"type": "string", "description": "SP4 World Coords"},
    }
}


# ===================== MAIN =====================
if __name__ == "__main__":
    
    foxglove.start_server(host="127.0.0.1", port=8765)
    scene_ch = SceneUpdateChannel(topic="/scene")
    tf_ch = FrameTransformsChannel(topic="/tf")

    # 2. CREATE THE CUSTOM CHANNEL OBJECT
    # This uses the same high-level pattern as SceneUpdateChannel
    status_ch = Channel(
        topic="/system_status",
        schema_name="SystemStatus",
        schema=STATUS_SCHEMA,
    )

    entities, tower_h, nacelle_l, nose_len = build_scene_entities()
    scene_ch.log(SceneUpdate(entities=entities))
    print_sp1_panel_preset()

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

        if (rotor_dia_m != last_rotor or hub_ht_m != last_hub or
            theta_deg_val != last_theta or blade_thickness_m != last_b_thk or
            blade_root_w_m != last_rootw or blade_tip_w_m != last_tipw):
            entities, tower_h, nacelle_l, nose_len = build_scene_entities()
            scene_ch.log(SceneUpdate(entities=entities))
            last_rotor, last_hub = rotor_dia_m, hub_ht_m
            last_theta, last_b_thk = theta_deg_val, blade_thickness_m
            last_rootw, last_tipw = blade_root_w_m, blade_tip_w_m

        yaw_cmd_deg = yaw_deg + yaw_rate_deg_s * t
        yaw_cmd_rad = math.radians(yaw_cmd_deg)
        publish_wtg_frames(tf_ch, yaw_cmd_rad, tower_h, nacelle_l)

        # 3. POPULATE AND PUBLISH THE STATUS MESSAGE using .log()
        sp_coords = get_sp_locations_world(yaw_cmd_rad, rotor_dia_m, hub_ht_m, theta_deg_val, sp_yaw_offset_deg)
        status_msg = {
            "rotor_dia_m": rotor_dia_m,
            "hub_ht_m": hub_ht_m,
            "system_height_m": hub_ht_m + rotor_dia_m / 2.0,
            "rotor_rpm": 12.0,
            "yaw_deg": wrap_deg(yaw_cmd_deg),
            "turbine_xyz": "0.00, 0.00, 0.00",
            "sp1_xyz": f"{sp_coords[0][0]:.2f}, {sp_coords[0][1]:.2f}, 0.00",
            "sp2_xyz": f"{sp_coords[1][0]:.2f}, {sp_coords[1][1]:.2f}, 0.00",
            "sp3_xyz": f"{sp_coords[2][0]:.2f}, {sp_coords[2][1]:.2f}, 0.00",
            "sp4_xyz": f"{sp_coords[3][0]:.2f}, {sp_coords[3][1]:.2f}, 0.00",
        }
        status_ch.log(status_msg)
        
        hub_x, hub_y, hub_z = hub_center_world(yaw_cmd_rad, nacelle_l)
        target_world = (hub_x, hub_y, hub_z + hub_target_z_offset_m)

        # ... (The rest of the main loop is unchanged) ...
        # ---------------- Tracker sensors (Benewake + LIVOX) ----------------
        pan_base_world = (0.0, rotor_dia_m*1.125, tracker_ht_m)
        pan_cmd, tilt_cmd = solve_pan_tilt_to_target(pan_base_world, target_world)
        tf_ch.log(FrameTransforms(transforms=[
            FrameTransform(parent_frame_id=WORLD, child_frame_id=B_PAN,
                           translation=Vector3(x=pan_base_world[0], y=pan_base_world[1], z=pan_base_world[2]),
                           rotation=quat_from_euler(0.0, 0.0, -math.pi/2 + pan_cmd)),
            FrameTransform(parent_frame_id=B_PAN, child_frame_id=B_TILT,
                           translation=Vector3(x=pan_to_tilt_offset[0], y=pan_to_tilt_offset[1], z=pan_to_tilt_offset[2]),
                           rotation=quat_from_euler(0.0, tilt_cmd, 0.0)),
            FrameTransform(parent_frame_id=B_TILT, child_frame_id=B_SENSOR,
                           translation=Vector3(x=tilt_to_sensor_offset[0], y=tilt_to_sensor_offset[1], z=tilt_to_sensor_offset[2]),
                           rotation=Quaternion(x=0,y=0,z=0,w=1)),
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
        ox, oy, oz = sensor_origin_with_offsets(pan_cmd, tilt_cmd)
        sx, sy, sz = pan_base_world[0] + ox, pan_base_world[1] + oy, pan_base_world[2] + oz
        dist_to_target = math.sqrt((target_world[0]-sx)**2 + (target_world[1]-sy)**2 + (target_world[2]-sz)**2)
        benewake_far = max(dist_to_target - 0.01, benewake_near_m + 1e-3)
        scene_ch.log(SceneUpdate(entities=[
            make_fov_frustum_X(B_SENSOR, "fov_benewake",
                               near_m=benewake_near_m, far_m=benewake_far,
                               haov_deg=benewake_haov_deg, vaov_deg=benewake_vaov_deg,
                               color=benewake_color),
            make_fov_frustum_X(L_SENSOR, "fov_livox",
                               near_m=livox_near_m, far_m=livox_far_m,
                               haov_deg=livox_haov_deg, vaov_deg=livox_vaov_deg,
                               color=livox_color),
        ]))

        # ---------------- SP1 S-Laser + Camera (same pose) ----------------
        sp1_xw, sp1_yw = sp1_xy_world(yaw_cmd_rad)
        sp1_base_world = (sp1_xw, sp1_yw, tracker_ht_m)
        sp1_pan, sp1_tilt = solve_pan_tilt_to_target(sp1_base_world, target_world)
        tf_ch.log(FrameTransforms(transforms=[
            FrameTransform(parent_frame_id=WORLD, child_frame_id=S_PAN,
                           translation=Vector3(x=sp1_base_world[0], y=sp1_base_world[1], z=sp1_base_world[2]),
                           rotation=quat_from_euler(0.0, 0.0, -math.pi/2 + sp1_pan)),
            FrameTransform(parent_frame_id=S_PAN, child_frame_id=S_TILT,
                           translation=Vector3(x=pan_to_tilt_offset[0], y=pan_to_tilt_offset[1], z=pan_to_tilt_offset[2]),
                           rotation=quat_from_euler(0.0, sp1_tilt, 0.0)),
            FrameTransform(parent_frame_id=S_TILT, child_frame_id=S_SENSOR,
                           translation=Vector3(x=tilt_to_sensor_offset[0], y=tilt_to_sensor_offset[1], z=tilt_to_sensor_offset[2]),
                           rotation=Quaternion(x=0,y=0,z=0,w=1)),
            FrameTransform(parent_frame_id=S_SENSOR, child_frame_id=C_SENSOR,
                           translation=Vector3(x=0.0, y=0.0, z=0.0),
                           rotation=Quaternion(x=0,y=0,z=0,w=1)),
        ]))
        scene_ch.log(SceneUpdate(entities=[
            make_fov_frustum_X(S_SENSOR, "fov_sp1_slaser",
                               near_m=slaser_near_m, far_m=slaser_far_m,
                               haov_deg=slaser_haov_deg, vaov_deg=slaser_vaov_deg,
                               color=slaser_color),
        ]))
        scene_ch.log(SceneUpdate(entities=[
            make_fov_frustum_X(C_SENSOR, "sp1_camera_fov",
                               near_m=camera_near_m, far_m=camera_far_m,
                               haov_deg=camera_haov_deg, vaov_deg=camera_vaov_deg,
                               color=camera_color),
            make_camera_view_mask(C_SENSOR, "sp1_camera_mask",
                                  haov_deg=camera_haov_deg, vaov_deg=camera_vaov_deg,
                                  dist_m=camera_mask_dist, pad_scale=camera_mask_pad,
                                  alpha=camera_mask_alpha),
            make_view_anchor_entity(C_SENSOR, "sp1_cam_target_anchor", PANEL_PRESET_DISTANCE),
        ]))

        now_s = time.time()
        if now_s - last_print >= 0.25:
            print(f"[Tracker] Pan {wrap_deg(math.degrees(pan_cmd)):+7.2f}°, "
                  f"Tilt {wrap_deg(math.degrees(tilt_cmd)):+7.2f}°   |   "
                  f"[SP1] Pan {wrap_deg(math.degrees(sp1_pan)):+7.2f}°, "
                  f"Tilt {wrap_deg(math.degrees(sp1_tilt)):+7.2f}°   |   "
                  f"RotorDia {rotor_dia_m:.1f} m, HubHt {hub_ht_m:.1f} m")
            last_print = now_s

        omega = 2.0*math.pi*(12.0/60.0)
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

        if now_s - last_scene > 1.0:
            scene_ch.log(SceneUpdate(entities=entities))
            last_scene = now_s

        publish_world_root(tf_ch)
        time.sleep(1.0/30.0)