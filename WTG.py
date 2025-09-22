import math, time, datetime
import foxglove
from foxglove.channels import SceneUpdateChannel, FrameTransformsChannel
from foxglove.schemas import (
    Color, CubePrimitive, Duration,
    FrameTransform, FrameTransforms, Pose, Quaternion,
    SceneEntity, SceneUpdate, Timestamp, Vector3,
    TriangleListPrimitive, Point3,
)

# ============== CONTROL KNOBS ==============
# --- Yaw control ---
yaw_deg = 0.0        # Fixed nacelle yaw about +Z (deg). 0 = facing +X in world.
yaw_rate_deg_s = 0.0  # Slew rate (deg/s). Set non-zero for continuous yawing.

# --- Rotor control ---
rotor_rpm = 12.0       # Rotor speed in RPM
rot_dir   = -1.0      # +1 = CCW (right-hand rule about +X, looking outward from nacelle)
                      # -1 = CW

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

# ============== Mesh builders (TriangleListPrimitive) ==============
# Vertical cylinder along +Z (base at z=0, top at z=height)
def make_cylinder_mesh_Z(
    frame_id: str,
    entity_id: str,
    radius: float,
    height: float,
    segments: int = 64,
    color: Color = Color(r=0.85, g=0.85, b=0.9, a=1.0),
) -> SceneEntity:
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
    # side quads as 2 triangles
    for i in range(segments):
        j = (i + 1) % segments
        bi, bj = verts_bot[i], verts_bot[j]
        ti, tj = verts_top[i], verts_top[j]
        tris += [bi, bj, tj]
        tris += [bi, tj, ti]
    # top cap
    for i in range(segments):
        j = (i + 1) % segments
        ti, tj = verts_top[i], verts_top[j]
        tris += [top_center, ti, tj]
    # bottom cap
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

# Cylinder aligned along +X, centered at origin: x ∈ [-L/2, +L/2], circle in YZ
def make_cylinder_mesh_X(
    frame_id: str,
    entity_id: str,
    radius: float,
    length: float,
    segments: int = 48,
    color: Color = Color(r=0.9, g=0.9, b=0.95, a=1.0),
) -> SceneEntity:
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
    # side quads
    for i in range(segments):
        j = (i + 1) % segments
        li, lj = ring_L[i], ring_L[j]
        ri, rj = ring_R[i], ring_R[j]
        tris += [li, lj, rj]
        tris += [li, rj, ri]
    # right cap (outward +X)
    for i in range(segments):
        j = (i + 1) % segments
        ri, rj = ring_R[i], ring_R[j]
        tris += [center_R, ri, rj]
    # left cap (outward -X)
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

# Cone along +X: base at x=0 (circle in YZ), tip at x=+length
def make_cone_mesh_X(
    frame_id: str,
    entity_id: str,
    base_radius: float,
    length: float,
    segments: int = 48,
    color: Color = Color(r=0.88, g=0.9, b=0.95, a=1.0),
) -> SceneEntity:
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
    # side fan
    for i in range(segments):
        j = (i + 1) % segments
        vi, vj = ring[i], ring[j]
        tris += [vi, vj, tip]
    # base disk (outward -X)
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
        A,B,C,  A,C,D,            # front
        A2,C2,B2,  A2,D2,C2,      # back
        A,A2,B2,  A,B2,B,         # edge AB
        B,B2,C2,  B,C2,C,         # edge BC
        C,C2,D2,  C,D2,D,         # edge CD
        D,D2,A2,  D,A2,A,         # edge DA
    ]
    return SceneEntity(
        frame_id=frame_id, id=entity_id,
        timestamp=now_ts(),
        lifetime=Duration.from_secs(0),
        triangles=[TriangleListPrimitive(points=tris, color=color)],
    )

# ============== Scene build ==============
def build_scene_entities():
    WORLD, NACELLE, HUB = "world", "nacelle", "hub"
    B1, B2, B3 = "blade_1", "blade_2", "blade_3"

    # Dimensions
    tower_h, tower_d = 30.0, 2.5
    tower_radius = (tower_d / 2.0) * 0.5   # 50% slimmer (your earlier ask)
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
        # Nacelle cuboid (drawn in NACELLE frame)
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
        # Hub cylinder along +X (in HUB frame)
        make_cylinder_mesh_X(
            frame_id=HUB, entity_id="hub_mesh",
            radius=hub_d/2.0, length=hub_len, segments=48,
            color=Color(r=0.92, g=0.92, b=0.95, a=1.0),
        ),
        # Nose cone along +X from hub origin
        make_cone_mesh_X(
            frame_id=HUB, entity_id="nose_cone_mesh",
            base_radius=nose_base_radius, length=nose_len, segments=48,
            color=Color(r=0.9, g=0.9, b=0.97, a=1.0),
        ),
        # Blades
        make_blade_entity("blade_1", "blade1", color=Color(r=0.95,g=0.2,b=0.2,a=1.0)),
        make_blade_entity("blade_2", "blade2", color=Color(r=0.2,g=0.95,b=0.2,a=1.0)),
        make_blade_entity("blade_3", "blade3", color=Color(r=0.2,g=0.2,b=0.95,a=1.0)),
    ]
    return entities, tower_h, nacelle_l, nose_len

# ============== Main ==============
if __name__ == "__main__":
    foxglove.start_server(host="127.0.0.1", port=8765)
    scene_ch = SceneUpdateChannel(topic="/scene")
    tf_ch = FrameTransformsChannel(topic="/tf")

    entities, tower_h, nacelle_l, nose_len = build_scene_entities()
    scene_ch.log(SceneUpdate(entities=entities))

    dt = 1.0 / 30.0
    t0, last_scene = time.time(), 0.0

    WORLD, NACELLE, HUB = "world", "nacelle", "hub"
    B1, B2, B3 = "blade_1", "blade_2", "blade_3"

    # Static root->world (kept alive)
    def publish_root_world():
        tf_ch.log(FrameTransforms(transforms=[
            FrameTransform(parent_frame_id="root", child_frame_id=WORLD,
                           translation=Vector3(x=0, y=0, z=0),
                           rotation=Quaternion(x=0,y=0,z=0,w=1)),
        ]))

    # world->nacelle with yaw around Z; nacelle->hub forward +X
    def publish_nacelle_and_hub(yaw_rad: float):
        tf_ch.log(FrameTransforms(transforms=[
            # world -> nacelle (top of tower), offset so yaw=0 faces world +Y
            FrameTransform(parent_frame_id=WORLD, child_frame_id=NACELLE,
                        translation=Vector3(x=0, y=0, z=tower_h),
                        rotation=quat_from_euler(0.0, 0.0, yaw_rad + math.pi/2)),
            # nacelle -> hub (forward along +X in nacelle frame)
            FrameTransform(parent_frame_id=NACELLE, child_frame_id=HUB,
                        translation=Vector3(x=nacelle_l*0.6, y=0, z=0),
                        rotation=Quaternion(x=0,y=0,z=0,w=1)),
        ]))

    publish_root_world()

    while True:
        t = time.time() - t0

        # --- Yaw command (deg -> rad), with optional slew ---
        yaw_cmd_deg = yaw_deg + yaw_rate_deg_s * t
        yaw_cmd_rad = math.radians(yaw_cmd_deg)
        publish_nacelle_and_hub(yaw_cmd_rad)

        # --- Rotor angular speed (rad/s), with direction ---
        omega = rot_dir * 2.0 * math.pi * (rotor_rpm / 60.0)
        roll = omega * t  # phase about +X

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

        # Re-send scene occasionally (helps late connections)
        now_s = time.time()
        if now_s - last_scene > 1.0:
            scene_ch.log(SceneUpdate(entities=entities))
            last_scene = now_s

        publish_root_world()
        time.sleep(dt)
