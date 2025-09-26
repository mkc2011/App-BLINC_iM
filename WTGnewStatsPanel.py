import math, time, datetime, threading, json
import http.server, socketserver
import foxglove
from foxglove.channels import SceneUpdateChannel, FrameTransformsChannel
from foxglove.schemas import (
    Color, CubePrimitive, Duration,
    FrameTransform, FrameTransforms, Pose, Quaternion,
    SceneEntity, SceneUpdate, Timestamp, Vector3,
    TriangleListPrimitive, Point3,
)

# Optional text labels (for SP markers)
try:
    from foxglove.schemas import TextPrimitive
    HAVE_TEXT = True
except Exception:
    HAVE_TEXT = False

# ===================== CONTROL KNOBS =====================
yaw_deg = 0.0
yaw_rate_deg_s = 0.0
sp_yaw_offset_deg = 270.0

# WTG geometry
rotor_dia_m        = 80.0
hub_ht_m           = 80.0
theta_deg_val      = 10.0

# Blades look
blade_thickness_m  = 0.25
blade_root_w_m     = 1.6
blade_tip_w_m      = 0.40

# Rotation (for animation + status)
rotor_rpm          = 12.0
rot_dir            = -1.0  # +1=CCW about +X looking forward, -1=CW

# Ground circle ring
circle_ring_width_m = 0.6
circle_segments     = 256
circle_color        = Color(r=0.10, g=0.45, b=0.12, a=0.9)

# SP bullseyes
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

# Mechanical stack: PAN -> (offset) -> TILT -> (offset) -> SENSOR (+X forward)
pan_to_tilt_offset    = (0.0, 0.05, 0.01)
tilt_to_sensor_offset = (0.0, 0.00, 0.01)

# Benewake (narrow; dynamic to hub)
benewake_haov_deg   = 0.35
benewake_vaov_deg   = 0.15
benewake_near_m     = 0.20
benewake_color      = Color(r=1.0, g=0.0, b=0.0, a=0.45)

# LIVOX (wide; fixed 130 m)
livox_haov_deg      = 70.4
livox_vaov_deg      = 74.2
livox_near_m        = 0.20
livox_far_m         = 130.0
livox_color         = Color(r=0.1, g=0.8, b=1.0, a=0.10)

# S-Laser at SP1
slaser_haov_deg     = 0.35
slaser_vaov_deg     = 0.15
slaser_near_m       = 0.20
slaser_far_m        = 150.0
slaser_color        = Color(r=1.0, g=0.4, b=0.0, a=0.45)

# Camera (co-located with S-Laser)
camera_haov_deg     = 2.8547
camera_vaov_deg     = 2.1388
camera_near_m       = 0.20
camera_far_m        = 150.0
camera_color        = Color(r=1.0, g=1.0, b=1.0, a=0.12)
camera_mask_alpha   = 0.80
camera_mask_pad     = 25.0
camera_mask_dist    = 0.12

# Aim offset
hub_target_z_offset_m = 20.0

# Web status server
STATUS_PORT = 8766

# Frames
WORLD, NACELLE, HUB, SCAN = "world", "nacelle", "hub", "scan_ground"
B1, B2, B3 = "blade_1", "blade_2", "blade_3"
# Tracker sensors
B_PAN, B_TILT, B_SENSOR = "benewake_pan", "benewake_tilt", "benewake_sensor"
L_PAN, L_TILT, L_SENSOR = "livox_pan", "livox_tilt", "livox_sensor"
# SP1 S-Laser + Camera
S_PAN, S_TILT, S_SENSOR = "sp1_slaser_pan", "sp1_slaser_tilt", "sp1_slaser_sensor"
C_SENSOR = "sp1_camera_sensor"

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
        z=cr*cp*sy - sr*cp*cy,
        w=cr*cp*cy + sr*cp*sy,
    )

def now_ts() -> Timestamp:
    return Timestamp.from_datetime(datetime.datetime.utcnow())

# ===================== GEOMETRY HELPERS =====================
def make_filled_disk_tris(cx, cy, radius, segments, z=0.0):
    tris=[]; center=Point3(x=cx,y=cy,z=z); ring=[]
    for i in range(segments):
        th=2.0*math.pi*i/segments
        ring.append(Point3(x=cx+radius*math.cos(th), y=cy+radius*math.sin(th), z=z))
    for i in range(segments):
        j=(i+1)%segments
        tris += [center, ring[i], ring[j]]
    return tris

def make_circle_ring_tris(cx, cy, r_outer, ring_w, segments, z=0.0):
    r_out=max(r_outer,1e-3); r_in=max(r_outer-max(ring_w,1e-3),1e-3)
    out_ring=[]; in_ring=[]
    for i in range(segments):
        th=2.0*math.pi*i/segments; c,s=math.cos(th),math.sin(th)
        out_ring.append(Point3(x=cx+r_out*c,y=cy+r_out*s,z=z))
        in_ring.append (Point3(x=cx+r_in *c,y=cy+r_in *s,z=z))
    tris=[]
    for i in range(segments):
        j=(i+1)%segments
        oi,oj=out_ring[i],out_ring[j]; ii,ij=in_ring[i],in_ring[j]
        tris += [oi,oj,ij]; tris += [oi,ij,ii]
    return tris

def make_ground_circle_ring(frame_id, entity_id, radius, ring_width, segments, color, z_offset=0.01):
    tris = make_circle_ring_tris(0.0,0.0,radius, ring_width, segments, z=z_offset)
    return SceneEntity(frame_id=frame_id,id=entity_id,timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=tris,color=color)])

def make_cylinder_mesh_Z(frame_id, entity_id, radius, height, segments=64, color=Color(r=0.85,g=0.85,b=0.9,a=1.0)):
    top_z, bot_z = height, 0.0
    vt, vb=[],[]
    for i in range(segments):
        th=2.0*math.pi*i/segments; x=radius*math.cos(th); y=radius*math.sin(th)
        vt.append(Point3(x=x,y=y,z=top_z)); vb.append(Point3(x=x,y=y,z=bot_z))
    tc=Point3(x=0.0,y=0.0,z=top_z); bc=Point3(x=0.0,y=0.0,z=bot_z)
    tris=[]
    for i in range(segments):
        j=(i+1)%segments; bi,bj=vb[i],vb[j]; ti,tj=vt[i],vt[j]
        tris += [bi,bj,tj]; tris += [bi,tj,ti]
    for i in range(segments):
        j=(i+1)%segments; ti,tj=vt[i],vt[j]; tris += [tc,ti,tj]
    for i in range(segments):
        j=(i+1)%segments; bi,bj=vb[i],vb[j]; tris += [bc,bj,bi]
    return SceneEntity(frame_id=frame_id,id=entity_id,timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=tris,color=color)])

def make_cylinder_mesh_Z_at(frame_id, entity_id, radius, height, cx, cy, segments=64, color=Color(r=0,g=0,b=0,a=1.0)):
    top_z, bot_z = height, 0.0
    vt, vb=[],[]
    for i in range(segments):
        th=2.0*math.pi*i/segments; x=cx+radius*math.cos(th); y=cy+radius*math.sin(th)
        vt.append(Point3(x=x,y=y,z=top_z)); vb.append(Point3(x=x,y=y,z=bot_z))
    tc=Point3(x=cx,y=cy,z=top_z); bc=Point3(x=cx,y=cy,z=bot_z)
    tris=[]
    for i in range(segments):
        j=(i+1)%segments; bi,bj=vb[i],vb[j]; ti,tj=vt[i],vt[j]
        tris += [bi,bj,tj]; tris += [bi,tj,ti]
    for i in range(segments):
        j=(i+1)%segments; ti,tj=vt[i],vt[j]; tris += [tc,ti,tj]
    for i in range(segments):
        j=(i+1)%segments; bi,bj=vb[i],vb[j]; tris += [bc,bj,bi]
    return SceneEntity(frame_id=frame_id,id=entity_id,timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=tris,color=color)])

def make_cylinder_mesh_X(frame_id, entity_id, radius, length, segments=48, color=Color(r=0.9,g=0.9,b=0.95,a=1.0)):
    xL,xR=-length/2.0,+length/2.0; rL,rR=[],[]
    for i in range(segments):
        th=2.0*math.pi*i/segments; y=radius*math.cos(th); z=radius*math.sin(th)
        rL.append(Point3(x=xL,y=y,z=z)); rR.append(Point3(x=xR,y=y,z=z))
    cL=Point3(x=xL,y=0.0,z=0.0); cR=Point3(x=xR,y=0.0,z=0.0)
    tris=[]
    for i in range(segments):
        j=(i+1)%segments; li,lj=rL[i],rL[j]; ri,rj=rR[i],rR[j]
        tris += [li,lj,rj]; tris += [li,rj,ri]
    for i in range(segments):
        j=(i+1)%segments; ri,rj=rR[i],rR[j]; tris += [cR,ri,rj]
    for i in range(segments):
        j=(i+1)%segments; li,lj=rL[i],rL[j]; tris += [cL,lj,li]
    return SceneEntity(frame_id=frame_id,id=entity_id,timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=tris,color=color)])

def make_cone_mesh_X(frame_id, entity_id, base_radius, length, segments=48, color=Color(r=0.88,g=0.9,b=0.95,a=1.0)):
    x_base,x_tip=0.0,length; ring=[]
    for i in range(segments):
        th=2.0*math.pi*i/segments; y=base_radius*math.cos(th); z=base_radius*math.sin(th)
        ring.append(Point3(x=x_base,y=y,z=z))
    cb=Point3(x=x_base,y=0.0,z=0.0); tip=Point3(x=x_tip,y=0.0,z=0.0)
    tris=[]
    for i in range(segments):
        j=(i+1)%segments; vi,vj=ring[i],ring[j]; tris += [vi,vj,tip]
    for i in range(segments):
        j=(i+1)%segments; vi,vj=ring[i],ring[j]; tris += [cb,vj,vi]
    return SceneEntity(frame_id=frame_id,id=entity_id,timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=tris,color=color)])

def make_blade_entity(frame_id, entity_id, Lz=18.0, root_w=1.2, tip_w=0.3, thickness=0.15, color=Color(r=0.95,g=0.2,b=0.2,a=1.0)):
    hx=thickness/2.0
    A  = Point3(x=+hx,y=-root_w/2,z=0.0)
    B  = Point3(x=+hx,y=-tip_w/2, z=Lz)
    C  = Point3(x=+hx,y=+tip_w/2, z=Lz)
    D  = Point3(x=+hx,y=+root_w/2,z=0.0)
    A2 = Point3(x=-hx,y=-root_w/2,z=0.0)
    B2 = Point3(x=-hx,y=-tip_w/2, z=Lz)
    C2 = Point3(x=-hx,y=+tip_w/2, z=Lz)
    D2 = Point3(x=-hx,y=+root_w/2, z=0.0)
    tris=[A,B,C, A,C,D, A2,C2,B2, A2,D2,C2, A,A2,B2, A,B2,B, B,B2,C2, B,C2,C, C,C2,D2, C,D2,D, D,D2,A2, D,A2,A]
    return SceneEntity(frame_id=frame_id,id=entity_id,timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=tris,color=color)])

def make_fov_frustum_X(frame_id, entity_id, near_m, far_m, haov_deg, vaov_deg, color):
    near_m=max(near_m,1e-3); far_m=max(far_m,near_m+1e-3)
    h2=math.radians(haov_deg*0.5); v2=math.radians(vaov_deg*0.5)
    y_n,z_n=near_m*math.tan(h2),near_m*math.tan(v2)
    y_f,z_f= far_m*math.tan(h2), far_m*math.tan(v2)
    n1=Point3(x=near_m,y=-y_n,z=-z_n); n2=Point3(x=near_m,y= y_n,z=-z_n)
    n3=Point3(x=near_m,y= y_n,z= z_n); n4=Point3(x=near_m,y=-y_n,z= z_n)
    f1=Point3(x= far_m,y=-y_f,z=-z_f); f2=Point3(x= far_m,y= y_f,z=-z_f)
    f3=Point3(x= far_m,y= y_f,z= z_f); f4=Point3(x= far_m,y=-y_f,z= z_f)
    tris=[]
    tris+=[n1,n2,n3]; tris+=[n1,n3,n4]
    tris+=[f1,f3,f2]; tris+=[f1,f4,f3]
    tris+=[n1,f2,f1]; tris+=[n1,n2,f2]
    tris+=[n2,f3,f2]; tris+=[n2,n3,f3]
    tris+=[n3,f4,f3]; tris+=[n3,n4,f4]
    tris+=[n4,f1,f4]; tris+=[n4,n1,f1]
    return SceneEntity(frame_id=frame_id,id=entity_id,timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=tris,color=color)])

def make_camera_view_mask(frame_id, entity_id, haov_deg, vaov_deg, dist_m, pad_scale, alpha=0.8):
    h2=math.radians(haov_deg*0.5); v2=math.radians(vaov_deg*0.5)
    wy=dist_m*math.tan(h2); wz=dist_m*math.tan(v2)
    pad=(wy+wz)*0.5*pad_scale
    def quad(y0,z0,y1,z1):
        p1=Point3(x=dist_m,y=y0,z=z0); p2=Point3(x=dist_m,y=y1,z=z0)
        p3=Point3(x=dist_m,y=y1,z=z1); p4=Point3(x=dist_m,y=y0,z=z1)
        return [p1,p2,p3, p1,p3,p4]
    tris=[]
    tris+=quad(-pad,-pad,-wy,+pad)  # left
    tris+=quad(+wy,-pad,+pad,+pad)  # right
    tris+=quad(-wy,-pad,+wy,-wz)    # bottom
    tris+=quad(-wy,+wz,+wy,+pad)    # top
    return SceneEntity(frame_id=frame_id,id=entity_id,timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=tris,color=Color(r=0,g=0,b=0,a=alpha))])

# ===================== SP MARKERS =====================
def make_bullseye_markers_with_labels(frame_id, entity_id):
    theta_rad=math.radians(theta_deg_val)
    x=rotor_dia_m/4.0; y_R=rotor_dia_m*1.125; y_T=hub_ht_m*math.tan(theta_rad)
    pts=[(-x,+y_R,0.0),(-x,+y_T,0.0),(-x,-y_R,0.0),(+x,-y_T,0.0)]
    labels=["SP1","SP2","SP3","SP4"]
    all_tris=[]
    for (px,py,_) in pts:
        all_tris+=make_filled_disk_tris(px,py,marker_inner_r_m,marker_segments)
        all_tris+=make_circle_ring_tris(px,py,marker_outer_r_m,marker_ring_w_m,marker_segments)
    entity_kwargs=dict(frame_id=frame_id,id=entity_id,timestamp=now_ts(),
                       lifetime=Duration.from_secs(0),
                       triangles=[TriangleListPrimitive(points=all_tris,color=marker_color_red)])
    if HAVE_TEXT:
        texts=[]
        for (px,py,_),name in zip(pts,labels):
            r=max(math.hypot(px,py),1e-6); ux,uy=px/r,py/r
            lx,ly=px+ux*label_offset_m, py+uy*label_offset_m
            kw=dict(pose=Pose(position=Vector3(x=lx,y=ly,z=label_height_m),
                               orientation=Quaternion(x=0,y=0,z=0,w=1)),
                    text=name, font_size=label_font_size, color=label_color)
            try:
                texts.append(TextPrimitive(**kw,billboard=label_billboard,background_color=label_bg_color))
            except TypeError:
                try:
                    texts.append(TextPrimitive(**kw,billboard=label_billboard,background=True,bg_color=label_bg_color))
                except TypeError:
                    texts.append(TextPrimitive(**kw,billboard=label_billboard))
        entity_kwargs["texts"]=texts
    return SceneEntity(**entity_kwargs)

# ===================== FRAMES / TRANSFORMS =====================
def publish_wtg_frames(tf_ch, yaw_rad, tower_h, nacelle_l):
    nacelle_rot=yaw_rad+math.pi/2
    sp_rot=yaw_rad+math.pi/2+math.radians(sp_yaw_offset_deg)
    tf_ch.log(FrameTransforms(transforms=[
        FrameTransform(parent_frame_id=WORLD, child_frame_id=SCAN,
                       translation=Vector3(x=0,y=0,z=0),
                       rotation=quat_from_euler(0,0,sp_rot)),
        FrameTransform(parent_frame_id=WORLD, child_frame_id=NACELLE,
                       translation=Vector3(x=0,y=0,z=tower_h),
                       rotation=quat_from_euler(0,0,nacelle_rot)),
        FrameTransform(parent_frame_id=NACELLE, child_frame_id=HUB,
                       translation=Vector3(x=0.6*nacelle_l,y=0,z=0),
                       rotation=Quaternion(x=0,y=0,z=0,w=1)),
    ]))

def hub_center_world(yaw_rad, nacelle_l):
    nacelle_rot=yaw_rad+math.pi/2
    hx_local,hy_local=(0.6*nacelle_l,0.0)
    hub_x= hx_local*math.cos(nacelle_rot)-hy_local*math.sin(nacelle_rot)
    hub_y= hx_local*math.sin(nacelle_rot)+hy_local*math.cos(nacelle_rot)
    hub_z= hub_ht_m
    return hub_x,hub_y,hub_z

def sp1_xy_world(yaw_rad):
    sp_rot=yaw_rad+math.pi/2+math.radians(sp_yaw_offset_deg)
    x_local=-rotor_dia_m/4.0; y_local=+rotor_dia_m*1.125
    c,s=math.cos(sp_rot),math.sin(sp_rot)
    return x_local*c - y_local*s, x_local*s + y_local*c

# ===================== SENSOR AIM =====================
def aim_pan_tilt_from_world_delta(dx,dy,dz):
    psi=math.atan2(dy,dx)
    theta=math.atan2(dz, math.hypot(dx,dy))
    pan = psi + math.pi/2
    tilt= -theta
    return pan,tilt

def sensor_origin_with_offsets(pan,tilt):
    p2t_x,p2t_y,p2t_z=pan_to_tilt_offset
    t2s_x,t2s_y,t2s_z=tilt_to_sensor_offset
    c,s=math.cos(pan),math.sin(pan)
    cy,sy=math.cos(tilt),math.sin(tilt)
    ox=p2t_x*c - p2t_y*s
    oy=p2t_x*s + p2t_y*c
    oz=p2t_z
    t2sx =  t2s_x*cy + t2s_z*sy
    t2sz = -t2s_x*sy + t2s_z*cy
    t2sy =  t2s_y
    ox += t2sx*c - t2sy*s
    oy += t2sx*s + t2sy*c
    oz += t2sz
    return ox,oy,oz

def solve_pan_tilt_to_target(pan_base_world, target_world):
    bx,by,bz=pan_base_world; tx,ty,tz=target_world
    pan,tilt=aim_pan_tilt_from_world_delta(tx-bx,ty-by,tz-bz)
    for _ in range(2):
        ox,oy,oz=sensor_origin_with_offsets(pan,tilt)
        sx,sy,sz=bx+ox,by+oy,bz+oz
        pan,tilt=aim_pan_tilt_from_world_delta(tx-sx,ty-sy,tz-sz)
    return pan,tilt

# ===================== SCENE BUILD =====================
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
                                radius=1.5*rotor_dia_m,
                                ring_width=circle_ring_width_m, segments=circle_segments,
                                color=circle_color, z_offset=0.01),

        # Tracker marker + mast in WORLD
        SceneEntity(frame_id=WORLD, id="tracker_marker", timestamp=now_ts(),
                    lifetime=Duration.from_secs(0),
                    triangles=[TriangleListPrimitive(
                        points=make_filled_disk_tris(0.0, rotor_dia_m*1.125, marker_outer_r_m, marker_segments, z=0.0),
                        color=tracker_color_blue
                    )]),
        make_cylinder_mesh_Z_at(WORLD, "tracker_mast",
                                radius=tracker_mast_radius_m, height=tracker_ht_m,
                                cx=0.0, cy=rotor_dia_m*1.125, segments=48, color=tracker_mast_color),

        # SPs (yaw with turbine)
        make_bullseye_markers_with_labels(SCAN, "sp_markers_bullseye"),

        # SP1 mast (same dims as tracker), authored in SCAN
        make_cylinder_mesh_Z_at(SCAN, "sp1_mast",
                                radius=tracker_mast_radius_m, height=tracker_ht_m,
                                cx=x_local_sp1, cy=y_local_sp1, segments=48, color=tracker_mast_color),
    ]

    return entities, tower_h, nacelle_l, nose_len

# ===================== SIMPLE WEB STATUS SERVER =====================
latest_status = {
    "rotor_dia_m": rotor_dia_m,
    "hub_ht_m": hub_ht_m,
    "system_ht_m": tracker_ht_m,
    "hub_xyz": (0.0, 0.0, hub_ht_m),
    "rotor_rpm": rotor_rpm,
    "yaw_deg": yaw_deg,
    "blade_count": 3,
    "blade_1": 1, "blade_2": 1, "blade_3": 1,
    "tracker_pan_deg": 0.0, "tracker_tilt_deg": 0.0,
    "scanner_pan_deg": 0.0, "scanner_tilt_deg": 0.0,
}

HTML_PAGE = """<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>WTG Status</title>
<style>
  :root { color-scheme: dark; }
  body { font-family: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Arial, 'Segoe UI Symbol', sans-serif;
         margin: 16px; background:#0b0f14; color:#e5e7eb; }
  h2 { margin: 0 0 12px; font-weight: 600; }
  .card { background:#111827; border:1px solid #1f2937; border-radius:12px; padding:16px; }
  table { width:100%; border-collapse:collapse; }
  th, td { text-align:left; padding:10px 12px; }
  th { width:32%; color:#9ca3af; font-weight:500; }
  tr { border-bottom:1px solid #1f2937; }
  tr:last-child { border-bottom:none; }
  .mono { font-variant-numeric: tabular-nums; font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace; }
  .grid { display:grid; gap:12px; grid-template-columns: 1fr; }
  @media (min-width: 860px) { .grid { grid-template-columns: 1fr 1fr; } }
</style>
</head>
<body>
  <h2>WTG Status</h2>
  <div class="grid">
    <div class="card">
      <table id="tbl1"><tbody></tbody></table>
    </div>
    <div class="card">
      <table id="tbl2"><tbody></tbody></table>
    </div>
  </div>
<script>
async function refresh() {
  const r = await fetch('/status.json', { cache: 'no-store' });
  const d = await r.json();

  const rows1 = [
    ['Rotor dia (m)', d.rotor_dia_m.toFixed(2)],
    ['Hub height (m)', d.hub_ht_m.toFixed(2)],
    ['System height (m)', d.system_ht_m.toFixed(2)],
    ['Hub XYZ (m)', `(${d.hub_xyz[0].toFixed(2)}, ${d.hub_xyz[1].toFixed(2)}, ${d.hub_xyz[2].toFixed(2)})`],
    ['Rotor RPM', d.rotor_rpm.toFixed(2)],
    ['Yaw (deg)', d.yaw_deg.toFixed(2)]
  ];
  const rows2 = [
    ['Blade count (global)', d.blade_count],
    ['Blade 1', d.blade_1],
    ['Blade 2', d.blade_2],
    ['Blade 3', d.blade_3],
    ['Tracker pan/tilt (deg)', `${d.tracker_pan_deg.toFixed(2)}, ${d.tracker_tilt_deg.toFixed(2)}`],
    ['Scanner pan/tilt (deg)', `${d.scanner_pan_deg.toFixed(2)}, ${d.scanner_tilt_deg.toFixed(2)}`]
  ];

  const t1 = document.querySelector('#tbl1 tbody');
  const t2 = document.querySelector('#tbl2 tbody');
  const mk = rows => rows.map(([k,v]) => `<tr><th>${k}</th><td class="mono">${v}</td></tr>`).join('');
  t1.innerHTML = mk(rows1);
  t2.innerHTML = mk(rows2);
}
refresh();
setInterval(refresh, 500);
</script>
</body>
</html>
"""

class StatusHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # keep console clean
        pass

    def do_GET(self):
        if self.path in ("/", "/status", "/index.html"):
            page = HTML_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.end_headers()
            self.wfile.write(page)
        elif self.path == "/status.json":
            data = json.dumps(latest_status).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        else:
            self.send_response(404)
            self.end_headers()

def start_status_server(port=STATUS_PORT):
    def run():
        with socketserver.TCPServer(("127.0.0.1", port), StatusHandler) as httpd:
            httpd.serve_forever()
    th = threading.Thread(target=run, daemon=True)
    th.start()
    print(f"[Status] Web table at http://127.0.0.1:{port}/")

# ===================== MAIN =====================
if __name__ == "__main__":
    # Start Foxglove + channels
    foxglove.start_server(host="127.0.0.1", port=8765)
    scene_ch = SceneUpdateChannel(topic="/scene")
    tf_ch = FrameTransformsChannel(topic="/tf")

    # Start status web server (open in Foxglove Web panel)
    start_status_server(STATUS_PORT)

    # Build scene
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

    # Static WORLD root
    tf_ch.log(FrameTransforms(transforms=[
        FrameTransform(parent_frame_id="root", child_frame_id=WORLD,
                       translation=Vector3(x=0,y=0,z=0),
                       rotation=Quaternion(x=0,y=0,z=0,w=1)),
    ]))

    while True:
        t = time.time() - t0

        # Live rebuild if knobs changed
        if (rotor_dia_m != last_rotor or hub_ht_m != last_hub or
            theta_deg_val != last_theta or blade_thickness_m != last_b_thk or
            blade_root_w_m != last_rootw or blade_tip_w_m != last_tipw):
            entities, tower_h, nacelle_l, nose_len = build_scene_entities()
            scene_ch.log(SceneUpdate(entities=entities))
            last_rotor, last_hub = rotor_dia_m, hub_ht_m
            last_theta, last_b_thk = theta_deg_val, blade_thickness_m
            last_rootw, last_tipw = blade_root_w_m, blade_tip_w_m

        # Yaw + frames
        yaw_cmd_deg = yaw_deg + yaw_rate_deg_s * t
        yaw_cmd_rad = math.radians(yaw_cmd_deg)
        publish_wtg_frames(tf_ch, yaw_cmd_rad, tower_h, nacelle_l)

        # Hub target (with vertical offset)
        hub_x, hub_y, hub_z = hub_center_world(yaw_cmd_rad, nacelle_l)
        target_world = (hub_x, hub_y, hub_z + hub_target_z_offset_m)

        # ------------- Tracker (Benewake + LIVOX) -------------
        pan_base_world = (0.0, rotor_dia_m*1.125, tracker_ht_m)
        pan_cmd, tilt_cmd = solve_pan_tilt_to_target(pan_base_world, target_world)

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
            # LIVOX (same)
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

        # Benewake dynamic far (from sensor origin)
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

        # ------------- SP1 S-Laser + Camera -------------
        sp1_xw, sp1_yw = sp1_xy_world(yaw_cmd_rad)
        sp1_base_world = (sp1_xw, sp1_yw, tracker_ht_m)
        sp1_pan, sp1_tilt = solve_pan_tilt_to_target(sp1_base_world, target_world)

        tf_ch.log(FrameTransforms(transforms=[
            # S-Laser chain
            FrameTransform(parent_frame_id=WORLD, child_frame_id=S_PAN,
                           translation=Vector3(x=sp1_base_world[0], y=sp1_base_world[1], z=sp1_base_world[2]),
                           rotation=quat_from_euler(0.0, 0.0, -math.pi/2 + sp1_pan)),
            FrameTransform(parent_frame_id=S_PAN, child_frame_id=S_TILT,
                           translation=Vector3(x=pan_to_tilt_offset[0], y=pan_to_tilt_offset[1], z=pan_to_tilt_offset[2]),
                           rotation=quat_from_euler(0.0, sp1_tilt, 0.0)),
            FrameTransform(parent_frame_id=S_TILT, child_frame_id=S_SENSOR,
                           translation=Vector3(x=tilt_to_sensor_offset[0], y=tilt_to_sensor_offset[1], z=tilt_to_sensor_offset[2]),
                           rotation=Quaternion(x=0,y=0,z=0,w=1)),
            # Camera = identical pose
            FrameTransform(parent_frame_id=S_SENSOR, child_frame_id=C_SENSOR,
                           translation=Vector3(x=0.0, y=0.0, z=0.0),
                           rotation=Quaternion(x=0,y=0,z=0,w=1)),
        ]))

        # Camera frustum + mask (optional)
        scene_ch.log(SceneUpdate(entities=[
            make_fov_frustum_X(C_SENSOR, "sp1_camera_fov",
                               near_m=camera_near_m, far_m=camera_far_m,
                               haov_deg=camera_haov_deg, vaov_deg=camera_vaov_deg,
                               color=camera_color),
            make_camera_view_mask(C_SENSOR, "sp1_camera_mask",
                                  haov_deg=camera_haov_deg, vaov_deg=camera_vaov_deg,
                                  dist_m=camera_mask_dist, pad_scale=camera_mask_pad, alpha=camera_mask_alpha),
        ]))

        # Spin blades
        omega = rot_dir * 2.0*math.pi*(rotor_rpm/60.0)
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

        # -------- Update web table data --------
        latest_status.update({
            "rotor_dia_m": rotor_dia_m,
            "hub_ht_m": hub_ht_m,
            "system_ht_m": tracker_ht_m,
            "hub_xyz": [hub_x, hub_y, hub_z],   # JSON friendly
            "rotor_rpm": rotor_rpm,
            "yaw_deg": yaw_cmd_deg,
            "blade_count": 3,
            "blade_1": 1, "blade_2": 1, "blade_3": 1,
            "tracker_pan_deg": wrap_deg(math.degrees(pan_cmd)),
            "tracker_tilt_deg": wrap_deg(math.degrees(tilt_cmd)),
            "scanner_pan_deg": wrap_deg(math.degrees(sp1_pan)),
            "scanner_tilt_deg": wrap_deg(math.degrees(sp1_tilt)),
        })

        # Periodically resend static scene (late joiners)
        now_s = time.time()
        if now_s - last_scene > 1.0:
            scene_ch.log(SceneUpdate(entities=entities))
            last_scene = now_s

        # Human-friendly rate
        time.sleep(1.0/30.0)
