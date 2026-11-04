"""Conservative 3-D planning against a hash-verified, known static simulator map.

This backend is deliberately separate from LiDAR mapping. No obstacle identities
or geometry are returned to the language model. VIO/PX4 supplies the flight pose;
the simulator pose registers the local frame to the known map once per sortie.
"""
import hashlib
import heapq
import itertools
import math
import uuid
from pathlib import Path
import xml.etree.ElementTree as ET

from mission import MAX_ALTITUDE, MissionError, PathUnavailable


def transform_pose(element):
    pose = element.find('pose')
    if pose is not None and pose.get('relative_to'):
        raise MissionError('static map requires explicit enclosing-frame poses')
    v = [float(x) for x in element.findtext('pose', '0 0 0 0 0 0').split()]
    if len(v) != 6 or not all(math.isfinite(x) for x in v):
        raise MissionError('invalid static map pose')
    x,y,z,r,p,w = v
    cr,sr,cp,sp,cw,sw = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(w),math.sin(w)
    return ([[cw*cp,cw*sp*sr-sw*cr,cw*sp*cr+sw*sr],
             [sw*cp,sw*sp*sr+cw*cr,sw*sp*cr-cw*sr],[-sp,cp*sr,cp*cr]], [x,y,z])


def apply(transform, point):
    rotation, offset = transform
    return [sum(row[j]*point[j] for j in range(3))+offset[i] for i,row in enumerate(rotation)]


def collision_boxes(world, expected_hash):
    raw = Path(world).read_bytes()
    if hashlib.sha256(raw).hexdigest() != expected_hash:
        raise MissionError('static navigation world hash mismatch')
    node = ET.fromstring(raw).find('world')
    if node is None or any(i.findtext('uri') != 'model://sun' for i in node.findall('include')):
        raise MissionError('static map contains unverified included geometry')
    boxes = []
    for model in node.findall('model'):
        if model.findtext('static') != 'true' or model.findall('model'):
            raise MissionError('static navigation requires static, unnested models')
        if model.get('name') == 'yard_ground':
            continue  # Ground is independently bounded by the permitted altitude.
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                if geometry.find('box') is not None:
                    half = [float(v)/2 for v in geometry.findtext('box/size').split()]
                elif geometry.find('cylinder') is not None:
                    radius = float(geometry.findtext('cylinder/radius'))
                    half = [radius,radius,float(geometry.findtext('cylinder/length'))/2]
                elif geometry.find('sphere') is not None:
                    half = [float(geometry.findtext('sphere/radius'))]*3
                else:
                    raise MissionError('unsupported collision shape in static navigation map')
                if len(half) != 3 or not all(math.isfinite(v) and v > 0 for v in half):
                    raise MissionError('invalid static collision dimensions')
                poses = [transform_pose(e) for e in (collision,link,model)]
                points = []
                for corner in itertools.product(*[(-v,v) for v in half]):
                    for transform in poses:
                        corner = apply(transform,corner)
                    points.append(corner)
                boxes.append(([min(p[i] for p in points) for i in range(3)],
                              [max(p[i] for p in points) for i in range(3)]))
    if not boxes:
        raise MissionError('static navigation map has no verified obstacles')
    return boxes


def intersects(start, end, low, high):
    """Exact slab intersection, including contact; no sampling gaps."""
    enter, leave = 0., 1.
    for i in range(3):
        delta = end[i]-start[i]
        if abs(delta) < 1e-12:
            if start[i] < low[i] or start[i] > high[i]:
                return False
        else:
            a,b = (low[i]-start[i])/delta,(high[i]-start[i])/delta
            enter,leave = max(enter,min(a,b)),min(leave,max(a,b))
            if enter > leave:
                return False
    return True


class StaticNavigator:
    def __init__(self, boxes, bounds, clearance=1., resolution=.5):
        self.raw_boxes = [([*low], [*high]) for low, high in boxes]
        self.registration_id = None
        self.world_sha256, self.ground_world_z = None, 0.
        self.bounds = [list(bounds[0]),list(bounds[1])]
        self.boxes = [([v-clearance for v in low],[v+clearance for v in high])
                      for low,high in boxes if all(high[i]+clearance >= bounds[0][i]
                                                   and low[i]-clearance <= bounds[1][i] for i in range(3))]
        self.clearance,self.resolution = clearance,resolution
        self.origin,self.yaw = None,None
        self.last_plan = None

    @classmethod
    def from_scene(cls, world, scene):
        region = scene['flight_region']
        margin = region['margin_m']
        low,high = list(region['bounds_min_m']),list(region['bounds_max_m'])
        for i in (0,1):
            low[i] += margin; high[i] -= margin
        low[2] = max(low[2], scene['ground_plane']['z_m']+.8)
        high[2] -= .2
        result = cls(collision_boxes(world, scene['world_sha256']), [low,high],
                     scene['navigation']['minimum_clearance_m'])
        result.world_sha256 = scene['world_sha256']
        result.ground_world_z = scene['ground_plane']['z_m']
        return result

    def bind(self, local, local_q, world):
        from scene_config import quaternion_yaw
        self.yaw = quaternion_yaw(world['orientation_xyzw'])-quaternion_yaw(local_q)
        c,s = math.cos(self.yaw),math.sin(self.yaw)
        self.origin = [world['position_m'][0]-c*local[0]+s*local[1],
                       world['position_m'][1]-s*local[0]-c*local[1],world['position_m'][2]-local[2]]
        self.registration_id = uuid.uuid4().hex

    def observation_visibility(self, pose, camera_info, grid_origin_xy, radius=12., resolution=.5):
        """Ground-centre LOS using raw collision AABBs, never flight inflation or semantics."""
        from coverage_search import project_ground_point
        from stereo_localization import quaternion_rotation, matvec, iris_left_optical_to_body, vector
        if self.origin is None or self.registration_id is None:
            raise MissionError('visibility requires an actual sortie registration')
        if radius != 12. or resolution != .5:
            raise MissionError('MVP visibility grid requires radius 12m and resolution 0.5m')
        centre = vector(grid_origin_xy, 2, 'grid origin ENU')
        rotation = quaternion_rotation(pose['orientation_xyzw'])
        offset = matvec(rotation, iris_left_optical_to_body()['translation_m'])
        camera = [p+d for p,d in zip(pose['position_m'], offset)]
        camera_world = self.to_world(camera)
        ground = self.ground_world_z-self.origin[2]
        eligible, visible, in_fov = [], [], 0
        for row in range(48):
            for col in range(48):
                point = [centre[0]-12+(col+.5)*.5, centre[1]-12+(row+.5)*.5, ground]
                world = self.to_world([*point[:2], ground+.01])
                if (math.dist(point[:2], centre) > 12 or
                    any(not self.bounds[0][i] <= world[i] <= self.bounds[1][i] for i in (0,1)) or
                    any(intersects(world, world, low, high) for low,high in self.raw_boxes)):
                    continue
                index = row*48+col
                eligible.append(index)
                if not project_ground_point(camera_info, pose, point, 10.)['inside_fov_and_range']:
                    continue
                in_fov += 1
                if not any(intersects(camera_world, world, low, high) for low,high in self.raw_boxes):
                    visible.append(index)
        return {'schema_version': 1, 'source': 'verified_raw_static_collision_aabb_visibility',
            'world_sha256': self.world_sha256, 'sortie_id': self.registration_id,
            'registration': {'world_from_enu_origin_m': list(self.origin), 'world_from_enu_yaw_rad': self.yaw,
                             'ground_world_z_m': self.ground_world_z},
            'grid': {'origin_xy_enu_m': centre, 'radius_m': radius, 'resolution_m': resolution,
                     'rows': 48, 'cols': 48, 'ground_enu_z_m': ground},
            'eligible_cell_ids': eligible, 'visible_cell_ids': visible,
            'in_fov_cell_count': in_fov, 'occluded_cell_count': in_fov-len(visible),
            'camera_origin_enu_m': camera,
            'limitations': ['centre-sampled geometric visibility; not recognition or absence proof',
                            'raw enclosing collision AABBs; no flight inflation, conservative for rotated/curved shapes']}

    def to_world(self, point):
        if self.origin is None:
            raise MissionError('static map local-frame registration is unavailable')
        c,s = math.cos(self.yaw),math.sin(self.yaw)
        return [c*point[0]-s*point[1]+self.origin[0],s*point[0]+c*point[1]+self.origin[1],point[2]+self.origin[2]]

    def raw_line_of_sight(self, camera_enu, target_enu):
        """Optical prediction using raw geometry; only exact endpoint contact is trimmed."""
        camera, target = self.to_world(camera_enu), self.to_world(target_enu)
        distance = math.dist(camera, target)
        if distance > 1e-4:
            target = [b+(a-b)*1e-4/distance for a, b in zip(camera, target)]
        return not any(intersects(camera, target, low, high) for low, high in self.raw_boxes)

    def to_local(self, point):
        x,y,z = [v-o for v,o in zip(point,self.origin)]
        c,s = math.cos(self.yaw),math.sin(self.yaw)
        return [c*x+s*y,-s*x+c*y,z]

    def world_segment_safe(self, start, end):
        if any(len(p)!=3 or any(not math.isfinite(v) or not self.bounds[0][i] <= v <= self.bounds[1][i]
                                for i,v in enumerate(p)) for p in (start,end)):
            return False
        return not any(intersects(start,end,low,high) for low,high in self.boxes)

    def segment_safe(self, start, end, now=None):
        return self.world_segment_safe(self.to_world(start),self.to_world(end))

    def vertical_safe(self, start, end):
        a,b = self.to_world(start),self.to_world(end)
        return (math.dist(a[:2],b[:2]) < 1e-6
                and all(self.bounds[0][i] <= a[i] <= self.bounds[1][i] for i in (0,1))
                and min(a[2],b[2]) >= -.2 and max(a[2],b[2]) <= self.bounds[1][2]
                and not any(intersects(a,b,low,high) for low,high in self.boxes))

    def plan(self, start, goal, now=None, *, search_origin_xy=None, search_radius_m=12., record=True):
        """Plan continuously safe segments; optional search disk constrains the full route."""
        from stereo_localization import vector, finite
        origin = None if search_origin_xy is None else vector(search_origin_xy, 2, 'search origin')
        radius = finite(search_radius_m, 'search radius')
        if not 0 < radius <= 12:
            raise MissionError('search radius must be in (0,12] metres')
        def permitted(p):
            if origin is None:
                return True
            local = self.to_local(p)
            return 1.-1e-9 <= local[2] <= MAX_ALTITUDE+1e-9 and math.dist(local[:2], origin) <= radius+1e-9
        def segment(a, b):
            # A disk and height slab are convex: endpoint inclusion bounds every segment.
            return permitted(a) and permitted(b) and self.world_segment_safe(a, b)
        a,b = self.to_world(start),self.to_world(goal)
        if not segment(a,a):
            raise MissionError('3-D start/goal lacks obstacle, boundary or search-volume clearance')
        if not segment(b,b):
            raise PathUnavailable('3-D start/goal lacks obstacle, boundary or search-volume clearance')
        route = [a,b]
        if not segment(a,b):
            low,high = self.bounds
            def cell(p): return tuple(int(round((p[i]-low[i])/self.resolution)) for i in range(3))
            def center(k): return [low[i]+k[i]*self.resolution for i in range(3)]
            def cruise_height(p): return 1. <= self.to_local(p)[2] <= MAX_ALTITUDE
            initial = cell(a)
            # Connect exact endpoints to grid centres only through checked segments.
            seeds = [tuple(initial[i]+d[i] for i in range(3)) for d in itertools.product((-1,0,1),repeat=3)]
            costs,previous,queue = {},{},[]
            for k in seeds:
                p = center(k)
                if cruise_height(p) and segment(a,p):
                    costs[k] = math.dist(a,p); previous[k] = None
                    heapq.heappush(queue,(costs[k]+math.dist(p,b),costs[k],k))
            deltas = [d for d in itertools.product((-1,0,1),repeat=3) if any(d)]
            visited = set(); endpoint = None
            while queue and len(visited) < 100000:
                _,cost,k = heapq.heappop(queue)
                if k in visited or cost != costs.get(k): continue
                visited.add(k); p = center(k)
                if segment(p,b): endpoint=k; break
                for delta in deltas:
                    nxt = tuple(k[i]+delta[i] for i in range(3)); q = center(nxt)
                    distance = cost+math.dist(p,q)
                    if (cruise_height(q) and distance < costs.get(nxt,float('inf'))
                            and segment(p,q)):
                        costs[nxt],previous[nxt] = distance,k
                        heapq.heappush(queue,(distance+math.dist(q,b),distance,nxt))
            if endpoint is None: raise PathUnavailable('no collision-free route in known static 3-D map')
            route = [b]
            while endpoint is not None:
                route.append(center(endpoint)); endpoint = previous[endpoint]
            route.append(a); route.reverse()
        # Remove unnecessary turns, retaining exact continuous collision checks.
        simplified = [route[0]]; index = 0
        while index < len(route)-1:
            nxt = len(route)-1
            while not segment(route[index],route[nxt]): nxt -= 1
            simplified.append(route[nxt]); index = nxt
        result = {'source':'verified_static_collision_map_3d',
                          'waypoints':[self.to_local(p) for p in simplified[1:]],
                          'route_complete':True,'goal_known':True,'frontier':False,
                          'obstacle_inflation_m':self.clearance,
                          'path_length_m':sum(math.dist(a,b) for a,b in zip(simplified,simplified[1:])),
                          'goal_enu_m':list(goal),'start_enu_m':list(start)}
        if origin is not None:
            result.update(search_origin_xy_enu_m=origin, search_radius_m=radius,
                          full_route_within_search_volume=True)
        if record:
            self.last_plan = result
        return result.copy()

    def preview_path(self, start, goal, search_origin_xy, search_radius_m=12.):
        """Candidate planning must not replace the executing flight's last_plan."""
        return self.plan(start, goal, search_origin_xy=search_origin_xy,
                         search_radius_m=search_radius_m, record=False)

    def diagnostics(self, now=None, position=None, include_map=False):
        result = {'source':'verified_static_collision_map_3d','method':'static_3d',
                'fresh':True,'source_age_s':None,'scan_count':0,'last_plan':self.last_plan,
                'obstacle_inflation_m':self.clearance,'map_registered':self.origin is not None,
                'scope':'known static 3-D collision geometry; no dynamic-obstacle guarantee',
                'lidar_role':'telemetry_only; not the source of this map'}
        if self.origin is not None:
            low,high = self.bounds
            centre = self.to_local([(a+b)/2 for a,b in zip(low,high)])
            hx,hy = (high[0]-low[0])/2,(high[1]-low[1])/2
            c,s = abs(math.cos(self.yaw)),abs(math.sin(self.yaw))
            ratio = min(hx/(c*hx+s*hy),hy/(s*hx+c*hy))
            ex,ey = hx*ratio,hy*ratio
            result.update(bounds_xy_enu_m=[centre[0]-ex,centre[1]-ey,centre[0]+ex,centre[1]+ey],
                          registration_id=self.registration_id,
                          visible_clearance_m=max(1.2,self.clearance))
        return result

    snapshot = diagnostics
