"""Three-drone, shared-map field survey using a synthetic 3D range sensor.

This is a reproducible perception/planning experiment, not a flight controller.
Truth geometry is restricted to the sensor and evaluation. The planner sees only
the shared evidence map, candidate camera geometry, and a pre-incident roof prior.
"""
from dataclasses import dataclass
import base64
import io
import json
from pathlib import Path
import numpy as np
from PIL import Image
from .geometry import Camera, Pose


SIZE = 48.0
RESOLUTION = .5
N = int(SIZE / RESOLUTION)
CAMERA = Camera(width=48, height=36, hfov=82, vfov=64, pitch=68, far=65)
SPEED = 3.0
OBS_SECONDS = 2.5
ALTITUDES = (13.0, 14.0, 15.0)
HOMES = ((4., 4.), (4., 44.), (44., 4.))


@dataclass(frozen=True)
class Structure:
    name: str
    x0: float
    y0: float
    x1: float
    y1: float
    roof: float

    def contains(self, x, y):
        return (x >= self.x0) & (x < self.x1) & (y >= self.y0) & (y < self.y1)


STRUCTURES = (
    Structure("FIELD STATION", 11., 12., 22., 20., 5.8),
    Structure("SERVICE DEPOT", 30., 27., 40., 36., 6.6),
)


class FieldWorld:
    """Hidden synthetic surface with terrain, canopies, and changed roofs."""

    def __init__(self, seed=17):
        self.seed = seed
        rng = np.random.default_rng(seed)
        axis = (np.arange(N) + .5) * RESOLUTION
        x, y = np.meshgrid(axis, axis)
        terrain = (.65 + .55*np.sin(x/6.2)*np.cos(y/8.1)
                   + 2.2*np.exp(-((x-36)**2/65+(y-10)**2/100))
                   + 1.0*np.exp(-((x-16)**2/110+(y-39)**2/38)))
        self.terrain = np.maximum(terrain, .05)
        self.surface = self.terrain.copy()
        self.material = np.zeros((N,N), dtype=np.uint8)  # 0 terrain, 1 canopy, 2 structure
        self.damage = np.zeros((N,N), dtype=bool)
        self.prior = np.full((N,N), np.nan)
        trees = []
        for _ in range(43):
            cx, cy = rng.uniform(5,43,2)
            if any(s.contains(cx,cy) for s in STRUCTURES):
                continue
            if not ((cx<26 and cy>27) or (cx>26 and cy>37) or (cx<11 and cy<34)):
                continue
            radius = float(rng.uniform(.65,1.65))
            height = float(rng.uniform(2.2,5.2))
            trees.append((round(float(cx),2),round(float(cy),2),round(radius,2),round(height,2)))
            r2=(x-cx)**2+(y-cy)**2
            canopy=r2<radius**2
            crown=height*(.7+.3*np.sqrt(np.maximum(0,1-r2/radius**2)))
            self.surface[canopy]=np.maximum(self.surface[canopy],self.terrain[canopy]+crown[canopy])
            self.material[canopy]=1
        self.trees=trees
        for index,s in enumerate(STRUCTURES):
            mask=s.contains(x,y)
            self.prior[mask]=s.roof
            # An irregular missing-roof region and a sagged roof corner.
            if index==0:
                changed=mask & (((x-17.8)**2/12+(y-16.1)**2/5)<1)
            else:
                changed=mask & (((x>35.5)&(y>31.))|(((x-33.)**2+(y-30.)**2)<2))
            self.damage |= changed
            self.surface[mask]=s.roof
            self.surface[changed]=self.terrain[changed]+rng.uniform(.5,1.9,changed.sum())
            self.material[mask]=2

    def scan(self, pose, frame_index):
        """Ray-marched metric range image. Missing returns stay missing."""
        rays=CAMERA.rays(pose).reshape(-1,3)
        origin=pose.xyz
        ranges=np.full(len(rays),np.nan)
        live=np.ones(len(rays),dtype=bool)
        # .35m march is an explicit resolution limit; sensor noise is deterministic.
        for distance in np.arange(.35,CAMERA.far,.35):
            if not live.any():
                break
            ids=np.flatnonzero(live)
            pts=origin+rays[ids]*distance
            valid=(pts[:,0]>=0)&(pts[:,0]<SIZE)&(pts[:,1]>=0)&(pts[:,1]<SIZE)
            ix=np.clip((pts[:,0]/RESOLUTION).astype(int),0,N-1)
            iy=np.clip((pts[:,1]/RESOLUTION).astype(int),0,N-1)
            hit=valid&(pts[:,2]<=self.surface[iy,ix])
            ranges[ids[hit]]=distance
            live[ids[hit]]=False
        rng=np.random.default_rng(self.seed*10000+frame_index)
        ranges += rng.normal(0,.06,len(rays))
        ranges[rng.random(len(rays))<.025]=np.nan
        return ranges.reshape(CAMERA.height,CAMERA.width)

    def reference(self):
        return {"terrain":self.terrain,"surface":self.surface,
                "material":self.material,"damage":self.damage}


class SharedSurfaceMap:
    def __init__(self):
        self.height=np.full((N,N),np.nan)
        self.count=np.zeros((N,N),dtype=np.uint16)
        self.sources=np.zeros((N,N),dtype=np.uint8)
        self.deviation=np.zeros((N,N),dtype=bool)

    def update(self, pose, ranges, drone_id, prior):
        rays=CAMERA.rays(pose).reshape(-1,3)
        flat=ranges.ravel()
        valid=np.isfinite(flat)&(flat>CAMERA.near)&(flat<CAMERA.far)
        points=pose.xyz+rays[valid]*flat[valid,None]
        within=(points[:,0]>=0)&(points[:,0]<SIZE)&(points[:,1]>=0)&(points[:,1]<SIZE)&(points[:,2]>=-.25)
        points=points[within]
        ids=np.floor(points[:,:2]/RESOLUTION).astype(int)
        old=int((self.count>0).sum())
        # Aggregate the measurement within each cell before fusion; this avoids
        # pixel-density weighting and permits agreement by independent drones.
        linear=ids[:,1]*N+ids[:,0]
        unique,inverse=np.unique(linear,return_inverse=True)
        sums=np.bincount(inverse,weights=points[:,2])
        nums=np.bincount(inverse)
        iy,ix=unique//N,unique%N
        old_count=self.count[iy,ix].astype(float)
        previous=np.nan_to_num(self.height[iy,ix],nan=0)
        self.height[iy,ix]=(previous*old_count+sums/nums)/(old_count+1)
        self.count[iy,ix]+=1
        self.sources[iy,ix] |= (1<<drone_id)
        valid_roof=np.isfinite(prior)&(self.count>0)
        self.deviation=valid_roof&(self.height<prior-1.0)
        return int((self.count>0).sum())-old,points

    def snapshot(self):
        return {"height":np.round(np.nan_to_num(self.height,nan=-1),2).ravel().tolist(),
                "sources":self.sources.ravel().tolist(),
                "deviation":np.flatnonzero(self.deviation).tolist(),
                "known":int((self.count>0).sum())}


class VoxelEvidenceMap:
    """0.5 m 3D occupancy evidence from ray endpoints and traversed free air.

    Rows are z, y, x. Unknown is kept distinct from observed free space.
    This ray-carved grid does not reconstruct unobserved surfaces or semantics.
    """

    def __init__(self, ceiling=16.0):
        self.nz=int(round(ceiling/RESOLUTION))
        self.free=np.zeros((self.nz,N,N),dtype=np.uint16)
        self.occupied=np.zeros_like(self.free)

    def update(self,pose,ranges):
        depth=ranges.ravel()
        rays=CAMERA.rays(pose).reshape(-1,3)
        valid=np.isfinite(depth)&(depth>CAMERA.near)&(depth<CAMERA.far)
        depth=depth[valid]
        rays=rays[valid]
        if not len(depth):
            return
        origin=pose.xyz
        ends=origin+rays*depth[:,None]
        steps=np.arange(.5,float(depth.max()),.75)
        samples=origin+rays[:,None,:]*steps[None,:,None]
        free_mask=(steps[None,:]<depth[:,None]-.6)
        free_pts=samples[free_mask]

        def indices(points):
            inside=(points[:,0]>=0)&(points[:,0]<SIZE)&(points[:,1]>=0)&(points[:,1]<SIZE)&(points[:,2]>=0)&(points[:,2]<self.nz*RESOLUTION)
            cells=np.floor(points[inside]/RESOLUTION).astype(int)
            return np.unique((cells[:,2]*N+cells[:,1])*N+cells[:,0])

        free_ids=indices(free_pts)
        occ_ids=indices(ends)
        self.free.ravel()[free_ids]+=1
        self.occupied.ravel()[occ_ids]+=1

    def labels(self):
        touched=(self.free+self.occupied)>0
        occupied=(self.occupied>0)&(self.occupied>=self.free)
        return np.where(~touched,-1,occupied.astype(np.int8))

    def summary(self):
        labels=self.labels()
        return {"free_voxels":int((labels==0).sum()),
                "occupied_voxels":int((labels==1).sum()),
                "unknown_voxels":int((labels==-1).sum()),
                "voxel_resolution_m":RESOLUTION}


def candidates():
    result=[]
    for y in np.arange(6,43,6):
        for x in np.arange(6,43,6):
            for yaw in np.arange(0,2*np.pi,np.pi/2):
                result.append((float(x),float(y),float(yaw)))
    return result


def run_swarm(seed=17, rounds=8, coordinated=True):
    if rounds<1 or rounds>18:
        raise ValueError("rounds must be between 1 and 18")
    world=FieldWorld(seed)
    belief=SharedSurfaceMap()
    voxel=VoxelEvidenceMap()
    cx,cy=np.meshgrid((np.arange(N)+.5)*RESOLUTION,(np.arange(N)+.5)*RESOLUTION)
    centers=np.column_stack([cx.ravel(),cy.ravel(),np.zeros(N*N)])
    candidate_list=candidates()
    # Candidate frustums assume a flat ground plane. Known occlusion and unknown
    # vegetation can make the information-gain forecast optimistic.
    visible=[]
    for x,y,yaw in candidate_list:
        pose=Pose(x,y,14.,yaw)
        visible.append(CAMERA.in_view(pose,centers))
    poses=[Pose(*home,ALTITUDES[i],0) for i,home in enumerate(HOMES)]
    distances=[0.,0.,0.]
    available=set(range(len(candidate_list)))
    events=[]
    sequence=0
    round_start=0.
    for cycle in range(rounds):
        assigned=[]
        reserved=np.zeros(N*N,dtype=bool)
        unseen=(belief.count.ravel()==0)
        for drone_id in range(3):
            if cycle==0:
                target=poses[drone_id]
                chosen=None
            else:
                scores=[]
                for j in available:
                    x,y,yaw=candidate_list[j]
                    travel=np.hypot(x-poses[drone_id].x,y-poses[drone_id].y)
                    gain=int((visible[j]&unseen&(~reserved if coordinated else True)).sum())
                    # A small distance penalty and reservation spread the team.
                    scores.append((gain/(1+.04*travel),gain,-travel,-j,j))
                if not scores:
                    break
                *_,chosen=max(scores)
                x,y,yaw=candidate_list[chosen]
                target=Pose(x,y,ALTITUDES[drone_id],yaw)
                if coordinated:
                    reserved |= visible[chosen]&unseen
                available.remove(chosen)
            travel=poses[drone_id].distance(target)
            arrival=round_start+travel/SPEED
            assigned.append((arrival+OBS_SECONDS,drone_id,target,poses[drone_id],travel,chosen))
        if not assigned:
            break
        for finish,drone_id,target,start,travel,chosen in sorted(assigned):
            # Separate altitude lanes; world surface is lower than every lane.
            ranges=world.scan(target,sequence)
            gained,points=belief.update(target,ranges,drone_id,world.prior)
            voxel.update(target,ranges)
            valid=np.isfinite(ranges)
            lo,hi=(float(np.min(ranges[valid])),float(np.max(ranges[valid]))) if valid.any() else (0.,1.)
            scaled=np.clip((np.nan_to_num(ranges,nan=hi)-lo)/max(.1,hi-lo),0,1)
            image=(225-165*scaled).astype(np.uint8)
            image[~valid]=14
            buffer=io.BytesIO()
            Image.fromarray(image).save(buffer,format="PNG")
            distances[drone_id]+=travel
            sample=points[::max(1,len(points)//700)]
            event={"step":sequence,"round":cycle,"drone":drone_id,
                   "start":{"x":start.x,"y":start.y,"z":start.z,"yaw":start.yaw},
                   "pose":{"x":target.x,"y":target.y,"z":target.z,"yaw":target.yaw},
                   "start_s":round_start,"capture_s":finish,
                   "new_cells":gained,"distance_m":round(distances[drone_id],2),
                   "valid_returns":int(np.isfinite(ranges).sum()),
                   "range_png":"data:image/png;base64,"+base64.b64encode(buffer.getvalue()).decode(),
                   "range_min_m":round(lo,1),"range_max_m":round(hi,1),
                   "points":np.round(sample,2).tolist(),
                   "map":belief.snapshot(),"volume":voxel.summary(),"choice":chosen}
            events.append(event)
            poses[drone_id]=target
            sequence+=1
        round_start=max(item[0] for item in assigned)+.75
    known=belief.count>0
    roof_known=known&world.damage
    predicted=belief.deviation
    tp=int((predicted&world.damage).sum())
    fp=int((predicted&~world.damage).sum())
    report={"coverage_pct":round(float(known.mean()*100),1),
            "mean_height_error_m":round(float(np.abs(belief.height[known]-world.surface[known]).mean()),2),
            "damage_recall_observed":round(tp/int(roof_known.sum()),3) if roof_known.any() else None,
            "damage_precision":round(tp/(tp+fp),3) if tp+fp else None,
            "anomaly_cells":int(predicted.sum()),
            "multi_drone_overlap_cells":int(np.count_nonzero((belief.sources&(belief.sources-1))!=0)),
            "distance_by_drone_m":[round(d,1) for d in distances],
            "samples":sum(e["valid_returns"] for e in events)}
    report.update(voxel.summary())
    return {"schema":"sentinel-swarm-v1","seed":seed,"size":SIZE,"resolution":RESOLUTION,
            "allocation":"coordinated" if coordinated else "independent",
            "camera":{"width":CAMERA.width,"height":CAMERA.height,"hfov":CAMERA.hfov,
                      "vfov":CAMERA.vfov,"pitch":CAMERA.pitch},
            "homes":[list(h)+[ALTITUDES[i]] for i,h in enumerate(HOMES)],
            "prior":{"structures":[s.__dict__ for s in STRUCTURES],
                     "roof_height":np.nan_to_num(world.prior,nan=-1).ravel().tolist()},
            "reference":{"surface":np.round(world.surface,2).ravel().tolist(),
                         "material":world.material.ravel().tolist(),
                         "damage":np.flatnonzero(world.damage).tolist(),
                         "trees":world.trees},
            "events":events,"report":report,
            "voxel":{"shape":[voxel.nz,N,N],"labels":voxel.labels().ravel().tolist()},
            "assumptions":["Synthetic 2.5D field and ray-marched depth; not live LiDAR or radar",
                           "Known exact pose; no SLAM, GPS error, or sensor calibration",
                           "Three separate altitude lanes, constant-speed travel, no flight dynamics or return planning",
                           "Height surface and ray-carved 0.5 m 3D occupancy grid; not a dense 3D reconstruction",
                           "Prior intact roof height is supplied for the change detector",
                           "Planner uses camera frustum and shared map, not reference surface",
                           "Reference surface and damage labels are used for evaluation and illustration only"]}


def save_swarm(payload,folder):
    folder=Path(folder)
    folder.mkdir(parents=True,exist_ok=True)
    (folder/"swarm.json").write_text(json.dumps(payload,separators=(",",":"),allow_nan=False))
    volume=np.array(payload["voxel"]["labels"],dtype=np.int8).reshape(payload["voxel"]["shape"])
    np.savez_compressed(folder/"voxel_grid.npz",labels=volume,resolution_m=RESOLUTION,
                        axis_order="z,y,x",label_meanings="-1 unknown; 0 free; 1 occupied")
    # These are recorded range endpoints in metric ENU coordinates. Their sample
    # density is reduced for a portable replay, and they are not a mesh or SLAM.
    points=[(x,y,z,e["drone"]) for e in payload["events"] for x,y,z in e["points"]]
    header=("ply\nformat ascii 1.0\ncomment synthetic range endpoints in metric ENU\n"
            f"element vertex {len(points)}\nproperty float x\nproperty float y\n"
            "property float z\nproperty uchar drone\nend_header\n")
    with (folder/"measured_points.ply").open("w") as stream:
        stream.write(header)
        for x,y,z,drone in points:
            stream.write(f"{x:.2f} {y:.2f} {z:.2f} {drone}\n")
    template=(Path(__file__).parent/"assets"/"swarm.html").read_text()
    token="__ATLAS_SWARM_DATA__"
    if template.count(token)!=1:
        raise ValueError("Swarm template needs one payload placeholder")
    (folder/"swarm.html").write_text(template.replace(token,json.dumps(payload,separators=(",",":"),allow_nan=False).replace("<","\\u003c")))
    # Route workspace runs locally in the browser with the completed evidence.
    nav_data={"size":payload["size"],"resolution":payload["resolution"],
              "events":[{"map":payload["events"][-1]["map"]}],"voxel":payload["voxel"]}
    nav_template=(Path(__file__).parent/"assets"/"navigation.html").read_text()
    nav_core=(Path(__file__).parent/"assets"/"navigation.js").read_text()
    (folder/"navigation.html").write_text(nav_template.replace("__NAV_DATA__",json.dumps(
        nav_data,separators=(",",":"),allow_nan=False).replace("<","\\u003c")).replace("__NAV_CORE__",nav_core))
    field_template=(Path(__file__).parent/"assets"/"field.html").read_text()
    field_core=(Path(__file__).parent/"assets"/"field_core.js").read_text()
    (folder/"field.html").write_text(field_template.replace(
        "__FIELD_DATA__",json.dumps(payload,separators=(",",":"),allow_nan=False).replace("<","\\u003c")
    ).replace("__FIELD_NAV_CORE__",nav_core).replace("__FIELD_CORE__",field_core))
    return folder/"field.html"


def benchmark_swarm(seeds=range(17,27),rounds=8):
    """Paired scenarios compare team reservation to independent viewpoint choice."""
    runs=[]
    for seed in seeds:
        for coordinated in (True,False):
            payload=run_swarm(seed,rounds,coordinated)
            runs.append({"seed":seed,"allocation":payload["allocation"],**payload["report"]})
    summary={}
    for mode in ("coordinated","independent"):
        group=[r for r in runs if r["allocation"]==mode]
        summary[mode]={key:round(float(np.mean([r[key] for r in group if r[key] is not None])),3)
                       for key in ("coverage_pct","mean_height_error_m",
                                   "damage_recall_observed","damage_precision",
                                   "multi_drone_overlap_cells")}
        summary[mode]["mean_total_distance_m"]=round(float(np.mean(
            [sum(r["distance_by_drone_m"]) for r in group])),2)
    return {"rounds":rounds,"seeds":list(seeds),"summary":summary,"runs":runs,
            "notes":["Synthetic scenes vary tree positions and sensor noise, not structural layout",
                     "Independent overlap counts cells seen by multiple drones; it is not wasted flight distance",
                     "The planner assumes a flat ground plane when estimating candidate visibility",
                     "This small paired study does not establish general field performance"]}
