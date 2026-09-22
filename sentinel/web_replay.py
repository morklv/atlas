"""Export a portable, offline browser replay of recorded inspection evidence.

The viewer animates presentation between logged poses; it never reruns planning.
Only the scene renderer receives hidden world geometry. Map colors come from the
saved belief snapshots, and sensor rays stop at recorded measured endpoints.
"""
import base64
from dataclasses import asdict
import io
import json
from pathlib import Path
import numpy as np
from PIL import Image
from .geometry import Camera, Pose
from .mapping import EvidenceMap
from .sensor import DepthFrame


def export_replay(result, world, folder):
    folder = Path(folder)
    m = result.metadata
    camera = Camera(**m["camera"])
    frames = []
    previous = np.full_like(result.snapshots[0], -1)
    for i, (event, depth, labels, heights) in enumerate(zip(
            m["events"], result.depths, result.snapshots, result.heights)):
        pose = Pose(**event["pose"])
        valid = np.isfinite(depth)
        finite = depth[valid]
        lo, hi = (float(np.min(finite)), float(np.max(finite))) if len(finite) else (0., 1.)
        normalized = np.clip((np.nan_to_num(depth, nan=lo)-lo)/max(hi-lo,.1),0,1)
        # Sequential near/far palette; black is missing data, never zero distance.
        near, far = np.array([237,179,103]), np.array([66,113,132])
        rgb = (near[None,None,:]*(1-normalized[...,None])
               + far[None,None,:]*normalized[...,None]).astype(np.uint8)
        rgb[~valid] = [12,18,23]
        buffer = io.BytesIO()
        Image.fromarray(rgb).save(buffer, format="PNG")
        points = pose.xyz + camera.rays(pose) * depth[...,None]
        sampled = points[::5, ::5].reshape(-1,3)
        sampled = sampled[np.all(np.isfinite(sampled),axis=1)]
        # An illustrative blocked line of sight, drawn from an actual surface hit.
        occlusion = None
        for point in sampled[np.argsort(-sampled[:,2])]:
            if point[2] < .5:
                break
            direction = point-pose.xyz
            if direction[2] >= -.1:
                continue
            ground = pose.xyz + direction*(-pose.z/direction[2])
            if np.all(ground[:2] > .5) and np.all(ground[:2] < world.size-.5):
                occlusion = [pose.xyz.tolist(),point.tolist(),ground.tolist()]
                break
        predicted = []
        if i+1 < len(m["events"]):
            belief = EvidenceMap(world.size,m["config"]["resolution"])
            belief.hits[labels==1] = 1
            belief.free_hits[labels==0] = 1
            belief.heights[:] = heights
            visible = belief.predicted_visibility(Pose(**m["events"][i+1]["pose"]),camera)
            predicted = np.flatnonzero(visible & (labels==-1)).tolist()
        frames.append({"event": event, "labels": labels.ravel().tolist(),
                       "new": np.flatnonzero((previous==-1)&(labels!=-1)).tolist(),
                       "changed": np.flatnonzero(previous!=labels).tolist(),
                       "points": np.round(sampled,3).tolist(),
                       "occlusion": np.round(occlusion,3).tolist() if occlusion else None,
                       "predicted": predicted,
                       "depthPNG": "data:image/png;base64,"+base64.b64encode(buffer.getvalue()).decode(),
                       "depthMin": round(lo,2), "depthMax": round(hi,2)})
        previous = labels
    payload = {"metadata": m, "scene": {"size":world.size,"boxes":[asdict(b) for b in world.boxes]},
               "gridSize": result.snapshots[0].shape[0], "frames": frames}
    serialized = json.dumps(payload,separators=(",",":"),allow_nan=False).replace("<","\\u003c")
    template = (Path(__file__).parent/"assets"/"replay.html").read_text()
    if template.count("__ATLAS_DATA__") != 1:
        raise ValueError("Replay template must have exactly one data placeholder")
    output = folder/"replay.html"
    output.write_text(template.replace("__ATLAS_DATA__",serialized))
    return output
