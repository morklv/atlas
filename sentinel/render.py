"""Portable image/GIF reports. Pillow keeps the first release small on a Mac."""
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont


BG = "#0b1220"
PANEL = "#142031"
TEXT = "#e7edf5"
MUTED = "#9eafc4"
TEAL = "#5ad6b2"
AMBER = "#f2ac66"


def font(size):
    for path in ("/System/Library/Fonts/Supplemental/Arial.ttf",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default(size=size)


def label(draw, xy, text, size=18, fill=TEXT):
    draw.text(xy, str(text), font=font(size), fill=fill)


def map_image(labels, pixels):
    colors = np.array([[35, 48, 65], [66, 146, 133], [242, 172, 102]], dtype=np.uint8)
    return Image.fromarray(colors[labels[::-1] + 1]).resize((pixels, pixels), Image.Resampling.NEAREST)


def render_frame(result, index):
    m = result.metadata
    event = m["events"][index]
    image = Image.new("RGB", (1280, 830), BG)
    d = ImageDraw.Draw(image)
    label(d, (32, 22), "ATLAS  /  AUTONOMOUS INSPECTION", 27)
    label(d, (32, 62), f"{m['policy'].upper()} POLICY   |   Scene {m['seed']}   |   {m['condition']} sensor   |   Observation {index+1}", 17, MUTED)
    d.rounded_rectangle((24, 104, 690, 770), radius=14, fill=PANEL)
    label(d, (45, 120), "WHAT THE DRONE HAS OBSERVED", 18)
    side = 560
    origin = (76, 166)
    image.paste(map_image(result.snapshots[index], side), origin)
    size = m["world_size"]
    def xy(p):
        return origin[0] + p["x"]/size*side, origin[1] + side - p["y"]/size*side
    poses = [e["pose"] for e in m["events"][:index+1]]
    if len(poses) > 1:
        d.line([xy(p) for p in poses], fill="#ecf2fc", width=3)
    for p in poses[:-1]:
        x,y = xy(p)
        d.ellipse((x-4,y-4,x+4,y+4), fill=TEXT)
    x,y = xy(poses[-1])
    a = poses[-1]["yaw"]
    triangle = [(x+14*np.cos(a), y-14*np.sin(a)),
                (x+9*np.cos(a+2.5), y-9*np.sin(a+2.5)),
                (x+9*np.cos(a-2.5), y-9*np.sin(a-2.5))]
    d.polygon(triangle, fill="#ffffff", outline="#122030")
    label(d, (78, 736), "Dark: unknown     Teal: free     Amber: occupied", 17, MUTED)
    d.rounded_rectangle((710, 104, 1256, 413), radius=14, fill=PANEL)
    label(d, (732, 120), "DEPTH SENSOR / RAY RANGE", 18)
    depth = result.depths[index]
    valid = np.isfinite(depth)
    z = np.nan_to_num(depth, nan=0.) / m["camera"]["far"]
    z = np.clip(z, 0, 1)
    rgb = np.stack([240*(1-z), 100+125*z, 70+155*z], axis=-1).astype(np.uint8)
    rgb[~valid] = [25, 32, 44]
    sensor = Image.fromarray(rgb).resize((300, 225), Image.Resampling.NEAREST)
    image.paste(sensor, (732, 162))
    label(d, (1050, 180), "Warm: near", 17, AMBER)
    label(d, (1050, 211), "Cool: far", 17, MUTED)
    label(d, (1050, 256), "No return:", 17, MUTED)
    label(d, (1050, 283), "still unknown", 17, MUTED)
    d.rounded_rectangle((710, 430, 1256, 770), radius=14, fill=PANEL)
    label(d, (732, 447), "MISSION EVIDENCE", 18)
    metrics = [
        ("Map observed", f"{event['coverage']:.1%}"),
        ("New cells this view", event["new_cells"]),
        ("Predicted new cells", event["predicted_new_cells"] if event["predicted_new_cells"] is not None else "initial view"),
        ("Flight distance so far", f"{event['distance_m']:.1f} m"),
        ("Simulated elapsed", f"{event['elapsed_seconds']:.1f} s"),
        ("Observed-cell accuracy*", f"{event['accuracy_observed']:.1%}" if event['accuracy_observed'] is not None else "n/a"),
    ]
    for i, (name, value) in enumerate(metrics):
        label(d, (732, 487+i*35), name, 17, MUTED)
        label(d, (1042, 487+i*35), value, 18, TEAL)
    label(d, (732, 713), "*Evaluator uses hidden truth; planner never does.", 15, MUTED)
    label(d, (32, 792), "KINEMATIC SENSOR SIMULATION  |  Exact pose  |  No flight dynamics  |  Depth geometry, not learned perception", 16, MUTED)
    return image


def render_result(result, folder):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    frames = [render_frame(result, i) for i in range(len(result.snapshots))]
    frames[-1].save(folder / "overview.png")
    frames[0].save(folder / "mission.gif", save_all=True, append_images=frames[1:],
                   duration=1100, loop=0)
    map_image(result.truth.astype(int), 576).save(folder / "evaluation_ground_truth.png")
    if (folder / "scene.json").exists():
        from .world import World
        world = World.load(folder / "scene.json")
        render_flight_replay(result, world, folder)
        from .web_replay import export_replay
        export_replay(result, world, folder)


def render_flight_replay(result, world, folder):
    """Isometric replay of recorded kinematic motion; never used as a sensor."""
    m = result.metadata
    frames = []
    scale = 12
    def project(xyz):
        x, y, z = xyz
        return (365 + (x-y)*scale, 235 + (x+y)*scale*.48 - z*scale*1.15)

    def frame_at(index, position, returning=False):
        image = Image.new("RGB", (1200, 750), BG)
        d = ImageDraw.Draw(image)
        label(d, (30, 20), "ATLAS / FLIGHT REPLAY", 28)
        label(d, (30, 62), f"One drone  |  {m['policy']} inspection  |  Lightweight geometric simulation", 17, MUTED)
        d.rounded_rectangle((20, 103, 705, 690), radius=12, fill=PANEL)
        label(d, (38, 119), "SIMULATOR SCENE / REPLAY ONLY", 18)
        ground = [project(p) for p in [(0,0,0),(world.size,0,0),(world.size,world.size,0),(0,world.size,0)]]
        d.polygon(ground, fill="#21384a", outline="#567283")
        for v in range(0, int(world.size)+1, 3):
            d.line([project((v,0,0)),project((v,world.size,0))], fill="#2d4658", width=1)
            d.line([project((0,v,0)),project((world.size,v,0))], fill="#2d4658", width=1)
        for b in sorted(world.boxes, key=lambda b: b.x+b.y):
            x,y,w,h,z = b.x,b.y,b.width,b.depth,b.height
            top = [(x,y,z),(x+w,y,z),(x+w,y+h,z),(x,y+h,z)]
            east = [(x+w,y,0),(x+w,y+h,0),(x+w,y+h,z),(x+w,y,z)]
            south = [(x,y+h,0),(x+w,y+h,0),(x+w,y+h,z),(x,y+h,z)]
            for face,color in ((east,"#ad714a"),(south,"#8e5b3f"),(top,"#d99c65")):
                d.polygon([project(p) for p in face], fill=color, outline="#1d2732")
        previous = [e["pose"] for e in m["events"][:index+1]]
        trail = [project((p["x"],p["y"],p["z"])) for p in previous]
        trail.append(project(position))
        if len(trail)>1:
            d.line(trail, fill="#8bd5c2", width=2)
        # Altitude drop line makes the constant-height flight assumption visible.
        cx,cy = project(position)
        gx,gy = project((position[0],position[1],0))
        d.line([(cx,cy),(gx,gy)], fill="#506779", width=1)
        d.ellipse((gx-4,gy-2,gx+4,gy+2), fill="#9ae4cf")
        for dx,dy in ((-.65,-.65),(.65,-.65),(.65,.65),(-.65,.65)):
            px,py = project((position[0]+dx,position[1]+dy,position[2]))
            d.line([(cx,cy),(px,py)], fill=TEXT, width=3)
            d.ellipse((px-7,py-3,px+7,py+3), outline=TEAL, width=2)
        d.ellipse((cx-5,cy-4,cx+5,cy+4), fill=TEXT)
        label(d, (38, 600), f"{'Returning to start' if returning else 'Inspecting / repositioning'}  |  z = {position[2]:.1f} m", 19, TEAL)
        label(d, (38, 638), "Scene geometry belongs to the simulator, not the planner.", 16, MUTED)
        d.rounded_rectangle((725, 103, 1180, 690), radius=12, fill=PANEL)
        label(d, (745, 119), "DRONE'S OBSERVED MAP", 18)
        image.paste(map_image(result.snapshots[index], 398), (753, 159))
        event = m["events"][index]
        label(d, (750, 580), f"Observed: {event['coverage']:.1%}    New cells: {event['new_cells']}", 19, TEAL)
        label(d, (750, 620), "Unknown space remains dark.", 17, MUTED)
        label(d, (30, 711), "No aerodynamics, motor control, learned perception, or physical-platform integration in this replay.", 16, MUTED)
        return image

    events = m["events"]
    for i, event in enumerate(events):
        p = event["pose"]
        end = np.array([p["x"],p["y"],p["z"]])
        if i:
            prev = events[i-1]["pose"]
            start = np.array([prev["x"],prev["y"],prev["z"]])
            for t in np.linspace(0,1,5,endpoint=False):
                frames.append(frame_at(i-1, start*(1-t)+end*t))
        frames.extend([frame_at(i,end)]*2)
    end = np.array(m["path_including_return"][-1])
    p = events[-1]["pose"]
    start = np.array([p["x"],p["y"],p["z"]])
    for t in np.linspace(0,1,7):
        frames.append(frame_at(len(events)-1, start*(1-t)+end*t, True))
    frames[-1].save(Path(folder) / "flight_replay.png")
    frames[0].save(Path(folder) / "flight_replay.gif", save_all=True, append_images=frames[1:],
                   duration=220, loop=0)


def render_comparison(rows, folder):
    image = Image.new("RGB", (1100, 690), BG)
    d = ImageDraw.Draw(image)
    label(d, (30, 24), "ATLAS / PAIRED EVALUATION", 28)
    label(d, (30, 66), "Same scenes, sensors and budgets. Error bars are not shown; inspect results.csv for every run.", 16, MUTED)
    conditions = list(dict.fromkeys(r["condition"] for r in rows))
    for j, condition in enumerate(conditions):
        x = 35 + j * 355
        label(d, (x, 125), condition.upper(), 20)
        for i, policy in enumerate(("fixed", "preplanned", "active")):
            selected = [r for r in rows if r["condition"] == condition and r["policy"] == policy]
            coverage = float(np.mean([r["coverage"] for r in selected]))
            distance = float(np.mean([r["flight_distance_m"] for r in selected]))
            y = 180 + i*140
            label(d, (x, y), f"{policy}: {coverage:.1%} mean coverage", 18, TEAL if policy == "active" else MUTED)
            d.rectangle((x, y+38, x+300, y+62), fill=PANEL)
            d.rectangle((x, y+38, x+300*coverage, y+62), fill=TEAL if policy == "active" else MUTED)
            label(d, (x, y+76), f"{distance:.1f} m mean flight | n={len(selected)}", 16, MUTED)
    label(d, (30, 640), "Simulation evidence only. Unknown cells remain unknown; accuracy is reported separately.", 17, MUTED)
    image.save(Path(folder) / "comparison.png")
