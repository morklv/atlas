import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import platform
from .world import World
from .mission import MissionConfig, run_mission, load_result
from .render import render_result
from .sensor import CONDITIONS


def main():
    parser = argparse.ArgumentParser(description="ATLAS: single-drone active inspection")
    commands = parser.add_subparsers(dest="command", required=True)
    demo = commands.add_parser("demo", help="Run an inspection and save images, GIF and sensor data")
    demo.add_argument("--seed", type=int, default=7)
    demo.add_argument("--world", type=Path, help="Optional scene JSON instead of a generated scene")
    demo.add_argument("--policy", choices=("active", "fixed", "preplanned"), default="active")
    demo.add_argument("--condition", choices=CONDITIONS, default="nominal")
    demo.add_argument("--out", type=Path, default=Path("output/demo"))
    swarm = commands.add_parser("swarm", help="Run the three-drone shared-map field survey")
    swarm.add_argument("--seed", type=int, default=17)
    swarm.add_argument("--rounds", type=int, default=8)
    swarm.add_argument("--out", type=Path, default=Path("output/swarm"))
    swarm.add_argument("--independent", action="store_true", help="Disable view reservations for a comparison run")
    field_server = commands.add_parser("field-server", help="Serve the field app with local aerial image recognition")
    field_server.add_argument("--port", type=int, default=8766)
    bench = commands.add_parser("swarm-benchmark", help="Paired coordinated/independent synthetic swarm trials")
    bench.add_argument("--seeds", type=int, nargs="+", default=list(range(17,27)))
    bench.add_argument("--rounds", type=int, default=8)
    bench.add_argument("--out", type=Path, default=Path("output/swarm-benchmark.json"))
    ev = commands.add_parser("evaluate", help="Compare policies on identical scenarios")
    ev.add_argument("--seeds", type=int, nargs="+", default=list(range(100, 110)))
    ev.add_argument("--conditions", choices=CONDITIONS, nargs="+", default=list(CONDITIONS))
    ev.add_argument("--out", type=Path, default=Path("output/evaluation"))
    for command in (demo, ev):
        command.add_argument("--max-observations", type=int, default=10)
        command.add_argument("--time-budget", type=float, default=160.)
    render = commands.add_parser("render", help="Rebuild the visual report from a saved run")
    render.add_argument("folder", type=Path)
    commands.add_parser("doctor", help="Read-only check of available tools")
    commands.add_parser("test", help="Run the repository's geometry, mapping and mission checks")
    export = commands.add_parser("export-world", help="Export scene geometry as JSON and Gazebo SDF")
    export.add_argument("--seed", type=int, default=7)
    export.add_argument("--out", type=Path, default=Path("output/world"))
    args = parser.parse_args()
    if args.command == "test":
        import unittest
        suite = unittest.defaultTestLoader.discover(str(Path(__file__).resolve().parents[1] / "tests"))
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        raise SystemExit(0 if result.wasSuccessful() else 1)
    if args.command == "swarm":
        from .swarm import run_swarm, save_swarm
        payload=run_swarm(args.seed,args.rounds,coordinated=not args.independent)
        path=save_swarm(payload,args.out)
        print(json.dumps(payload["report"],indent=2))
        print(path.resolve())
        return
    if args.command == "field-server":
        import uvicorn
        if not (Path("output/swarm/field.html")).is_file():
            from .swarm import run_swarm, save_swarm
            save_swarm(run_swarm(), Path("output/swarm"))
        print(f"Open http://127.0.0.1:{args.port}/ in Safari", flush=True)
        uvicorn.run("sentinel.field_server:app", host="127.0.0.1", port=args.port)
        return
    if args.command == "swarm-benchmark":
        from .swarm import benchmark_swarm
        result=benchmark_swarm(args.seeds,args.rounds)
        args.out.parent.mkdir(parents=True,exist_ok=True)
        args.out.write_text(json.dumps(result,indent=2,allow_nan=False))
        print(json.dumps(result["summary"],indent=2))
        print(args.out.resolve())
        return
    if args.command == "doctor":
        report = {"platform": platform.platform(), "architecture": platform.machine(),
                  "python": platform.python_version(),
                  "numpy": importlib.util.find_spec("numpy") is not None,
                  "Pillow": importlib.util.find_spec("PIL") is not None,
                  "tools": {name: shutil.which(name) for name in ("gz", "brew", "cmake", "git")},
                  "free_disk_gb": round(shutil.disk_usage(".").free/1e9, 1),
                  "status": "Core uses Python/NumPy/Pillow. PX4/Gazebo are separate, unverified integrations."}
        print(json.dumps(report, indent=2))
        return
    if args.command == "render":
        render_result(load_result(args.folder), args.folder)
        return
    if args.command == "export-world":
        args.out.mkdir(parents=True, exist_ok=True)
        world = World.generated(args.seed)
        world.save(args.out / "scene.json")
        world.export_sdf(args.out / "sentinel.sdf")
        print(args.out.resolve())
        return
    config = MissionConfig(max_observations=args.max_observations, time_budget=args.time_budget)
    config.validate()
    if args.command == "evaluate":
        from .evaluate import evaluate
        evaluate(args.seeds, args.conditions, args.out, config)
    else:
        world = World.load(args.world) if args.world else World.generated(args.seed)
        result = run_mission(world, args.policy, args.condition, config)
        result.save(args.out)
        world.save(args.out / "scene.json")
        world.export_sdf(args.out / "sentinel.sdf")
        render_result(result, args.out)
        print(json.dumps(result.metadata["metrics"], indent=2))
    print(f"Saved to {args.out.resolve()}")


if __name__ == "__main__":
    main()
