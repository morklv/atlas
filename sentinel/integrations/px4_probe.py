"""Read-only PX4 SITL telemetry probe. Never arms or commands a vehicle.

Requires MAVSDK-Python's 3.x API. This optional dependency is not installed by
the core package. Use only a locally running simulator for this project.
"""
import argparse
import asyncio
import json


async def probe(address, timeout):
    from mavsdk import System
    drone = System()
    await drone.connect(system_address=address)
    async def receive():
        async for state in drone.core.connection_state():
            if state.is_connected:
                break
        async for pv in drone.telemetry.position_velocity_ned():
            p = pv.position
            print(json.dumps({"connected": True, "position_ned_m": [p.north_m, p.east_m, p.down_m],
                              "position_enu_m": [p.east_m, p.north_m, -p.down_m],
                              "armed_by_probe": False}, indent=2))
            return
    await asyncio.wait_for(receive(), timeout)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--address", default="udpin://127.0.0.1:14540")
    parser.add_argument("--timeout", type=float, default=20.)
    args = parser.parse_args()
    try:
        asyncio.run(probe(args.address, args.timeout))
    except ImportError:
        parser.exit(1, "MAVSDK is not installed. See docs/GAZEBO.md for the separate integration environment.\n")
    except asyncio.TimeoutError:
        parser.exit(1, "No telemetry received before timeout. Check that local PX4 SITL is running.\n")


if __name__ == "__main__":
    main()
